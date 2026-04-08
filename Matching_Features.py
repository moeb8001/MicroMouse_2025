#New

#!/usr/bin/env python3
"""

"""

from __future__ import annotations

# ── Standard library ──────────────────────────────────────────────────────────
import csv, gc, json, logging, math, sys, time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, Rectangle
from PIL import Image as PilImage
from scipy.spatial import cKDTree


# ══════════════════════════════════════════════════════════════════════════════
#  ╔══ CONFIG — edit these, then click Run ═══════════════════════════════════╗
# ══════════════════════════════════════════════════════════════════════════════
SCRIPT_DIR = Path(__file__).resolve().parent

CONFIG: Dict[str, Any] = {

    # ── Input / output folders ───────────────────────────────────────────────
    "pre_folder":  str(SCRIPT_DIR / "pre"),
    "post_folder": str(SCRIPT_DIR / "post"),
    "results_dir": str(SCRIPT_DIR / "results_map_compare"),

    # ── Wafer physical geometry (SEMI M1-0302, 2" wafer) ─────────────────────
    "wafer_diameter_mm": 50.8,
    "flat_length_mm":    15.875,
    "flat_orientation":  "bottom",     # fallback if flat is not auto-detected

    # ── Memory management (huge JPEGs) ───────────────────────────────────────
    "use_memmap":  True,
    "memmap_dir":  str(SCRIPT_DIR / "memmap_cache"),

    # ── Tiling grid (per spec: 25 rows × 20 columns) ─────────────────────────
    "grid_rows": 25,
    "grid_cols": 20,
    # Each tile is processed with a small border of overlap so particles that
    # straddle a tile boundary are detected fully in BOTH neighbouring tiles
    # and then deduplicated globally below.
    "tile_overlap_px": 64,
    # Skip tiles that are less than this much on the active wafer area.
    "min_wafer_coverage": 0.20,

    # ── Per-tile illumination normalisation ──────────────────────────────────
    # We re-introduce CLAHE here.  The companion script avoids it because it
    # is run AFTER differencing where it amplifies seams; in this script the
    # detector runs PER TILE so CLAHE makes the local contrast inside each
    # tile uniform without ever crossing tile boundaries.
    "use_clahe":              True,
    "clahe_clip_limit":       2.0,
    "clahe_tile_grid":        8,         # CLAHE's own internal sub-tile grid
    # In addition we subtract the local background using a white top-hat
    # whose kernel is clearly larger than any plausible particle.
    "bg_tophat_kernel_px":    35,
    "gaussian_blur_ksize":    3,

    # ── Independent particle detection (per tile) ────────────────────────────
    # Threshold the top-hat residual at k * robust-sigma above the tile's
    # median.  Robust statistics → tolerant of bright outliers.
    "k_sigma":                4.5,
    "min_residual_floor":     6.0,
    "detect_bright":          True,
    "detect_dark":            False,
    "min_particle_area_px":   4,
    "max_particle_area_px":   5000,
    "min_circularity":        0.40,

    # ── Global deduplication across tile borders ─────────────────────────────
    # When two detections (from neighbouring overlapping tiles) lie within
    # this radius (µm) they are merged into one.
    "dedup_radius_um":        4.0,

    # ── Cluster-aware comparison ─────────────────────────────────────────────
    # We allow the AFTER particle map to be shifted by a small RIGID
    # translation relative to BEFORE before we compare them.  This absorbs
    # residual misregistration of the stitched mosaic.
    "global_shift_search_um": 25.0,      # ± search range (µm)
    "global_shift_step_um":   1.0,       # search granularity (µm)
    # After the global shift is applied, two particles within this distance
    # (µm) are considered the same physical particle.
    "match_radius_um":        6.0,
    # Cluster definition: build connected components of the union of the two
    # maps with this linking distance.  Within each cluster, run a local
    # rigid-shift refinement (the cluster may have moved a tiny bit on top of
    # the global shift, e.g. one stitching cell is offset).
    "cluster_link_um":        80.0,
    "local_shift_search_um":  15.0,
    "local_shift_step_um":    1.0,

    # ── Edge exclusion ───────────────────────────────────────────────────────
    "edge_exclusion_mm":      2.5,

    # ── Output ───────────────────────────────────────────────────────────────
    "figure_max_panel_px":    1800,
    "dpi":                    180,
    "save_csv":               True,
    "save_json":              True,
}


# ══════════════════════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════════════════════
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s  %(levelname)-8s  %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("particle_map_compare")

PilImage.MAX_IMAGE_PIXELS = None


# ══════════════════════════════════════════════════════════════════════════════
#  DATA CLASSES
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class Particle:
    """A single detection in a single image, in wafer-µm coordinates."""
    id:           int   = -1
    x_px:         float = 0.0     # full-image pixel column
    y_px:         float = 0.0     # full-image pixel row
    x_um:         float = 0.0     # wafer-centred (Y-up)
    y_um:         float = 0.0
    area_px:      int   = 0
    circularity:  float = 0.0
    peak_residual: float = 0.0
    tile_label:   str   = ""

    # Set after comparison:
    status:       str   = "unchanged"   # unchanged | added | removed
    cluster_id:   int   = -1
    match_dist_um: float = 0.0


@dataclass
class TileSpec:
    row: int; col: int; label: str
    x1: int;  y1: int;  x2: int;  y2: int       # inner bounds
    ox1: int; oy1: int; ox2: int; oy2: int      # outer (with overlap)
    coverage: float


# ══════════════════════════════════════════════════════════════════════════════
#  LAZY IMAGE  (memmap or RAM, uniform interface)
# ══════════════════════════════════════════════════════════════════════════════
class LazyImage:
    """Provides .crop(x1,y1,x2,y2) on a possibly-huge grayscale image without
    requiring the whole thing in RAM."""

    def __init__(self, jpeg_path: str, use_memmap: bool, memmap_dir: str):
        self.path   = Path(jpeg_path)
        self.use_mm = use_memmap
        self.mm_dir = Path(memmap_dir)
        self._arr: Optional[np.ndarray] = None
        self._shape: Optional[Tuple[int, int]] = None

    def open(self) -> None:
        if self.use_mm:
            self._arr = self._get_or_make_memmap()
        else:
            logger.info("Loading %s into RAM ...", self.path.name)
            arr = cv2.imread(str(self.path), cv2.IMREAD_GRAYSCALE)
            if arr is None:
                raise FileNotFoundError(str(self.path))
            self._arr = arr
        self._shape = self._arr.shape
        logger.info("  %s shape=%s backend=%s (%.0f MB)",
                    self.path.name, self._shape,
                    "memmap" if self.use_mm else "RAM",
                    self._arr.nbytes / 1e6)

    @property
    def shape(self) -> Tuple[int, int]:
        if self._shape is None:
            raise RuntimeError("call .open() first")
        return self._shape

    def crop(self, x1: int, y1: int, x2: int, y2: int) -> np.ndarray:
        if self._arr is None:
            raise RuntimeError("call .open() first")
        H, W = self._shape
        cx1, cy1 = max(0, x1), max(0, y1)
        cx2, cy2 = min(W, x2), min(H, y2)
        out = np.zeros((y2 - y1, x2 - x1), dtype=np.uint8)
        if cx1 < cx2 and cy1 < cy2:
            region = np.asarray(self._arr[cy1:cy2, cx1:cx2])
            out[cy1 - y1:cy2 - y1, cx1 - x1:cx2 - x1] = region
        return out

    def thumbnail(self, max_side: int = 2000) -> np.ndarray:
        H, W = self._shape
        ds = max(1.0, max(H, W) / max_side)
        sw, sh = int(W / ds), int(H / ds)
        try:
            img = PilImage.open(str(self.path))
            img.draft("L", (sw, sh))
            img = img.convert("L").resize((sw, sh), PilImage.LANCZOS)
            thumb = np.asarray(img, dtype=np.uint8)
            img.close()
            return thumb
        except Exception:
            step = max(1, int(ds))
            return self._arr[::step, ::step].copy()

    def close(self) -> None:
        self._arr = None
        gc.collect()

    def _get_or_make_memmap(self) -> np.memmap:
        self.mm_dir.mkdir(parents=True, exist_ok=True)
        stem_key = (f"{self.path.stem}_"
                    f"{abs(hash(str(self.path.resolve()))) & 0xFFFFFFFF:08x}")
        dat   = self.mm_dir / f"{stem_key}.dat"
        shape = self.mm_dir / f"{stem_key}.shape.npy"
        if dat.exists() and shape.exists():
            sh = tuple(np.load(shape).tolist())
            logger.info("  reusing memmap cache: %s shape=%s", dat.name, sh)
            return np.memmap(dat, dtype=np.uint8, mode="r", shape=sh)

        logger.info("  decoding %s to memmap (one-time) ...", self.path.name)
        t0 = time.time()
        arr = cv2.imread(str(self.path), cv2.IMREAD_GRAYSCALE)
        if arr is None:
            raise FileNotFoundError(str(self.path))
        sh = arr.shape
        mm = np.memmap(dat, dtype=np.uint8, mode="w+", shape=sh)
        mm[:] = arr[:]
        mm.flush()
        np.save(shape, np.array(sh))
        del arr, mm
        gc.collect()
        logger.info("  memmap ready in %.1fs", time.time() - t0)
        return np.memmap(dat, dtype=np.uint8, mode="r", shape=sh)


# ══════════════════════════════════════════════════════════════════════════════
#  WAFER GEOMETRY
# ══════════════════════════════════════════════════════════════════════════════
class WaferGeometry:
    """Wafer disc + primary flat, with px ↔ µm conversion."""

    def __init__(self, cfg: Dict[str, Any], img_shape: Tuple[int, int],
                 center_px: Tuple[float, float], radius_px: float,
                 flat_angle_rad: float):
        self.cfg = cfg
        self.H, self.W = img_shape
        self.cx, self.cy = center_px
        self.radius_px = radius_px
        self.flat_angle = flat_angle_rad

        self.diameter_mm   = cfg["wafer_diameter_mm"]
        self.flat_length_mm = cfg["flat_length_mm"]
        self.px_per_mm = (2.0 * radius_px) / self.diameter_mm
        self.um_per_px = 1000.0 / self.px_per_mm
        self.edge_excl_px = int(cfg["edge_exclusion_mm"] * self.px_per_mm)

        r_mm = self.diameter_mm / 2.0
        half_flat = self.flat_length_mm / 2.0
        self.flat_dist_mm = math.sqrt(max(0.0, r_mm**2 - half_flat**2))
        self.flat_dist_px = self.flat_dist_mm * self.px_per_mm

        logger.info("  wafer: c=(%.0f,%.0f) R=%.0f px  %.2f px/mm  flat=%.1f°",
                    self.cx, self.cy, self.radius_px, self.px_per_mm,
                    math.degrees(flat_angle_rad))

    def build_active_mask(self, downsample: int = 1) -> np.ndarray:
        H = self.H // downsample
        W = self.W // downsample
        cx = self.cx / downsample
        cy = self.cy / downsample
        R = (self.radius_px - self.edge_excl_px) / downsample
        yy, xx = np.ogrid[:H, :W]
        circle = (xx - cx) ** 2 + (yy - cy) ** 2 <= R ** 2
        nx, ny = math.cos(self.flat_angle), math.sin(self.flat_angle)
        sd = (xx - cx) * nx + (yy - cy) * ny
        active = circle & (sd < (self.flat_dist_px - self.edge_excl_px) / downsample)
        return active.astype(np.uint8) * 255

    def is_point_active(self, col: float, row: float) -> bool:
        dx = col - self.cx
        dy = row - self.cy
        if dx * dx + dy * dy > (self.radius_px - self.edge_excl_px) ** 2:
            return False
        nx, ny = math.cos(self.flat_angle), math.sin(self.flat_angle)
        sd = dx * nx + dy * ny
        return sd < (self.flat_dist_px - self.edge_excl_px)

    def px_to_um(self, col: float, row: float) -> Tuple[float, float]:
        x_um = (col - self.cx) * self.um_per_px
        y_um = -(row - self.cy) * self.um_per_px
        return x_um, y_um


# ══════════════════════════════════════════════════════════════════════════════
#  AUTO-DETECT wafer (white-wipe-aware)
# ══════════════════════════════════════════════════════════════════════════════
def detect_wafer_on_thumbnail(lazy: LazyImage, cfg: Dict[str, Any]
                              ) -> Tuple[Tuple[float, float], float, float]:
    """Find the wafer disc + primary flat in a downsampled thumbnail.

    The wafer is on a WHITE WIPE inside a black-cornered scan area:
        outside scan = BLACK,  wipe = BRIGHT,  silicon wafer = DARKER
    so the wafer is the largest dark connected component that does NOT touch
    any image border.
    """
    full_H, full_W = lazy.shape
    thumb = lazy.thumbnail(max_side=2000)
    th, tw = thumb.shape
    sx = full_W / tw
    sy = full_H / th

    blur = cv2.GaussianBlur(thumb, (9, 9), 0)
    otsu_val, _ = cv2.threshold(blur, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, bright = cv2.threshold(blur, otsu_val, 255, cv2.THRESH_BINARY)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, k, iterations=2)
    dark = 255 - bright

    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    H_t, W_t = thumb.shape
    wafer_label = -1
    wafer_area = 0
    for i in range(1, n_lab):
        x, y, w, h, area = stats[i]
        if x == 0 or y == 0 or x + w >= W_t or y + h >= H_t:
            continue
        if area > wafer_area:
            wafer_area = int(area)
            wafer_label = i

    if wafer_label < 0:
        logger.warning("Wipe not fully framing wafer — using bright-blob fallback.")
        contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise RuntimeError("Wafer not found in thumbnail — check input image")
        wafer_cnt = max(contours, key=cv2.contourArea)
    else:
        m = (labels == wafer_label).astype(np.uint8) * 255
        m = cv2.morphologyEx(
            m, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
            iterations=2)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise RuntimeError("Wafer mask empty after cleanup")
        wafer_cnt = max(contours, key=cv2.contourArea)

    (cx_t, cy_t), r_t = cv2.minEnclosingCircle(wafer_cnt)

    # Detect primary flat as the indented arc
    pts = wafer_cnt.reshape(-1, 2).astype(np.float64)
    dists = np.hypot(pts[:, 0] - cx_t, pts[:, 1] - cy_t)
    indent = r_t - dists
    flat_mask = indent > r_t * 0.015
    if flat_mask.sum() > 10:
        fpts = pts[flat_mask]
        fmid = fpts.mean(axis=0)
        flat_angle = float(math.atan2(fmid[1] - cy_t, fmid[0] - cx_t))
    else:
        orient = cfg.get("flat_orientation", "bottom")
        flat_angle = {"bottom": math.pi / 2, "top": -math.pi / 2,
                      "left": math.pi, "right": 0.0}.get(orient, math.pi / 2)
        logger.warning("Flat not detected — using config orientation")

    cx_full = cx_t * sx
    cy_full = cy_t * sy
    r_full  = r_t * (sx + sy) / 2.0
    return (cx_full, cy_full), r_full, flat_angle


# ══════════════════════════════════════════════════════════════════════════════
#  GRID
# ══════════════════════════════════════════════════════════════════════════════
def build_tile_grid(geom: WaferGeometry, cfg: Dict[str, Any]) -> List[TileSpec]:
    H, W = geom.H, geom.W
    nr, nc = cfg["grid_rows"], cfg["grid_cols"]
    ov = cfg["tile_overlap_px"]

    cell_w = W / nc
    cell_h = H / nr

    ds = 8
    mask_small = geom.build_active_mask(downsample=ds)

    tiles: List[TileSpec] = []
    for r in range(nr):
        for c in range(nc):
            x1 = int(round(c * cell_w));  x2 = int(round((c + 1) * cell_w))
            y1 = int(round(r * cell_h));  y2 = int(round((r + 1) * cell_h))
            ox1 = max(0, x1 - ov); oy1 = max(0, y1 - ov)
            ox2 = min(W, x2 + ov); oy2 = min(H, y2 + ov)
            x1s, y1s = x1 // ds, y1 // ds
            x2s, y2s = max(x1s + 1, x2 // ds), max(y1s + 1, y2 // ds)
            sub = mask_small[y1s:y2s, x1s:x2s]
            cov = float((sub > 0).mean()) if sub.size > 0 else 0.0
            if cov < cfg["min_wafer_coverage"]:
                continue
            tiles.append(TileSpec(
                row=r, col=c, label=f"R{r:02d}C{c:02d}",
                x1=x1, y1=y1, x2=x2, y2=y2,
                ox1=ox1, oy1=oy1, ox2=ox2, oy2=oy2,
                coverage=cov))
    logger.info("Grid: %d active tiles / %d total (%dx%d)",
                len(tiles), nr * nc, nr, nc)
    return tiles


# ══════════════════════════════════════════════════════════════════════════════
#  INDEPENDENT PER-TILE PARTICLE DETECTION
# ══════════════════════════════════════════════════════════════════════════════
def _robust_sigma(values: np.ndarray) -> float:
    if values.size == 0:
        return 1.0
    med = float(np.median(values))
    mad = float(np.median(np.abs(values - med)))
    return max(1.0, 1.4826 * mad)


def normalise_tile(gray: np.ndarray, mask: np.ndarray,
                   cfg: Dict[str, Any]) -> np.ndarray:
    """Per-tile illumination normalisation:
        1. CLAHE (local contrast equalisation, INSIDE the tile only)
        2. Gentle Gaussian smoothing
    Returns a uint8 image suitable for top-hat detection.
    """
    out = gray.copy()
    if cfg.get("use_clahe", True):
        clip = float(cfg.get("clahe_clip_limit", 2.0))
        ts   = int(cfg.get("clahe_tile_grid", 8))
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(ts, ts))
        out = clahe.apply(out)
    k = int(cfg.get("gaussian_blur_ksize", 3)) | 1
    if k >= 3:
        out = cv2.GaussianBlur(out, (k, k), 0)
    out[mask == 0] = 0
    return out


def detect_particles_in_tile(gray: np.ndarray, mask: np.ndarray,
                             cfg: Dict[str, Any]
                             ) -> List[Tuple[float, float, int, float, float]]:
    """Return list of (cx_local, cy_local, area, circularity, peak) detections
    for one tile (already normalised).  Coordinates are in tile-local pixels.
    """
    valid = mask > 0
    if valid.sum() < 200:
        return []

    bg_k = int(cfg["bg_tophat_kernel_px"]) | 1
    se   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bg_k, bg_k))

    detections: List[Tuple[float, float, int, float, float]] = []

    for enabled, op in [
        (cfg["detect_bright"], cv2.MORPH_TOPHAT),
        (cfg["detect_dark"],   cv2.MORPH_BLACKHAT),
    ]:
        if not enabled:
            continue
        residual = cv2.morphologyEx(gray, op, se).astype(np.float32)
        residual[~valid] = 0.0

        v = residual[valid]
        sigma = _robust_sigma(v)
        thr = max(float(cfg["k_sigma"]) * sigma,
                  float(cfg["min_residual_floor"]))

        bw = (residual > thr).astype(np.uint8) * 255
        bw[~valid] = 0
        ok = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN,  ok)
        bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, ok)

        contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = int(cv2.contourArea(cnt))
            if area < cfg["min_particle_area_px"] or area > cfg["max_particle_area_px"]:
                continue
            perim = cv2.arcLength(cnt, True)
            circ = (4 * math.pi * area / (perim * perim)) if perim > 0 else 0.0
            if circ < cfg["min_circularity"]:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            ix, iy = int(round(cx)), int(round(cy))
            h, w = residual.shape
            if 0 <= iy < h and 0 <= ix < w:
                peak = float(residual[iy, ix])
            else:
                peak = 0.0
            detections.append((cx, cy, area, circ, peak))
    return detections


def detect_all_particles(lazy: LazyImage, geom: WaferGeometry,
                         tiles: List[TileSpec], cfg: Dict[str, Any],
                         label: str) -> List[Particle]:
    """Build a complete particle map for one image, in wafer-µm coordinates."""
    logger.info("Detecting particles in %s ...", label)
    all_p: List[Particle] = []
    pid = 0
    t0 = time.time()
    for k, tile in enumerate(tiles):
        gray = lazy.crop(tile.ox1, tile.oy1, tile.ox2, tile.oy2)
        # active wafer mask for this outer crop
        h, w = gray.shape
        yy, xx = np.ogrid[tile.oy1:tile.oy1 + h, tile.ox1:tile.ox1 + w]
        nx, ny = math.cos(geom.flat_angle), math.sin(geom.flat_angle)
        sd = (xx - geom.cx) * nx + (yy - geom.cy) * ny
        circ_ok = ((xx - geom.cx) ** 2 + (yy - geom.cy) ** 2 <=
                   (geom.radius_px - geom.edge_excl_px) ** 2)
        flat_ok = sd < (geom.flat_dist_px - geom.edge_excl_px)
        mask = (circ_ok & flat_ok).astype(np.uint8) * 255

        norm = normalise_tile(gray, mask, cfg)
        dets = detect_particles_in_tile(norm, mask, cfg)

        for (cx_loc, cy_loc, area, circ, peak) in dets:
            cx_full = cx_loc + tile.ox1
            cy_full = cy_loc + tile.oy1
            # Drop detections that fell in the OVERLAP band of this tile
            # AND lie inside a neighbouring tile's INNER cell — those will
            # be picked up by the neighbour, and the global dedup pass will
            # collapse exact duplicates anyway.  But we keep them here so
            # particles that strictly straddle the boundary aren't lost.
            x_um, y_um = geom.px_to_um(cx_full, cy_full)
            all_p.append(Particle(
                id=pid, x_px=cx_full, y_px=cy_full,
                x_um=x_um, y_um=y_um,
                area_px=area, circularity=circ, peak_residual=peak,
                tile_label=tile.label))
            pid += 1
        if (k + 1) % 50 == 0 or k + 1 == len(tiles):
            logger.info("  %s tile %d/%d  total=%d  (%.1fs)",
                        label, k + 1, len(tiles), len(all_p), time.time() - t0)
    logger.info("  %s: %d raw detections", label, len(all_p))
    all_p = dedup_particles(all_p, cfg)
    logger.info("  %s: %d particles after dedup", label, len(all_p))
    return all_p


def dedup_particles(parts: List[Particle], cfg: Dict[str, Any]) -> List[Particle]:
    """Merge duplicate detections from neighbouring overlapping tiles."""
    if not parts:
        return parts
    pts = np.array([(p.x_um, p.y_um) for p in parts], dtype=np.float32)
    tree = cKDTree(pts)
    radius = float(cfg["dedup_radius_um"])
    keep = np.ones(len(parts), dtype=bool)
    for i in range(len(parts)):
        if not keep[i]:
            continue
        idx = tree.query_ball_point(pts[i], r=radius)
        for j in idx:
            if j == i or not keep[j]:
                continue
            # keep the brighter / larger detection
            if (parts[j].peak_residual, parts[j].area_px) > \
               (parts[i].peak_residual, parts[i].area_px):
                keep[i] = False
                break
            else:
                keep[j] = False
    out = [p for p, k in zip(parts, keep) if k]
    for new_id, p in enumerate(out):
        p.id = new_id
    return out


# ══════════════════════════════════════════════════════════════════════════════
#  CLUSTER-AWARE PARTICLE-MAP COMPARISON
# ══════════════════════════════════════════════════════════════════════════════
def _grid_search_shift(src_xy: np.ndarray, dst_xy: np.ndarray,
                       search_um: float, step_um: float,
                       match_radius_um: float
                       ) -> Tuple[Tuple[float, float], int]:
    """Find the (dx, dy) shift that maximises the number of matched particles
    when src_xy is shifted by (dx, dy) and compared to dst_xy.

    Brute-force grid search; cheap because lists are small after dedup.
    Returns ((dx, dy), best_match_count).
    """
    if len(src_xy) == 0 or len(dst_xy) == 0:
        return (0.0, 0.0), 0
    dst_tree = cKDTree(dst_xy)
    best = (0.0, 0.0)
    best_n = -1
    n_steps = int(round(search_um / step_um))
    for iy in range(-n_steps, n_steps + 1):
        for ix in range(-n_steps, n_steps + 1):
            dx, dy = ix * step_um, iy * step_um
            shifted = src_xy + np.array([dx, dy], dtype=np.float32)
            d, _ = dst_tree.query(shifted, distance_upper_bound=match_radius_um)
            n = int(np.sum(np.isfinite(d)))
            if n > best_n:
                best_n = n
                best = (dx, dy)
    return best, best_n


def compare_particle_maps(before: List[Particle], after: List[Particle],
                          cfg: Dict[str, Any]
                          ) -> Tuple[List[Particle], List[Particle], List[Particle],
                                     Dict[str, Any]]:
    """Cluster-aware comparison of two particle maps.

    Steps:
        1. Global rigid-shift search:  find (dx, dy) that maximises the
           number of matches when AFTER is shifted onto BEFORE.
        2. Build clusters by linking the union of (BEFORE + shifted-AFTER)
           with `cluster_link_um`.  Each cluster represents a local
           neighbourhood of particles likely belonging together.
        3. Per-cluster local rigid-shift refinement (some sub-regions of the
           mosaic may be offset slightly differently from the global shift).
        4. Within each refined cluster, KD-tree match BEFORE ↔ AFTER:
                matched pairs        → unchanged
                unmatched in BEFORE  → removed
                unmatched in AFTER   → added

    Returns (added, removed, unchanged, info_dict).
    """
    info: Dict[str, Any] = {}

    if not before and not after:
        return [], [], [], {"global_shift_um": (0.0, 0.0),
                             "global_match_count": 0,
                             "n_clusters": 0,
                             "max_local_shift_um": 0.0}

    b_xy = np.array([(p.x_um, p.y_um) for p in before], dtype=np.float32) \
           if before else np.zeros((0, 2), dtype=np.float32)
    a_xy = np.array([(p.x_um, p.y_um) for p in after], dtype=np.float32) \
           if after else np.zeros((0, 2), dtype=np.float32)

    # ── 1. global shift  (shift AFTER onto BEFORE) ───────────────────────────
    (gdx, gdy), gn = _grid_search_shift(
        a_xy, b_xy,
        search_um=float(cfg["global_shift_search_um"]),
        step_um=float(cfg["global_shift_step_um"]),
        match_radius_um=float(cfg["match_radius_um"]))
    info["global_shift_um"] = (gdx, gdy)
    info["global_match_count"] = gn
    logger.info("  global shift: dx=%+.2f µm  dy=%+.2f µm  matches=%d",
                gdx, gdy, gn)

    a_shift = a_xy + np.array([gdx, gdy], dtype=np.float32)

    # ── 2. cluster the union ────────────────────────────────────────────────
    union = np.vstack([b_xy, a_shift]) if (len(b_xy) and len(a_shift)) \
            else (b_xy if len(b_xy) else a_shift)
    src_flag = np.concatenate([
        np.zeros(len(b_xy), dtype=np.int8),       # 0 = before
        np.ones(len(a_shift), dtype=np.int8),     # 1 = after
    ])
    src_index = np.concatenate([
        np.arange(len(b_xy), dtype=np.int32),
        np.arange(len(a_shift), dtype=np.int32),
    ])

    link = float(cfg["cluster_link_um"])
    cluster_id = -np.ones(len(union), dtype=np.int32)

    if len(union) > 0:
        u_tree = cKDTree(union)
        next_id = 0
        for i in range(len(union)):
            if cluster_id[i] != -1:
                continue
            # BFS
            stack = [i]
            cluster_id[i] = next_id
            while stack:
                k = stack.pop()
                neigh = u_tree.query_ball_point(union[k], r=link)
                for j in neigh:
                    if cluster_id[j] == -1:
                        cluster_id[j] = next_id
                        stack.append(j)
            next_id += 1
        info["n_clusters"] = int(next_id)
    else:
        info["n_clusters"] = 0

    # ── 3 & 4. per-cluster local refinement + matching ──────────────────────
    matched_b = np.zeros(len(b_xy), dtype=bool)
    matched_a = np.zeros(len(a_xy), dtype=bool)
    match_dist = np.zeros(len(b_xy), dtype=np.float32)
    a_match_dist = np.zeros(len(a_xy), dtype=np.float32)
    cl_id_b = -np.ones(len(b_xy), dtype=np.int32)
    cl_id_a = -np.ones(len(a_xy), dtype=np.int32)

    max_local_shift = 0.0

    for cid in range(info["n_clusters"]):
        members = np.where(cluster_id == cid)[0]
        b_members = src_index[members[src_flag[members] == 0]]
        a_members = src_index[members[src_flag[members] == 1]]
        cl_id_b[b_members] = cid
        cl_id_a[a_members] = cid

        if len(b_members) == 0 or len(a_members) == 0:
            continue

        b_pts = b_xy[b_members]
        a_pts = a_xy[a_members] + np.array([gdx, gdy], dtype=np.float32)

        # local refinement (only worth it for clusters with several points)
        if len(b_members) >= 2 and len(a_members) >= 2:
            (ldx, ldy), _ = _grid_search_shift(
                a_pts, b_pts,
                search_um=float(cfg["local_shift_search_um"]),
                step_um=float(cfg["local_shift_step_um"]),
                match_radius_um=float(cfg["match_radius_um"]))
        else:
            ldx, ldy = 0.0, 0.0
        max_local_shift = max(max_local_shift, math.hypot(ldx, ldy))
        a_pts_ref = a_pts + np.array([ldx, ldy], dtype=np.float32)

        # mutual nearest-neighbour matching within the match radius
        b_tree = cKDTree(b_pts)
        a_tree = cKDTree(a_pts_ref)
        d_a, idx_b_for_a = b_tree.query(
            a_pts_ref, distance_upper_bound=float(cfg["match_radius_um"]))
        d_b, idx_a_for_b = a_tree.query(
            b_pts, distance_upper_bound=float(cfg["match_radius_um"]))

        for ai, bi in enumerate(idx_b_for_a):
            if not np.isfinite(d_a[ai]):
                continue
            # mutual NN check
            if idx_a_for_b[bi] == ai and np.isfinite(d_b[bi]):
                global_b = b_members[bi]
                global_a = a_members[ai]
                if not matched_b[global_b] and not matched_a[global_a]:
                    matched_b[global_b] = True
                    matched_a[global_a] = True
                    match_dist[global_b] = d_a[ai]
                    a_match_dist[global_a] = d_a[ai]

    info["max_local_shift_um"] = float(max_local_shift)

    unchanged: List[Particle] = []
    removed: List[Particle]   = []
    added: List[Particle]     = []

    for i, p in enumerate(before):
        p.cluster_id = int(cl_id_b[i])
        if matched_b[i]:
            p.status = "unchanged"
            p.match_dist_um = float(match_dist[i])
            unchanged.append(p)
        else:
            p.status = "removed"
            removed.append(p)

    for j, p in enumerate(after):
        p.cluster_id = int(cl_id_a[j])
        if matched_a[j]:
            # already represented by its before-counterpart in `unchanged`
            p.status = "unchanged"
            p.match_dist_um = float(a_match_dist[j])
        else:
            p.status = "added"
            added.append(p)

    return added, removed, unchanged, info


# ══════════════════════════════════════════════════════════════════════════════
#  REVIEW: flagged tiles & caveats
# ══════════════════════════════════════════════════════════════════════════════
def flag_suspicious_tiles(tiles: List[TileSpec],
                          added: List[Particle], removed: List[Particle],
                          before: List[Particle], after: List[Particle],
                          cfg: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Identify tiles that contain a suspiciously large fraction of the
    Added/Removed counts (likely registration or seam artefacts)."""
    by_tile: Dict[str, Dict[str, int]] = {
        t.label: {"added": 0, "removed": 0, "before": 0, "after": 0}
        for t in tiles
    }
    for p in added:
        by_tile.setdefault(p.tile_label, {"added": 0, "removed": 0,
                                           "before": 0, "after": 0})["added"] += 1
    for p in removed:
        by_tile.setdefault(p.tile_label, {"added": 0, "removed": 0,
                                           "before": 0, "after": 0})["removed"] += 1
    for p in before:
        by_tile.setdefault(p.tile_label, {"added": 0, "removed": 0,
                                           "before": 0, "after": 0})["before"] += 1
    for p in after:
        by_tile.setdefault(p.tile_label, {"added": 0, "removed": 0,
                                           "before": 0, "after": 0})["after"] += 1

    total_added = max(1, len(added))
    total_removed = max(1, len(removed))

    flagged = []
    for label, counts in by_tile.items():
        # heuristic: a single tile holding > 15 % of the Added or Removed
        # population, or holding both > 5 added AND > 5 removed (a tell-tale
        # sign of misalignment), gets flagged.
        share_a = counts["added"] / total_added
        share_r = counts["removed"] / total_removed
        if (share_a > 0.15 and counts["added"] >= 5) \
           or (share_r > 0.15 and counts["removed"] >= 5) \
           or (counts["added"] >= 5 and counts["removed"] >= 5):
            flagged.append({
                "tile":    label,
                "added":   counts["added"],
                "removed": counts["removed"],
                "before":  counts["before"],
                "after":   counts["after"],
                "reason":  ("high concentration of changes — verify "
                            "registration / illumination in this tile"),
            })
    flagged.sort(key=lambda d: d["added"] + d["removed"], reverse=True)
    return flagged


# ══════════════════════════════════════════════════════════════════════════════
#  3-PANEL FIGURE  (Before | After | Difference)
# ══════════════════════════════════════════════════════════════════════════════
def _downsample_for_panel(lazy: LazyImage, max_side: int) -> Tuple[np.ndarray, float]:
    H, W = lazy.shape
    ds = max(1.0, max(H, W) / max_side)
    sw, sh = int(W / ds), int(H / ds)
    try:
        img = PilImage.open(str(lazy.path))
        img.draft("L", (sw, sh))
        img = img.convert("L").resize((sw, sh), PilImage.LANCZOS)
        thumb = np.asarray(img, dtype=np.uint8)
        img.close()
    except Exception:
        step = max(1, int(ds))
        thumb = np.asarray(lazy._arr[::step, ::step]).copy()
        sh, sw = thumb.shape
    return thumb, ds


def _draw_wafer_outline(ax, geom: WaferGeometry, ds: float):
    cx = geom.cx / ds
    cy = geom.cy / ds
    R  = geom.radius_px / ds
    # Wafer disc
    theta = np.linspace(0, 2 * np.pi, 720)
    ax.plot(cx + R * np.cos(theta), cy + R * np.sin(theta),
            color="cyan", lw=0.8, alpha=0.7)
    # Edge-exclusion ring
    R2 = (geom.radius_px - geom.edge_excl_px) / ds
    ax.plot(cx + R2 * np.cos(theta), cy + R2 * np.sin(theta),
            color="cyan", lw=0.4, alpha=0.4, linestyle="--")
    # Centre cross
    ax.plot([cx - 8, cx + 8], [cy, cy], color="cyan", lw=0.5)
    ax.plot([cx, cx], [cy - 8, cy + 8], color="cyan", lw=0.5)


def render_three_panel(lazy_b: LazyImage, lazy_a: LazyImage,
                       geom: WaferGeometry,
                       before: List[Particle], after: List[Particle],
                       added: List[Particle], removed: List[Particle],
                       unchanged: List[Particle],
                       info: Dict[str, Any], cfg: Dict[str, Any],
                       out_path: Path, wafer_name: str):
    """Render the Before / After / Difference figure."""
    max_side = int(cfg["figure_max_panel_px"])
    thumb_b, ds_b = _downsample_for_panel(lazy_b, max_side)
    thumb_a, ds_a = _downsample_for_panel(lazy_a, max_side)
    # use a single ds for the difference panel — pick the larger of the two
    ds = max(ds_b, ds_a)
    # rebuild thumbnails at common scale if needed
    H, W = lazy_b.shape
    sw, sh = int(W / ds), int(H / ds)
    if thumb_b.shape != (sh, sw):
        thumb_b = cv2.resize(thumb_b, (sw, sh), interpolation=cv2.INTER_AREA)
    if thumb_a.shape != (sh, sw):
        thumb_a = cv2.resize(thumb_a, (sw, sh), interpolation=cv2.INTER_AREA)

    fig, axes = plt.subplots(1, 3, figsize=(18, 7), dpi=cfg["dpi"])
    titles = ["BEFORE", "AFTER", "DIFFERENCE"]

    # --- BEFORE panel ---
    ax = axes[0]
    ax.imshow(thumb_b, cmap="gray", interpolation="nearest")
    if before:
        bx = [p.x_px / ds for p in before]
        by = [p.y_px / ds for p in before]
        ax.scatter(bx, by, s=8, facecolors="none", edgecolors="yellow",
                   linewidths=0.6, label=f"{len(before)} particles")
    _draw_wafer_outline(ax, geom, ds)
    ax.set_title(f"{titles[0]}  —  {len(before)} particles")
    ax.set_xticks([]); ax.set_yticks([])
    if before:
        ax.legend(loc="lower right", fontsize=8, framealpha=0.7)

    # --- AFTER panel ---
    ax = axes[1]
    ax.imshow(thumb_a, cmap="gray", interpolation="nearest")
    if after:
        ax_x = [p.x_px / ds for p in after]
        ax_y = [p.y_px / ds for p in after]
        ax.scatter(ax_x, ax_y, s=8, facecolors="none", edgecolors="yellow",
                   linewidths=0.6, label=f"{len(after)} particles")
    _draw_wafer_outline(ax, geom, ds)
    ax.set_title(f"{titles[1]}  —  {len(after)} particles")
    ax.set_xticks([]); ax.set_yticks([])
    if after:
        ax.legend(loc="lower right", fontsize=8, framealpha=0.7)

    # --- DIFFERENCE panel ---
    ax = axes[2]
    # neutral background = the AFTER thumbnail dimmed
    ax.imshow(thumb_a, cmap="gray", interpolation="nearest", alpha=0.35)
    if unchanged:
        ux = [p.x_px / ds for p in unchanged]
        uy = [p.y_px / ds for p in unchanged]
        ax.scatter(ux, uy, s=4, color="0.65", marker=".",
                   label=f"unchanged ({len(unchanged)})")
    if removed:
        rx = [p.x_px / ds for p in removed]
        ry = [p.y_px / ds for p in removed]
        ax.scatter(rx, ry, s=22, facecolors="none", edgecolors="red",
                   linewidths=1.0, marker="o",
                   label=f"removed ({len(removed)})")
    if added:
        gx = [p.x_px / ds for p in added]
        gy = [p.y_px / ds for p in added]
        ax.scatter(gx, gy, s=28, color="lime", marker="x",
                   linewidths=1.2, label=f"added ({len(added)})")
    _draw_wafer_outline(ax, geom, ds)
    ax.set_title(f"{titles[2]}  —  +{len(added)} / -{len(removed)}")
    ax.set_xticks([]); ax.set_yticks([])
    ax.legend(loc="lower right", fontsize=8, framealpha=0.8)

    gdx, gdy = info.get("global_shift_um", (0.0, 0.0))
    fig.suptitle(
        f"{wafer_name}     particle-map comparison     "
        f"global shift = ({gdx:+.1f}, {gdy:+.1f}) µm     "
        f"clusters = {info.get('n_clusters', 0)}",
        fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=cfg["dpi"], bbox_inches="tight")
    plt.close(fig)
    logger.info("  3-panel figure → %s", out_path.name)


# ══════════════════════════════════════════════════════════════════════════════
#  CSV / JSON OUTPUT
# ══════════════════════════════════════════════════════════════════════════════
def save_particle_csv(path: Path, parts: List[Particle], status_label: str):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["id", "status", "x_um", "y_um", "x_px", "y_px",
                    "area_px", "circularity", "peak_residual",
                    "cluster_id", "match_dist_um", "tile"])
        for p in parts:
            w.writerow([p.id, status_label, f"{p.x_um:.2f}", f"{p.y_um:.2f}",
                        f"{p.x_px:.1f}", f"{p.y_px:.1f}", p.area_px,
                        f"{p.circularity:.3f}", f"{p.peak_residual:.2f}",
                        p.cluster_id, f"{p.match_dist_um:.2f}", p.tile_label])


def save_review_summary(path: Path, wafer_name: str,
                        before: List[Particle], after: List[Particle],
                        added: List[Particle], removed: List[Particle],
                        unchanged: List[Particle],
                        info: Dict[str, Any],
                        flagged: List[Dict[str, Any]],
                        cfg: Dict[str, Any]):
    gdx, gdy = info.get("global_shift_um", (0.0, 0.0))
    n_clusters = info.get("n_clusters", 0)
    max_local = info.get("max_local_shift_um", 0.0)

    lines: List[str] = []
    lines.append("=" * 78)
    lines.append(f"  REVIEW SUMMARY — {wafer_name}")
    lines.append("=" * 78)
    lines.append("")
    lines.append("Particle counts")
    lines.append("---------------")
    lines.append(f"  Before  : {len(before):>6d}")
    lines.append(f"  After   : {len(after):>6d}")
    lines.append(f"  Δ       : {len(after) - len(before):>+6d}")
    lines.append("")
    lines.append("Cluster-aware comparison")
    lines.append("------------------------")
    lines.append(f"  Unchanged (matched)   : {len(unchanged):>6d}")
    lines.append(f"  Added (new in After)  : {len(added):>6d}")
    lines.append(f"  Removed (gone in After): {len(removed):>5d}")
    lines.append(f"  Clusters considered   : {n_clusters:>6d}")
    lines.append("")
    lines.append("Residual misalignment")
    lines.append("---------------------")
    lines.append(f"  Global rigid shift  : dx = {gdx:+.2f} µm   dy = {gdy:+.2f} µm")
    lines.append(f"  Max local refinement: {max_local:.2f} µm  (per-cluster)")
    if max_local > 0.6 * float(cfg["local_shift_search_um"]):
        lines.append("  ⚠  local refinement saturating — consider widening "
                     "`local_shift_search_um`.")
    if math.hypot(gdx, gdy) > 0.6 * float(cfg["global_shift_search_um"]):
        lines.append("  ⚠  global shift saturating — consider widening "
                     "`global_shift_search_um`.")
    lines.append("")
    lines.append(f"Flagged tiles (n = {len(flagged)})")
    lines.append("-------------------------------")
    if not flagged:
        lines.append("  (none)")
    else:
        lines.append("  These tiles concentrate so many changes that they are")
        lines.append("  likely registration / illumination artefacts and warrant")
        lines.append("  manual review BEFORE trusting their counts:")
        lines.append("")
        lines.append("    Tile    +added  -removed  before  after   reason")
        for f in flagged[:25]:
            lines.append(f"    {f['tile']}  {f['added']:>6d}  {f['removed']:>8d}  "
                         f"{f['before']:>6d}  {f['after']:>5d}   {f['reason']}")
        if len(flagged) > 25:
            lines.append(f"    ... ({len(flagged) - 25} more)")
    lines.append("")
    lines.append("Caveats")
    lines.append("-------")
    lines.append("  • This pipeline NEVER thresholds the raw difference image.")
    lines.append("    Each scan is detected independently, then the two particle")
    lines.append("    MAPS are compared with a small rigid-shift tolerance.")
    lines.append("  • Particles that physically MOVED by less than")
    lines.append(f"    `match_radius_um` ({cfg['match_radius_um']:.1f} µm) are")
    lines.append("    classified as Unchanged, not as Added/Removed.")
    lines.append("  • Particles that moved by MORE than the cluster link distance")
    lines.append(f"    ({cfg['cluster_link_um']:.0f} µm) are reported as one")
    lines.append("    Added and one Removed event — they will not be merged.")
    lines.append("  • The rim of width "
                 f"{cfg['edge_exclusion_mm']:.1f} mm is excluded entirely.")
    lines.append("  • Tiles flagged above should be inspected by hand on the")
    lines.append("    raw imagery before being included in any production count.")
    lines.append("")
    lines.append("=" * 78)

    text = "\n".join(lines)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    # also echo to log
    for ln in lines:
        logger.info(ln)


# ══════════════════════════════════════════════════════════════════════════════
#  PER-WAFER PIPELINE
# ══════════════════════════════════════════════════════════════════════════════
def process_wafer_pair(pre_path: Path, post_path: Path, cfg: Dict[str, Any]):
    wafer_name = pre_path.stem
    out_dir = Path(cfg["results_dir"]) / wafer_name
    out_dir.mkdir(parents=True, exist_ok=True)

    logger.info("")
    logger.info("════════════════════════════════════════════════════════════════")
    logger.info("  WAFER: %s", wafer_name)
    logger.info("    pre : %s", pre_path)
    logger.info("    post: %s", post_path)
    logger.info("════════════════════════════════════════════════════════════════")

    lazy_b = LazyImage(str(pre_path),  cfg["use_memmap"], cfg["memmap_dir"])
    lazy_a = LazyImage(str(post_path), cfg["use_memmap"], cfg["memmap_dir"])
    lazy_b.open()
    lazy_a.open()

    if lazy_b.shape != lazy_a.shape:
        logger.warning("  before/after differ in shape (%s vs %s) — using BEFORE",
                       lazy_b.shape, lazy_a.shape)

    # Wafer geometry from BEFORE (the post-test image may have wipe disturbed)
    (cx, cy), R, fa = detect_wafer_on_thumbnail(lazy_b, cfg)
    geom = WaferGeometry(cfg, lazy_b.shape, (cx, cy), R, fa)

    tiles = build_tile_grid(geom, cfg)

    before = detect_all_particles(lazy_b, geom, tiles, cfg, "BEFORE")
    after  = detect_all_particles(lazy_a, geom, tiles, cfg, "AFTER")

    added, removed, unchanged, info = compare_particle_maps(before, after, cfg)
    flagged = flag_suspicious_tiles(tiles, added, removed, before, after, cfg)

    # Outputs
    fig_path = out_dir / f"{wafer_name}_three_panel.png"
    render_three_panel(lazy_b, lazy_a, geom, before, after,
                       added, removed, unchanged, info, cfg,
                       fig_path, wafer_name)

    save_review_summary(out_dir / f"{wafer_name}_review.txt",
                        wafer_name, before, after, added, removed, unchanged,
                        info, flagged, cfg)

    if cfg["save_csv"]:
        save_particle_csv(out_dir / f"{wafer_name}_added.csv",     added,     "added")
        save_particle_csv(out_dir / f"{wafer_name}_removed.csv",   removed,   "removed")
        save_particle_csv(out_dir / f"{wafer_name}_unchanged.csv", unchanged, "unchanged")
    if cfg["save_json"]:
        summary = {
            "wafer": wafer_name,
            "n_before":   len(before),
            "n_after":    len(after),
            "n_added":    len(added),
            "n_removed":  len(removed),
            "n_unchanged": len(unchanged),
            "global_shift_um": info.get("global_shift_um", (0.0, 0.0)),
            "n_clusters":      info.get("n_clusters", 0),
            "max_local_shift_um": info.get("max_local_shift_um", 0.0),
            "flagged_tiles": flagged,
            "config": {k: v for k, v in cfg.items()
                       if not isinstance(v, (Path,))},
        }
        with open(out_dir / f"{wafer_name}_summary.json", "w",
                  encoding="utf-8") as f:
            json.dump(summary, f, indent=2, default=str)

    lazy_b.close()
    lazy_a.close()
    logger.info("  done.")


# ══════════════════════════════════════════════════════════════════════════════
#  PAIR DISCOVERY
# ══════════════════════════════════════════════════════════════════════════════
def discover_pairs(cfg: Dict[str, Any]) -> List[Tuple[Path, Path]]:
    pre_dir  = Path(cfg["pre_folder"])
    post_dir = Path(cfg["post_folder"])
    if not pre_dir.exists() or not post_dir.exists():
        logger.error("Missing folders:\n  pre : %s\n  post: %s", pre_dir, post_dir)
        return []
    exts = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp"}
    pre  = {p.stem: p for p in pre_dir.iterdir()  if p.suffix.lower() in exts}
    post = {p.stem: p for p in post_dir.iterdir() if p.suffix.lower() in exts}
    common = sorted(set(pre.keys()) & set(post.keys()))
    if not common:
        logger.error("No matching wafer files in both folders. "
                     "Files must share the same base name in pre/ and post/.")
    pairs = [(pre[name], post[name]) for name in common]
    logger.info("Discovered %d wafer pair(s).", len(pairs))
    return pairs


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main():
    Path(cfg_results := CONFIG["results_dir"]).mkdir(parents=True, exist_ok=True)
    pairs = discover_pairs(CONFIG)
    if not pairs:
        sys.exit(1)
    t0 = time.time()
    for pre, post in pairs:
        try:
            process_wafer_pair(pre, post, CONFIG)
        except Exception as e:
            logger.exception("Failed on %s: %s", pre.name, e)
    logger.info("All wafers done in %.1f s.", time.time() - t0)


if __name__ == "__main__":
    main()
