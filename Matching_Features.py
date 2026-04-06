#!/usr/bin/env python3
"""

Dependencies
────────────
    pip install opencv-python-headless numpy scipy matplotlib pillow
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
from matplotlib.patches import Arc
from matplotlib.widgets import Button
from PIL import Image as PilImage
from scipy.spatial import cKDTree


# ══════════════════════════════════════════════════════════════════════════════
#  ╔══ CONFIG — edit these, then click Run ═══════════════════════════════════╗
# ══════════════════════════════════════════════════════════════════════════════
SCRIPT_DIR = Path(__file__).resolve().parent

CONFIG: Dict[str, Any] = {

    # ── Input / output folders ────────────────────────────────────────────────
    "pre_folder":   str(SCRIPT_DIR / "pre"),     # before-testing JPEGs
    "post_folder":  str(SCRIPT_DIR / "post"),    # after-testing JPEGs
    "results_dir":  str(SCRIPT_DIR / "results"),

    # ── Interactive review? ───────────────────────────────────────────────────
    "interactive_review": True,

    # ── Wafer physical geometry (SEMI M1-0302, 2" wafer) ──────────────────────
    "wafer_diameter_mm": 50.8,
    "flat_length_mm":    15.875,
    "flat_orientation":  "bottom",     # "bottom" | "top" | "left" | "right"

    # Pixel scale: set ONE of these (use the Keyence objective spec).
    # If auto-detected wafer edge is available, we'll derive it from there.
    "pixel_size_um":    None,          # e.g. 1.0 (µm/pixel)

    # ── Wafer centre override ─────────────────────────────────────────────────
    # None → auto-detect from a downsampled thumbnail (recommended).
    # [col, row] → force a specific centre.
    "wafer_center_px":  None,

    # ── Memory management ────────────────────────────────────────────────────
    # True  → one-time JPEG → .dat memmap conversion (minutes), then tiny RAM.
    # False → decode full JPEG into RAM (fast if you have 8+ GB free).
    "use_memmap":  True,
    "memmap_dir":  str(SCRIPT_DIR / "memmap_cache"),

    # ── Processing grid ──────────────────────────────────────────────────────
    "grid_cols":          22,
    "grid_rows":          22,
    "section_overlap_px": 48,          # border added to each side before crop
    # Sections must be MOSTLY on the active wafer area (not just touching it).
    # The old default of 0.30 let in fragmentary edge sections that produced
    # the bulk of false positives.  0.85+ keeps only essentially-full sections.
    "min_wafer_coverage": 0.85,

    # ── Per-section registration ─────────────────────────────────────────────
    # "ecc_with_orb_init" (best) | "ecc_only" | "orb_only" | "none"
    "registration_method": "ecc_with_orb_init",
    "ecc_motion_type":     "euclidean",   # "translation"|"euclidean"|"affine"
    "ecc_max_iter":        200,
    "ecc_epsilon":         1e-5,
    "ecc_gauss_size":      5,
    "ecc_min_score":       0.30,          # below this → low-confidence flag

    # ── Preprocessing ────────────────────────────────────────────────────────
    # NOTE: CLAHE has been DELIBERATELY removed.  On a stitched mosaic CLAHE
    # locally amplifies tile boundaries and illumination steps, which then
    # dominate the difference image.  Background variation is removed
    # algorithmically further down via top-hat + photometric matching.
    "gaussian_blur_ksize": 3,             # mild noise smoothing only

    # ── Background removal & photometric normalisation ───────────────────────
    # Top-hat structuring element diameter, in pixels.  Must be CLEARLY larger
    # than the largest particle you care about — anything smaller than this
    # kernel survives, anything larger is treated as background and removed.
    # Slow illumination gradients across a stitched section get killed
    # automatically as long as this kernel is smaller than the gradient scale.
    "bg_tophat_kernel_px": 41,
    # Photometric normalisation matches the AFTER section's mean/std to the
    # BEFORE section's mean/std (within the wafer mask) before differencing.
    # This kills tile-to-tile gain/offset drift that ECC cannot correct.
    "photometric_normalize": True,

    # ── Stitching-seam suppression ───────────────────────────────────────────
    # Long straight axis-aligned edges (= stitching seams) are detected via
    # morphological line opening and excluded from particle detection.  The
    # minimum line length must exceed the largest plausible particle.
    "seam_detect_enable":   True,
    "seam_min_length_px":   61,           # straight runs of >= this are seams
    "seam_dilate_px":       6,            # widen the seam mask by this much

    # ── Change-image particle detection ──────────────────────────────────────
    # We do NOT threshold the raw image.  We compute the SIGNED change image
    # (top-hat(after) − top-hat(before)) and threshold THAT at k·MAD above
    # the section's local noise floor.  k·MAD is robust to outliers and
    # auto-adapts to per-section noise.
    "k_sigma":              5.0,          # detection threshold in robust sigmas
    "min_change_floor":     6.0,          # absolute minimum residual (8-bit cts)
    "detect_bright":        True,
    "detect_dark":          True,
    "min_particle_area_px": 4,
    "max_particle_area_px": 5000,
    "min_circularity":      0.40,         # rejects scratches & registration crescents

    # The signal at a candidate must be MOSTLY explained by the change.
    # 0.5 means: at least half the after-image residual must be NEW signal
    # (i.e. the before image was mostly clean at that spot).  Increase for
    # stricter "new" classification, decrease for more permissive.
    "change_dominance":     0.50,

    # ── Global deduplication across tile borders ─────────────────────────────
    "dedup_radius_um":      3.0,

    # ── Edge exclusion (ignore the noisy wafer rim) ──────────────────────────
    # 2.5 mm rim is typically unusable on a 2-inch wafer due to handling
    # artifacts, edge bead, and unstable registration.
    "edge_exclusion_mm":   2.5,

    # Confidence display threshold (low values get a yellow ring on overlay)
    "confidence_low_thresh": 0.60,

    # ── Output ───────────────────────────────────────────────────────────────
    # "all" | "particles_only" | "none"
    "save_section_images": "particles_only",
    "overlay_max_side":    4000,
    "dpi":                 200,
}


# ══════════════════════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════════════════════
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s  %(levelname)-8s  %(message)s",
                    datefmt="%H:%M:%S")
logger = logging.getLogger("wafer_inspector")

PilImage.MAX_IMAGE_PIXELS = None     # allow very large JPEGs


# ══════════════════════════════════════════════════════════════════════════════
#  DATA CLASSES
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class Particle:
    """One detected particle, in full-image + wafer-µm coordinates."""
    id:          int       = -1
    x_px:        float     = 0.0     # full-image pixel column
    y_px:        float     = 0.0     # full-image pixel row
    x_um:        float     = 0.0     # wafer centre = (0, 0)
    y_um:        float     = 0.0     # Y-up (semiconductor convention)
    x_mm:        float     = 0.0
    y_mm:        float     = 0.0
    area_px:     int       = 0
    circularity: float     = 0.0
    peak_intensity: float  = 0.0
    kind:        str       = "bright"    # bright | dark
    status:      str       = "new"       # new | existing | removed | rejected
    confidence:  float     = 1.0         # 0–1, higher = more sure it's new
    ecc_score:   float     = 1.0         # per-section registration score
    section_label: str     = ""
    reviewed:    Optional[bool] = None

    @property
    def r_mm(self) -> float:
        return math.hypot(self.x_mm, self.y_mm)


@dataclass
class GridSection:
    row: int;  col: int;  label: str
    x1: int;  y1: int;  x2: int;  y2: int       # inner bounds
    ox1: int; oy1: int; ox2: int; oy2: int      # outer (with overlap)
    wafer_coverage: float


@dataclass
class SectionResult:
    section:      GridSection
    ecc_score:    float = 1.0
    particles_before: List[Particle] = field(default_factory=list)
    particles_after:  List[Particle] = field(default_factory=list)
    new:          List[Particle] = field(default_factory=list)
    existing:     List[Particle] = field(default_factory=list)
    removed:      List[Particle] = field(default_factory=list)
    error:        str = ""


# ══════════════════════════════════════════════════════════════════════════════
#  LAZY IMAGE  (memmap or RAM, uniform interface)
# ══════════════════════════════════════════════════════════════════════════════
class LazyImage:
    """Provides .crop(x1,y1,x2,y2) on a possibly-huge grayscale image without
    requiring the whole thing in RAM."""

    def __init__(self, jpeg_path: str, use_memmap: bool, memmap_dir: str):
        self.path     = Path(jpeg_path)
        self.use_mm   = use_memmap
        self.mm_dir   = Path(memmap_dir)
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
        logger.info("  %s  shape=%s  backend=%s  (%.0f MB)",
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
            out[cy1 - y1 : cy2 - y1, cx1 - x1 : cx2 - x1] = region
        return out

    def thumbnail(self, max_side: int = 2000) -> np.ndarray:
        """Return a downscaled copy (for wafer-geometry detection)."""
        H, W = self._shape
        ds = max(1.0, max(H, W) / max_side)
        sh, sw = int(H / ds), int(W / ds)
        # Build thumbnail by reading in strips to stay memory-light
        strip_rows = max(1, sh)
        # Simple path: read whole small thumbnail via Pillow
        try:
            img = PilImage.open(str(self.path))
            img.draft("L", (sw, sh))
            img = img.convert("L").resize((sw, sh), PilImage.LANCZOS)
            thumb = np.asarray(img, dtype=np.uint8)
            img.close()
            return thumb
        except Exception:
            # Fallback: read from the memmap/array in chunks
            step = max(1, int(ds))
            return self._arr[::step, ::step].copy()

    def close(self) -> None:
        if self._arr is not None and self.use_mm:
            self._arr = None
        else:
            self._arr = None
        gc.collect()

    # ---- internals ----------------------------------------------------------

    def _get_or_make_memmap(self) -> np.memmap:
        self.mm_dir.mkdir(parents=True, exist_ok=True)
        # Key memmap files by the full resolved source path so that two wafers
        # sharing the same filename in different folders don't collide.
        stem_key = f"{self.path.stem}_{abs(hash(str(self.path.resolve()))) & 0xFFFFFFFF:08x}"
        dat   = self.mm_dir / f"{stem_key}.dat"
        shape = self.mm_dir / f"{stem_key}.shape.npy"

        if dat.exists() and shape.exists():
            sh = tuple(np.load(shape).tolist())
            logger.info("  reusing memmap cache: %s  shape=%s", dat.name, sh)
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
    """Encodes the wafer disc + primary flat and handles px ↔ µm conversion."""

    def __init__(self, cfg: Dict[str, Any], img_shape: Tuple[int, int],
                 center_px: Tuple[float, float], radius_px: float,
                 flat_angle_rad: float):
        self.cfg = cfg
        self.H, self.W = img_shape
        self.cx, self.cy = center_px
        self.radius_px = radius_px
        self.flat_angle = flat_angle_rad   # angle from center → flat midpoint

        self.diameter_mm = cfg["wafer_diameter_mm"]
        self.flat_length_mm = cfg["flat_length_mm"]

        # Pixel scale from auto-detected radius
        self.px_per_mm = (2.0 * radius_px) / self.diameter_mm
        self.um_per_px = 1000.0 / self.px_per_mm
        self.edge_excl_px = int(cfg["edge_exclusion_mm"] * self.px_per_mm)

        # Distance (pixels) from centre to flat chord
        r_mm = self.diameter_mm / 2.0
        half_flat = self.flat_length_mm / 2.0
        # chord distance from centre (mm)
        self.flat_dist_mm = math.sqrt(max(0.0, r_mm**2 - half_flat**2))
        self.flat_dist_px = self.flat_dist_mm * self.px_per_mm

        logger.info("  wafer: centre=(%.0f, %.0f) R=%.0f px  %.2f px/mm  "
                    "flat_angle=%.1f°",
                    self.cx, self.cy, self.radius_px, self.px_per_mm,
                    math.degrees(flat_angle_rad))

    # ---- masking ------------------------------------------------------------

    def build_active_mask(self, downsample: int = 1) -> np.ndarray:
        """Binary mask of the active wafer area (circle ∩ above-flat) at a
        given downsample factor, with edge exclusion applied."""
        H = self.H // downsample
        W = self.W // downsample
        cx = self.cx / downsample
        cy = self.cy / downsample
        R = (self.radius_px - self.edge_excl_px) / downsample

        yy, xx = np.ogrid[:H, :W]
        circle = (xx - cx) ** 2 + (yy - cy) ** 2 <= R ** 2

        # flat: keep points on the FAR side of the flat chord from the flat itself
        fa = self.flat_angle
        nx, ny = math.cos(fa), math.sin(fa)      # unit vector centre→flat
        # signed distance from centre along (nx, ny)
        sd = (xx - cx) * nx + (yy - cy) * ny
        # active if sd < flat_dist (because flat is at sd = flat_dist)
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

    # ---- coordinate conversions --------------------------------------------

    def px_to_um(self, col: float, row: float) -> Tuple[float, float]:
        """Full-image (col, row) → wafer µm (X right, Y up from centre)."""
        x_um = (col - self.cx) * self.um_per_px
        y_um = -(row - self.cy) * self.um_per_px       # flip Y
        return x_um, y_um

    def px_to_mm(self, col: float, row: float) -> Tuple[float, float]:
        x, y = self.px_to_um(col, row)
        return x / 1000.0, y / 1000.0


# ══════════════════════════════════════════════════════════════════════════════
#  AUTO-DETECT wafer geometry on a thumbnail
# ══════════════════════════════════════════════════════════════════════════════
def detect_wafer_on_thumbnail(lazy: LazyImage, cfg: Dict[str, Any]
                              ) -> Tuple[Tuple[float, float], float, float]:
    """Return (centre_px_full, radius_px_full, flat_angle_rad) in FULL-image
    coordinates by detecting the wafer in a downsampled thumbnail."""
    full_H, full_W = lazy.shape
    thumb = lazy.thumbnail(max_side=2000)
    th, tw = thumb.shape
    scale_x = full_W / tw
    scale_y = full_H / th

    # ──────────────────────────────────────────────────────────────────────────
    # The wafer is sitting on a WHITE WIPE inside a black-cornered scan area:
    #   • Outside the scan rectangle  → BLACK (Keyence didn't image there)
    #   • White wipe around the wafer → BRIGHT
    #   • The wafer disc itself       → DARKER (silicon is gray)
    # So we cannot just take "the largest bright blob" (that would be the wipe).
    # Instead we find the largest DARK connected component that does NOT touch
    # the image border (the black corners do touch the border; the wafer is an
    # interior dark hole inside the bright wipe).
    # ──────────────────────────────────────────────────────────────────────────
    blur = cv2.GaussianBlur(thumb, (9, 9), 0)
    otsu_val, _ = cv2.threshold(blur, 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # bright = wipe; dark = wafer + black scan corners
    _, bright = cv2.threshold(blur, otsu_val, 255, cv2.THRESH_BINARY)

    # cleanup
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    bright = cv2.morphologyEx(bright, cv2.MORPH_CLOSE, k, iterations=2)

    # connected components of the DARK regions
    dark = 255 - bright
    n_lab, labels, stats, _ = cv2.connectedComponentsWithStats(dark, connectivity=8)
    H_t, W_t = thumb.shape

    # candidate wafer = largest dark CC that does NOT touch the image border
    wafer_label = -1
    wafer_area  = 0
    for i in range(1, n_lab):
        x, y, w, h, area = stats[i]
        if x == 0 or y == 0 or x + w >= W_t or y + h >= H_t:
            continue                          # touches border → black corner
        if area > wafer_area:
            wafer_area  = int(area)
            wafer_label = i

    if wafer_label < 0:
        # Fallback: maybe the wipe doesn't fully surround the wafer.
        # Try the classic "largest bright blob" approach but warn the user.
        logger.warning("White wipe not fully detected around wafer — "
                        "falling back to brightest-blob method.")
        contours, _ = cv2.findContours(bright, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise RuntimeError("Wafer not found in thumbnail — check input image")
        wafer_cnt = max(contours, key=cv2.contourArea)
    else:
        wafer_mask_t = (labels == wafer_label).astype(np.uint8) * 255
        # smooth the mask edges (the wafer rim is somewhat ragged in the thumbnail)
        wafer_mask_t = cv2.morphologyEx(
            wafer_mask_t, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
            iterations=2)
        contours, _ = cv2.findContours(wafer_mask_t, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise RuntimeError("Wafer mask empty after cleanup")
        wafer_cnt = max(contours, key=cv2.contourArea)

    (cx_t, cy_t), r_t = cv2.minEnclosingCircle(wafer_cnt)

    # --- primary flat: contour points indented from the circle ---
    pts = wafer_cnt.reshape(-1, 2).astype(np.float64)
    dists = np.hypot(pts[:, 0] - cx_t, pts[:, 1] - cy_t)
    indent = r_t - dists
    flat_mask = indent > r_t * 0.015
    if flat_mask.sum() > 10:
        fpts = pts[flat_mask]
        fmid = fpts.mean(axis=0)
        flat_angle = float(math.atan2(fmid[1] - cy_t, fmid[0] - cx_t))
    else:
        # fall back to config orientation
        orient = cfg.get("flat_orientation", "bottom")
        flat_angle = {"bottom": math.pi / 2, "top": -math.pi / 2,
                      "left": math.pi, "right": 0.0}.get(orient, math.pi / 2)
        logger.warning("Primary flat not detected — using config orientation")

    # scale back to full resolution
    cx_full = cx_t * scale_x
    cy_full = cy_t * scale_y
    # radius: average the two scales (should be ~equal)
    r_full = r_t * (scale_x + scale_y) / 2.0

    # honour user override
    override = cfg.get("wafer_center_px")
    if override is not None:
        cx_full, cy_full = float(override[0]), float(override[1])

    return (cx_full, cy_full), r_full, flat_angle


# ══════════════════════════════════════════════════════════════════════════════
#  GRID BUILDER
# ══════════════════════════════════════════════════════════════════════════════
def build_grid(geom: WaferGeometry, cfg: Dict[str, Any]) -> List[GridSection]:
    H, W = geom.H, geom.W
    nc, nr = cfg["grid_cols"], cfg["grid_rows"]
    ov = cfg["section_overlap_px"]

    cell_w = W / nc
    cell_h = H / nr

    # Build a coarse active mask for fast coverage queries
    ds = 8
    mask_small = geom.build_active_mask(downsample=ds)
    mask_total = mask_small.sum() / 255.0

    sections = []
    for r in range(nr):
        for c in range(nc):
            x1 = int(round(c * cell_w));  x2 = int(round((c + 1) * cell_w))
            y1 = int(round(r * cell_h));  y2 = int(round((r + 1) * cell_h))
            ox1 = max(0, x1 - ov); oy1 = max(0, y1 - ov)
            ox2 = min(W, x2 + ov); oy2 = min(H, y2 + ov)

            # coverage: how much of the inner cell is on the active wafer
            x1s, y1s = x1 // ds, y1 // ds
            x2s, y2s = max(x1s + 1, x2 // ds), max(y1s + 1, y2 // ds)
            sub = mask_small[y1s:y2s, x1s:x2s]
            coverage = (sub > 0).mean() if sub.size > 0 else 0.0

            if coverage < cfg["min_wafer_coverage"]:
                continue

            sections.append(GridSection(
                row=r, col=c, label=f"R{r:02d}C{c:02d}",
                x1=x1, y1=y1, x2=x2, y2=y2,
                ox1=ox1, oy1=oy1, ox2=ox2, oy2=oy2,
                wafer_coverage=float(coverage)))
    logger.info("Grid: %d active sections / %d total",
                len(sections), nr * nc)
    return sections


# ══════════════════════════════════════════════════════════════════════════════
#  PREPROCESS + REGISTRATION
# ══════════════════════════════════════════════════════════════════════════════
def preprocess(gray: np.ndarray, cfg: Dict[str, Any]) -> np.ndarray:
    """Mild noise smoothing only.

    DELIBERATELY no CLAHE / no histogram equalisation: on a stitched mosaic
    those operations locally amplify tile-boundary intensity steps and
    illumination gradients, which then dominate the change image and produce
    massive false-positive counts.  All background flattening is done later
    via top-hat morphology, which by construction removes anything larger
    than the structuring element.
    """
    k = cfg["gaussian_blur_ksize"] | 1
    if k >= 3:
        return cv2.GaussianBlur(gray, (k, k), 0)
    return gray.copy()


# ── Photometric & seam helpers ────────────────────────────────────────────────

def photometric_normalize(after: np.ndarray, before: np.ndarray,
                          mask: np.ndarray) -> np.ndarray:
    """Linearly remap the AFTER section's intensities so its mean and std
    match the BEFORE section, computed within the wafer mask only.

    This compensates for tile-to-tile gain/offset drift between the two
    Keyence sessions which ECC registration cannot correct.  Done per-section
    so global brightness drift across the mosaic doesn't matter — only the
    local match counts.
    """
    valid = mask > 0
    if valid.sum() < 100:
        return after.copy()
    a = after[valid].astype(np.float32)
    b = before[valid].astype(np.float32)
    ma, sa = float(a.mean()), max(1.0, float(a.std()))
    mb, sb = float(b.mean()), max(1.0, float(b.std()))
    out = (after.astype(np.float32) - ma) * (sb / sa) + mb
    return np.clip(out, 0, 255).astype(np.uint8)


def detect_seams(gray: np.ndarray, mask: np.ndarray,
                 cfg: Dict[str, Any]) -> np.ndarray:
    """Return a binary mask (uint8, 255 = seam) of stitching-seam pixels.

    Stitching seams are LONG STRAIGHT axis-aligned step edges.  We isolate
    them with morphological line opening on a Canny edge map: any edge that
    survives opening with a 1-D structuring element of length >= seam_min
    is by definition a long straight line — particles cannot satisfy this.
    The mask is then dilated so the seam neighbourhood is also excluded.
    """
    if not cfg.get("seam_detect_enable", True):
        return np.zeros_like(gray, dtype=np.uint8)

    # Canny on the smoothed image
    edges = cv2.Canny(gray, 30, 90, L2gradient=True)
    edges[mask == 0] = 0

    L = max(15, int(cfg.get("seam_min_length_px", 61)))
    h_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (L, 1))
    v_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, L))
    h_lines = cv2.morphologyEx(edges, cv2.MORPH_OPEN, h_kernel)
    v_lines = cv2.morphologyEx(edges, cv2.MORPH_OPEN, v_kernel)
    seam = cv2.bitwise_or(h_lines, v_lines)

    d = max(1, int(cfg.get("seam_dilate_px", 6)))
    seam = cv2.dilate(seam, cv2.getStructuringElement(cv2.MORPH_RECT, (d, d)))
    return seam


def orb_coarse_align(ref: np.ndarray, mov: np.ndarray) -> np.ndarray:
    orb = cv2.ORB_create(nfeatures=1500)
    kp1, d1 = orb.detectAndCompute(ref, None)
    kp2, d2 = orb.detectAndCompute(mov, None)
    I = np.eye(2, 3, dtype=np.float32)
    if d1 is None or d2 is None or len(kp1) < 8 or len(kp2) < 8:
        return I
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    try:
        raw = bf.knnMatch(d1, d2, k=2)
    except cv2.error:
        return I
    good = [m for pair in raw if len(pair) == 2
            for m, n in [pair] if m.distance < 0.75 * n.distance]
    if len(good) < 8:
        return I
    p1 = np.float32([kp1[m.queryIdx].pt for m in good])
    p2 = np.float32([kp2[m.trainIdx].pt for m in good])
    warp, _ = cv2.estimateAffinePartial2D(p2, p1, method=cv2.RANSAC,
                                          ransacReprojThreshold=3.0)
    return warp.astype(np.float32) if warp is not None else I


_ECC_MOTION = {
    "translation": cv2.MOTION_TRANSLATION,
    "euclidean":   cv2.MOTION_EUCLIDEAN,
    "affine":      cv2.MOTION_AFFINE,
}

def ecc_refine(ref: np.ndarray, mov: np.ndarray, warp_init: np.ndarray,
               cfg: Dict[str, Any]) -> Tuple[np.ndarray, float]:
    motion = _ECC_MOTION[cfg["ecc_motion_type"]]
    if motion == cv2.MOTION_HOMOGRAPHY:
        warp = np.eye(3, dtype=np.float32)
    else:
        warp = warp_init.copy() if warp_init is not None \
               else np.eye(2, 3, dtype=np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                cfg["ecc_max_iter"], cfg["ecc_epsilon"])
    try:
        score, warp = cv2.findTransformECC(
            ref, mov, warp, motion, criteria, None, cfg["ecc_gauss_size"])
        return warp, float(score)
    except cv2.error:
        return warp_init if warp_init is not None else \
               np.eye(2, 3, dtype=np.float32), 0.0


def register_section(before: np.ndarray, after: np.ndarray,
                     cfg: Dict[str, Any]) -> Tuple[np.ndarray, float]:
    """Return (aligned_after, ecc_score).  Aligns after → before."""
    method = cfg["registration_method"]
    h, w = before.shape
    if method == "none":
        return after, 1.0

    if method == "orb_only":
        warp = orb_coarse_align(before, after)
        aligned = cv2.warpAffine(after, warp, (w, h),
                                 flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                 borderMode=cv2.BORDER_REFLECT_101)
        return aligned, 1.0

    if method == "ecc_only":
        warp_init = np.eye(2, 3, dtype=np.float32)
    else:  # ecc_with_orb_init
        warp_init = orb_coarse_align(before, after)
        after = cv2.warpAffine(after, warp_init, (w, h),
                               flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                               borderMode=cv2.BORDER_REFLECT_101)
        warp_init = np.eye(2, 3, dtype=np.float32)

    warp, score = ecc_refine(before, after, warp_init, cfg)
    aligned = cv2.warpAffine(after, warp, (w, h),
                             flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                             borderMode=cv2.BORDER_REFLECT_101)
    return aligned, score


# ══════════════════════════════════════════════════════════════════════════════
#  CHANGE-IMAGE PARTICLE DETECTION
# ══════════════════════════════════════════════════════════════════════════════
#
# Algorithm (per section, after registration & photometric normalisation):
#
#   1. Compute the top-hat residual of BOTH images using a kernel clearly
#      larger than the largest particle.  This removes the local background
#      (slow illumination gradients) by definition.
#         th_b = white_tophat(before_pp,  bg_kernel)
#         th_a = white_tophat(after_pp,   bg_kernel)
#
#   2. Compute the SIGNED change image:
#         change = th_a − th_b
#      Positive values  → bright signal that GAINED intensity (new particles)
#      Negative values  → bright signal that LOST intensity (removed particles)
#      Near-zero values → unchanged (existing particles or empty background)
#
#   3. Estimate the per-section noise floor robustly:
#         sigma = 1.4826 · MAD(change[mask])
#      and threshold at  k · sigma  (default k = 5).  This adapts to each
#      section's actual noise level — no fixed Otsu/manual threshold needed.
#
#   4. Detect connected components above the threshold.  Filter by area
#      and circularity (rejects scratches and stitching crescents).
#
#   5. For each "after" detection, classify by inspecting BOTH residuals at
#      that location:
#         • If the BEFORE residual is also strong → EXISTING particle
#         • If the BEFORE residual is weak/absent → NEW particle
#      Same logic in reverse for removed (detect on −change, look up before).
#
# All thresholding is done on the CHANGE image, never on raw pixels — so
# tile-to-tile lighting differences are removed BEFORE the detector ever
# sees them.
# ══════════════════════════════════════════════════════════════════════════════

def _local_peak(arr: np.ndarray, x: int, y: int, r: int = 2) -> float:
    """Peak value in a small neighbourhood (handles edges)."""
    h, w = arr.shape
    y0, y1 = max(0, y - r), min(h, y + r + 1)
    x0, x1 = max(0, x - r), min(w, x + r + 1)
    if y1 <= y0 or x1 <= x0:
        return 0.0
    return float(arr[y0:y1, x0:x1].max())


def _robust_sigma(values: np.ndarray) -> float:
    """1.4826 · MAD — robust standard-deviation estimator."""
    if values.size == 0:
        return 1.0
    med = float(np.median(values))
    mad = float(np.median(np.abs(values - med)))
    return max(1.0, 1.4826 * mad)


def _components_with_area_circ(bw: np.ndarray, cfg: Dict[str, Any]
                                ) -> List[Tuple[float, float, int, float]]:
    """Return list of (cx, cy, area, circularity) for blobs passing filters."""
    ok = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN,  ok)
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, ok)
    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    out = []
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
        out.append((cx, cy, area, circ))
    return out


def detect_changes_in_section(before_pp: np.ndarray, after_pp: np.ndarray,
                               mask: np.ndarray, cfg: Dict[str, Any]
                               ) -> Tuple[List[Particle], List[Particle],
                                          List[Particle]]:
    """Detect new / existing / removed particles in one section pair.

    Both images must already be aligned and photometrically normalised.
    `mask` must be 0 outside the wafer AND outside any seams (bitwise-AND'd
    by the caller).

    Returns (new, existing, removed) — particle coordinates are in section-
    local pixels (the caller offsets them to full-image space).
    """
    new: List[Particle] = []
    existing: List[Particle] = []
    removed: List[Particle] = []

    valid = mask > 0
    if valid.sum() < 200:
        return new, existing, removed

    bg_k = cfg["bg_tophat_kernel_px"] | 1
    se   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (bg_k, bg_k))

    pid = 0
    for enabled, op, kind in [
        (cfg["detect_bright"], cv2.MORPH_TOPHAT,   "bright"),
        (cfg["detect_dark"],   cv2.MORPH_BLACKHAT, "dark"),
    ]:
        if not enabled:
            continue

        # 1. background-flat residuals (slow illumination removed)
        th_b = cv2.morphologyEx(before_pp, op, se).astype(np.float32)
        th_a = cv2.morphologyEx(after_pp,  op, se).astype(np.float32)
        th_b[~valid] = 0.0
        th_a[~valid] = 0.0

        # 2. signed change image
        change = th_a - th_b

        # 3. robust per-section threshold
        v = change[valid]
        sigma = _robust_sigma(v)
        k_sig = float(cfg["k_sigma"])
        floor = float(cfg["min_change_floor"])
        thr = max(k_sig * sigma, floor)

        # ── NEW: positive change ─────────────────────────────────────────────
        bw_new = (change > thr).astype(np.uint8) * 255
        bw_new[~valid] = 0
        for cx, cy, area, circ in _components_with_area_circ(bw_new, cfg):
            ix, iy = int(round(cx)), int(round(cy))
            peak_change = _local_peak(change, ix, iy, r=2)
            peak_after  = _local_peak(th_a,   ix, iy, r=2)
            peak_before = _local_peak(th_b,   ix, iy, r=2)
            # The signal in AFTER must be mostly explained by the change,
            # i.e. BEFORE was relatively clean at this location.
            denom = max(1.0, peak_after)
            dominance = peak_change / denom
            if dominance < cfg["change_dominance"]:
                # signal exists in both images → not new, it's an existing
                # particle that just moved a hair due to residual misalignment
                existing.append(Particle(
                    id=pid, x_px=cx, y_px=cy, area_px=area,
                    circularity=circ, peak_intensity=peak_after,
                    kind=kind, status="existing", confidence=0.0))
                pid += 1
                continue
            new.append(Particle(
                id=pid, x_px=cx, y_px=cy, area_px=area, circularity=circ,
                peak_intensity=peak_after, kind=kind,
                status="new",
                confidence=float(np.clip(dominance, 0.0, 1.0))))
            pid += 1

        # ── REMOVED: negative change ─────────────────────────────────────────
        bw_rem = (change < -thr).astype(np.uint8) * 255
        bw_rem[~valid] = 0
        for cx, cy, area, circ in _components_with_area_circ(bw_rem, cfg):
            ix, iy = int(round(cx)), int(round(cy))
            peak_change = -_local_peak(-change, ix, iy, r=2)   # most-negative
            peak_before = _local_peak(th_b, ix, iy, r=2)
            denom = max(1.0, peak_before)
            dominance = (-peak_change) / denom
            if dominance < cfg["change_dominance"]:
                continue   # not really removed, just registration noise
            removed.append(Particle(
                id=pid, x_px=cx, y_px=cy, area_px=area, circularity=circ,
                peak_intensity=peak_before, kind=kind,
                status="removed",
                confidence=float(np.clip(dominance, 0.0, 1.0))))
            pid += 1

        # ── EXISTING: particles present in BOTH (low-change blobs above
        #              the strong-residual threshold in BOTH images) ─────────
        strong_thr = thr * 1.5    # stricter — we want clear particles
        bw_both = ((th_a > strong_thr) & (th_b > strong_thr)
                    & (np.abs(change) < thr)).astype(np.uint8) * 255
        bw_both[~valid] = 0
        for cx, cy, area, circ in _components_with_area_circ(bw_both, cfg):
            ix, iy = int(round(cx)), int(round(cy))
            existing.append(Particle(
                id=pid, x_px=cx, y_px=cy, area_px=area, circularity=circ,
                peak_intensity=_local_peak(th_a, ix, iy, r=2),
                kind=kind, status="existing", confidence=0.0))
            pid += 1

    return new, existing, removed


# ══════════════════════════════════════════════════════════════════════════════
#  SECTION PROCESSING
# ══════════════════════════════════════════════════════════════════════════════
def process_section(sec: GridSection,
                    lazy_before: LazyImage, lazy_after: LazyImage,
                    geom: WaferGeometry, cfg: Dict[str, Any],
                    out_dir: Path) -> SectionResult:
    result = SectionResult(section=sec)
    try:
        # --- crop (outer / overlap) ---
        before_out = lazy_before.crop(sec.ox1, sec.oy1, sec.ox2, sec.oy2)
        after_out  = lazy_after .crop(sec.ox1, sec.oy1, sec.ox2, sec.oy2)

        if before_out.size == 0 or after_out.size == 0:
            return result
        if before_out.max() == 0 and after_out.max() == 0:
            return result          # fully in the JPEG black corner

        # --- register after → before ---
        aligned_after, score = register_section(before_out, after_out, cfg)
        result.ecc_score = score

        # --- mild preprocessing (no CLAHE — see preprocess() docstring) ---
        before_pp = preprocess(before_out,   cfg)
        after_pp  = preprocess(aligned_after, cfg)

        # --- build section-local wafer mask (geometry only) ---
        H, W = before_pp.shape
        yy, xx = np.ogrid[:H, :W]
        cols_full = xx + sec.ox1
        rows_full = yy + sec.oy1
        dx = cols_full - geom.cx
        dy = rows_full - geom.cy
        R = geom.radius_px - geom.edge_excl_px
        circle = dx * dx + dy * dy <= R * R
        nx, ny = math.cos(geom.flat_angle), math.sin(geom.flat_angle)
        sd = dx * nx + dy * ny
        active = circle & (sd < (geom.flat_dist_px - geom.edge_excl_px))
        section_mask = (active.astype(np.uint8) * 255)

        if section_mask.max() == 0:
            return result

        # --- photometric normalisation: match AFTER's brightness to BEFORE,
        #     computed within the wafer mask only (so the white wipe outside
        #     the wafer doesn't skew the statistics) ---
        if cfg.get("photometric_normalize", True):
            after_pp = photometric_normalize(after_pp, before_pp, section_mask)

        # --- detect & exclude stitching seams ---
        seam_mask = detect_seams(before_pp, section_mask, cfg)
        if seam_mask.any():
            # OR with after-image seams too (registration may shift them)
            seam_mask = cv2.bitwise_or(
                seam_mask, detect_seams(after_pp, section_mask, cfg))
            section_mask = cv2.bitwise_and(
                section_mask, cv2.bitwise_not(seam_mask))

        if section_mask.max() == 0:
            return result

        # --- single-pass change-image detection (new / existing / removed) ---
        new_parts, exist_parts, rem_parts = detect_changes_in_section(
            before_pp, after_pp, section_mask, cfg)
        new       = new_parts
        existing  = exist_parts
        removed   = rem_parts

        # --- keep only particles whose CENTROID is in the inner zone ---
        inner_x0 = sec.x1 - sec.ox1
        inner_y0 = sec.y1 - sec.oy1
        inner_x1 = sec.x2 - sec.ox1
        inner_y1 = sec.y2 - sec.oy1

        def in_inner(p: Particle) -> bool:
            return (inner_x0 <= p.x_px < inner_x1
                    and inner_y0 <= p.y_px < inner_y1)

        def finalise(lst: List[Particle], source: str) -> List[Particle]:
            kept = []
            for p in lst:
                if not in_inner(p):
                    continue
                # convert to full-image pixels
                p.x_px = sec.ox1 + p.x_px
                p.y_px = sec.oy1 + p.y_px
                # check wafer active area
                if not geom.is_point_active(p.x_px, p.y_px):
                    continue
                # wafer µm / mm
                p.x_um, p.y_um = geom.px_to_um(p.x_px, p.y_px)
                p.x_mm = p.x_um / 1000.0
                p.y_mm = p.y_um / 1000.0
                p.ecc_score = score
                p.section_label = sec.label
                kept.append(p)
            return kept

        result.existing = finalise(existing, "existing")
        result.new      = finalise(new,      "new")
        result.removed  = finalise(removed,  "removed")

        # --- optional per-section annotated image ---
        save_mode = cfg["save_section_images"]
        have_particles = bool(result.new or result.removed)
        if save_mode == "all" or (save_mode == "particles_only" and have_particles):
            save_section_panel(sec, before_out, aligned_after,
                               before_pp, after_pp,
                               result, out_dir)

        return result

    except Exception as e:
        logger.exception("Error in section %s: %s", sec.label, e)
        result.error = str(e)
        return result


# ══════════════════════════════════════════════════════════════════════════════
#  GLOBAL DEDUPLICATION  (particles from adjacent tiles that slipped through)
# ══════════════════════════════════════════════════════════════════════════════
def dedup_particles(parts: List[Particle], radius_px: float) -> List[Particle]:
    if len(parts) < 2:
        return parts
    coords = np.array([(p.x_px, p.y_px) for p in parts])
    tree = cKDTree(coords)
    keep_flag = np.ones(len(parts), dtype=bool)
    for i in range(len(parts)):
        if not keep_flag[i]:
            continue
        idxs = tree.query_ball_point(coords[i], radius_px)
        for j in idxs:
            if j == i or not keep_flag[j]:
                continue
            # keep the larger / more circular one
            a, b = parts[i], parts[j]
            if (b.area_px, b.circularity) > (a.area_px, a.circularity):
                keep_flag[i] = False
                break
            else:
                keep_flag[j] = False
    return [p for p, k in zip(parts, keep_flag) if k]


# ══════════════════════════════════════════════════════════════════════════════
#  REPORTS + VISUALISATION
# ══════════════════════════════════════════════════════════════════════════════
def save_csv(parts: List[Particle], path: Path) -> None:
    cols = ["id", "status", "confidence", "kind", "x_mm", "y_mm", "r_mm",
            "x_um", "y_um", "x_px", "y_px", "area_px", "circularity",
            "peak_intensity", "ecc_score", "section_label", "reviewed"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for p in parts:
            row = {c: getattr(p, c) for c in cols}
            for k in ("confidence", "x_mm", "y_mm", "r_mm", "x_um", "y_um",
                      "x_px", "y_px", "circularity", "ecc_score"):
                row[k] = f"{float(row[k]):.3f}"
            w.writerow(row)
    logger.info("CSV → %s", path)


def save_json_summary(existing: List[Particle], new: List[Particle],
                      removed: List[Particle], geom: WaferGeometry,
                      path: Path) -> None:
    summary = {
        "wafer_diameter_mm":  geom.diameter_mm,
        "px_per_mm":          round(geom.px_per_mm, 3),
        "um_per_px":          round(geom.um_per_px, 3),
        "counts": {
            "existing": len(existing),
            "new":      len(new),
            "removed":  len(removed),
            "new_high_confidence":
                sum(1 for p in new if p.confidence >= 0.7),
            "new_uncertain":
                sum(1 for p in new if p.confidence < 0.7),
        },
        "new_particles": [
            {"id": p.id, "x_mm": round(p.x_mm, 3), "y_mm": round(p.y_mm, 3),
             "r_mm": round(p.r_mm, 3), "confidence": round(p.confidence, 2),
             "area_px": p.area_px, "circularity": round(p.circularity, 2),
             "kind": p.kind, "section": p.section_label,
             "ecc_score": round(p.ecc_score, 2),
             "reviewed": p.reviewed}
            for p in new
        ],
        "removed_particles": [
            {"id": p.id, "x_mm": round(p.x_mm, 3), "y_mm": round(p.y_mm, 3),
             "r_mm": round(p.r_mm, 3), "area_px": p.area_px,
             "kind": p.kind, "section": p.section_label}
            for p in removed
        ],
    }
    path.write_text(json.dumps(summary, indent=2))
    logger.info("JSON → %s", path)


def draw_wafer_outline(ax, geom: WaferGeometry):
    r = geom.diameter_mm / 2.0
    half_flat = geom.flat_length_mm / 2.0
    flat_angle_deg = math.degrees(math.asin(half_flat / r))

    arc = Arc((0, 0), 2 * r, 2 * r,
              theta1=-90 + flat_angle_deg,
              theta2=270 - flat_angle_deg,
              linewidth=1.5, edgecolor="black")
    ax.add_patch(arc)
    y_flat = -math.sqrt(r * r - half_flat * half_flat)
    ax.plot([-half_flat, half_flat], [y_flat, y_flat],
            color="black", linewidth=1.5)


def generate_wafer_map(existing: List[Particle], new: List[Particle],
                       removed: List[Particle], geom: WaferGeometry,
                       out_dir: Path, cfg: Dict[str, Any]) -> None:
    fig, ax = plt.subplots(figsize=(9, 9), dpi=cfg["dpi"])
    ax.set_aspect("equal")
    r = geom.diameter_mm / 2.0
    ax.set_xlim(-r - 3, r + 3)
    ax.set_ylim(-r - 3, r + 3)
    draw_wafer_outline(ax, geom)

    for tick in np.arange(-25, 26, 5):
        ax.axhline(tick, color="#eeeeee", lw=0.3, zorder=0)
        ax.axvline(tick, color="#eeeeee", lw=0.3, zorder=0)
    ax.axhline(0, color="#cccccc", lw=0.5, zorder=0)
    ax.axvline(0, color="#cccccc", lw=0.5, zorder=0)

    if existing:
        xy = np.array([(p.x_mm, p.y_mm) for p in existing])
        ax.scatter(xy[:, 0], xy[:, 1], s=10, c="royalblue", alpha=0.55,
                   label=f"Existing ({len(existing)})", zorder=2)

    if removed:
        xy = np.array([(p.x_mm, p.y_mm) for p in removed])
        ax.scatter(xy[:, 0], xy[:, 1], s=26, c="green", marker="x",
                   linewidths=1.2,
                   label=f"Removed ({len(removed)})", zorder=3)

    if new:
        xy = np.array([(p.x_mm, p.y_mm) for p in new])
        conf = np.array([p.confidence for p in new])
        sc = ax.scatter(xy[:, 0], xy[:, 1], s=32, c=conf,
                        cmap="RdYlGn", vmin=0, vmax=1,
                        edgecolors="red", linewidths=0.6,
                        label=f"NEW ({len(new)})", zorder=4)
        cbar = fig.colorbar(sc, ax=ax, shrink=0.5, pad=0.02)
        cbar.set_label("Confidence (1 = certain new)")

    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    ax.set_xlabel("X (mm from wafer centre)")
    ax.set_ylabel("Y (mm from wafer centre)")
    ax.set_title("Wafer Particle Map — New vs Existing vs Removed",
                 fontsize=12, weight="bold")
    path = out_dir / "wafer_map.png"
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    logger.info("Wafer map → %s", path)


def generate_overlay(lazy_after: LazyImage,
                     existing: List[Particle], new: List[Particle],
                     removed: List[Particle],
                     out_dir: Path, cfg: Dict[str, Any]) -> None:
    H, W = lazy_after.shape
    max_side = cfg["overlay_max_side"]
    ds = max(1.0, max(H, W) / max_side)
    sh, sw = int(H / ds), int(W / ds)

    # Build thumbnail through Pillow to avoid huge reads
    try:
        img = PilImage.open(str(lazy_after.path))
        img.draft("L", (sw, sh))
        img = img.convert("L").resize((sw, sh), PilImage.LANCZOS)
        thumb = np.asarray(img, dtype=np.uint8)
        img.close()
    except Exception:
        thumb = cv2.resize(np.asarray(lazy_after._arr), (sw, sh),
                           interpolation=cv2.INTER_AREA)

    rgb = cv2.cvtColor(thumb, cv2.COLOR_GRAY2BGR)

    def _px(p):
        return int(p.x_px / ds), int(p.y_px / ds)

    for p in existing:
        cv2.circle(rgb, _px(p), 3, (255, 180, 0), 1)
    for p in removed:
        x, y = _px(p)
        cv2.drawMarker(rgb, (x, y), (0, 200, 0), cv2.MARKER_TILTED_CROSS,
                        10, 2)
    for p in new:
        pt = _px(p)
        cv2.circle(rgb, pt, 6, (0, 0, 255), 2)
        if p.confidence < cfg["confidence_low_thresh"]:
            cv2.circle(rgb, pt, 9, (0, 255, 255), 1)

    path = out_dir / "overlay.png"
    cv2.imwrite(str(path), rgb)
    logger.info("Overlay → %s", path)


def save_section_panel(sec: GridSection, before_raw: np.ndarray,
                       after_raw: np.ndarray,
                       before_pp: np.ndarray, after_pp: np.ndarray,
                       result: SectionResult, out_dir: Path) -> None:
    sub = out_dir / "annotated_sections"
    sub.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.5))
    axes[0].imshow(before_pp, cmap="gray")
    axes[0].set_title("BEFORE")
    axes[1].imshow(after_pp, cmap="gray")
    axes[1].set_title("AFTER (aligned)")

    diff = cv2.absdiff(before_pp, after_pp)
    axes[2].imshow(diff, cmap="hot")
    axes[2].set_title("|diff|")

    for ax in axes:
        ax.axis("off")

    # draw markers (particle coords are already in full-image; convert back)
    def _local(p): return (p.x_px - sec.ox1, p.y_px - sec.oy1)
    for p in result.new:
        x, y = _local(p)
        axes[1].scatter([x], [y], s=80, facecolors="none",
                        edgecolors="red", linewidths=1.5)
    for p in result.removed:
        x, y = _local(p)
        axes[0].scatter([x], [y], s=80, marker="x", c="lime", linewidths=1.5)

    fig.suptitle(f"{sec.label}  ecc={result.ecc_score:.2f}  "
                 f"new={len(result.new)}  removed={len(result.removed)}",
                 fontsize=10)
    fig.tight_layout()
    path = sub / f"{sec.label}.png"
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════════
#  INTERACTIVE REVIEW
# ══════════════════════════════════════════════════════════════════════════════
class InteractiveReviewer:
    """Flip through new-particle candidates one-by-one.
       A/→ accept   R/Del reject   ← previous   Q quit"""

    def __init__(self, particles: List[Particle],
                 lazy_before: LazyImage, lazy_after: LazyImage,
                 half: int = 80):
        self.particles = particles
        self.lb = lazy_before
        self.la = lazy_after
        self.half = half
        self.idx = 0
        self._done = False
        if not particles:
            logger.info("No new particles to review.")
            return
        # sort uncertain first
        particles.sort(key=lambda p: p.confidence)

        self.fig, (self.ax_b, self.ax_a) = plt.subplots(1, 2, figsize=(10, 5))
        self.fig.canvas.manager.set_window_title("Particle Review")
        self.fig.subplots_adjust(bottom=0.18)

        ax_prev = self.fig.add_axes([0.12, 0.04, 0.12, 0.06])
        ax_acc  = self.fig.add_axes([0.30, 0.04, 0.15, 0.06])
        ax_rej  = self.fig.add_axes([0.50, 0.04, 0.15, 0.06])
        self.btn_prev = Button(ax_prev, "< Prev")
        self.btn_acc  = Button(ax_acc,  "Accept (A)")
        self.btn_rej  = Button(ax_rej,  "Reject (R)")
        self.btn_prev.on_clicked(lambda _: self._prev())
        self.btn_acc .on_clicked(lambda _: self._accept())
        self.btn_rej .on_clicked(lambda _: self._reject())
        self.fig.canvas.mpl_connect("key_press_event", self._key)
        self._show()
        plt.show()

    def _accept(self):
        if self._done: return
        self.particles[self.idx].reviewed = True
        self._next()

    def _reject(self):
        if self._done: return
        self.particles[self.idx].reviewed = False
        self.particles[self.idx].status = "rejected"
        self._next()

    def _prev(self):
        if self.idx > 0:
            self.idx -= 1
            self._show()

    def _next(self):
        self.idx += 1
        if self.idx >= len(self.particles):
            self._finish()
        else:
            self._show()

    def _finish(self):
        self._done = True
        plt.close(self.fig)
        logger.info("Review done.")

    def _key(self, event):
        if event.key in ("a", "right"):   self._accept()
        elif event.key in ("r", "delete"): self._reject()
        elif event.key == "left":          self._prev()
        elif event.key == "q":             self._finish()

    def _show(self):
        p = self.particles[self.idx]
        h = self.half
        H, W = self.lb.shape
        x, y = int(p.x_px), int(p.y_px)
        x0, y0 = max(0, x - h), max(0, y - h)
        x1, y1 = min(W, x + h), min(H, y + h)
        cb = self.lb.crop(x0, y0, x1, y1)
        ca = self.la.crop(x0, y0, x1, y1)

        self.ax_b.clear(); self.ax_a.clear()
        self.ax_b.imshow(cb, cmap="gray"); self.ax_b.set_title("BEFORE")
        self.ax_a.imshow(ca, cmap="gray"); self.ax_a.set_title("AFTER")
        lx, ly = p.x_px - x0, p.y_px - y0
        for ax in (self.ax_b, self.ax_a):
            ax.axhline(ly, color="lime", lw=0.5, alpha=0.6)
            ax.axvline(lx, color="lime", lw=0.5, alpha=0.6)
            ax.axis("off")
        tag = {True: "ACCEPTED", False: "REJECTED", None: ""}[p.reviewed]
        self.fig.suptitle(
            f"Particle {self.idx + 1}/{len(self.particles)}  │  "
            f"id={p.id}  conf={p.confidence:.2f}  "
            f"area={p.area_px}px  circ={p.circularity:.2f}  "
            f"({p.x_mm:+.2f}, {p.y_mm:+.2f}) mm   {tag}",
            fontsize=10)
        self.fig.canvas.draw_idle()


# ══════════════════════════════════════════════════════════════════════════════
#  PER-WAFER PIPELINE
# ══════════════════════════════════════════════════════════════════════════════
def process_wafer(before_path: Path, after_path: Path,
                  out_dir: Path, cfg: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()

    # --- open images (memmap or RAM) ---
    lazy_b = LazyImage(str(before_path), cfg["use_memmap"], cfg["memmap_dir"])
    lazy_a = LazyImage(str(after_path),  cfg["use_memmap"], cfg["memmap_dir"])
    lazy_b.open();  lazy_a.open()

    # --- auto-detect wafer geometry in BEFORE ---
    (cxB, cyB), rB, faB = detect_wafer_on_thumbnail(lazy_b, cfg)
    (cxA, cyA), rA, faA = detect_wafer_on_thumbnail(lazy_a, cfg)
    # we'll use BEFORE's geometry as the authoritative frame; per-tile ECC
    # handles the (small) misalignment between the two images.
    geom = WaferGeometry(cfg, lazy_b.shape, (cxB, cyB), rB, faB)

    # sanity check
    d_center = math.hypot(cxB - cxA, cyB - cyA)
    d_rad = abs(rB - rA)
    logger.info("  after-wafer offset: centre Δ=%.0f px  radius Δ=%.0f px",
                d_center, d_rad)
    if d_center > rB * 0.15 or d_rad > rB * 0.10:
        logger.warning("  → LARGE geometry mismatch; per-tile ECC will "
                       "compensate, but check results carefully.")

    # --- build grid ---
    sections = build_grid(geom, cfg)
    if not sections:
        raise RuntimeError("No active grid sections found.")

    # --- process every section ---
    all_existing: List[Particle] = []
    all_new:      List[Particle] = []
    all_removed:  List[Particle] = []
    ecc_scores: List[float] = []
    low_ecc_sections = 0

    t_loop = time.time()
    for i, sec in enumerate(sections, 1):
        res = process_section(sec, lazy_b, lazy_a, geom, cfg, out_dir)
        ecc_scores.append(res.ecc_score)
        if res.ecc_score < cfg["ecc_min_score"]:
            low_ecc_sections += 1
        all_existing.extend(res.existing)
        all_new     .extend(res.new)
        all_removed .extend(res.removed)
        if i % 25 == 0 or i == len(sections):
            dt = time.time() - t_loop
            logger.info("  [%3d/%3d] %s  new=%d existing=%d removed=%d  "
                        "(%.1fs, %.2fs/sec)",
                        i, len(sections), sec.label,
                        len(all_new), len(all_existing), len(all_removed),
                        dt, dt / max(1, i))

    logger.info("Section ECC stats: mean=%.2f  min=%.2f  low=%d/%d",
                float(np.mean(ecc_scores)) if ecc_scores else 0.0,
                float(np.min(ecc_scores))  if ecc_scores else 0.0,
                low_ecc_sections, len(sections))

    # --- assign IDs, global dedup across tile borders ---
    for i, p in enumerate(all_existing + all_new + all_removed):
        p.id = i

    dedup_r_px = cfg["dedup_radius_um"] / geom.um_per_px
    all_existing = dedup_particles(all_existing, dedup_r_px)
    all_new      = dedup_particles(all_new,      dedup_r_px)
    all_removed  = dedup_particles(all_removed,  dedup_r_px)

    # Confidence is already set inline by detect_changes_in_section
    # (peak_change / peak_after dominance ratio), so no separate cross-check
    # is needed.  Sort uncertain candidates first so the reviewer sees them.
    all_new.sort(key=lambda p: p.confidence)

    # --- interactive review ---
    if cfg["interactive_review"] and all_new:
        logger.info("Opening interactive review for %d new particle(s) ...",
                    len(all_new))
        InteractiveReviewer(all_new, lazy_b, lazy_a)
        rejected = [p for p in all_new if p.status == "rejected"]
        all_new  = [p for p in all_new if p.status != "rejected"]
        if rejected:
            logger.info("  rejected by user: %d", len(rejected))
    else:
        rejected = []

    # --- reports ---
    all_particles = all_existing + all_new + rejected + all_removed
    save_csv(all_particles, out_dir / "particles.csv")
    save_json_summary(all_existing, all_new, all_removed, geom,
                      out_dir / "summary.json")

    generate_wafer_map(all_existing, all_new, all_removed, geom, out_dir, cfg)
    generate_overlay(lazy_a, all_existing, all_new, all_removed, out_dir, cfg)

    # --- final log block ---
    dt = time.time() - t0
    logger.info("=" * 60)
    logger.info("DONE in %.1f s", dt)
    logger.info("  Existing (still there) : %d", len(all_existing))
    logger.info("  NEW (added)            : %d", len(all_new))
    logger.info("  REMOVED (disappeared)  : %d", len(all_removed))
    if rejected:
        logger.info("  Rejected by review     : %d", len(rejected))
    logger.info("-" * 60)
    logger.info("  Particles BEFORE test  : %d",
                len(all_existing) + len(all_removed))
    logger.info("  Particles AFTER  test  : %d",
                len(all_existing) + len(all_new))
    logger.info("  Net change             : %+d",
                len(all_new) - len(all_removed))
    logger.info("=" * 60)
    logger.info("Results: %s", out_dir.resolve())

    lazy_b.close();  lazy_a.close()
    gc.collect()


# ══════════════════════════════════════════════════════════════════════════════
#  BATCH: auto-discover pre/post pairs
# ══════════════════════════════════════════════════════════════════════════════
IMAGE_EXT = {".jpg", ".jpeg", ".png", ".tif", ".tiff", ".bmp"}

def discover_pairs(pre: Path, post: Path
                   ) -> List[Tuple[Path, Path, str]]:
    pre_files  = {f.name.lower(): f for f in pre.iterdir()
                  if f.is_file() and f.suffix.lower() in IMAGE_EXT}
    post_files = {f.name.lower(): f for f in post.iterdir()
                  if f.is_file() and f.suffix.lower() in IMAGE_EXT}
    common = sorted(set(pre_files) & set(post_files))
    if not common:
        raise FileNotFoundError(
            f"No matching filenames between\n  {pre}\n  {post}\n"
            f"pre/ and post/ must contain images with the SAME filenames.")
    return [(pre_files[n], post_files[n], Path(n).stem) for n in common]


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════
def main() -> None:
    cfg = CONFIG
    pre  = Path(cfg["pre_folder"])
    post = Path(cfg["post_folder"])
    results_root = Path(cfg["results_dir"])
    results_root.mkdir(parents=True, exist_ok=True)

    if not pre.is_dir() or not post.is_dir():
        logger.error("Missing folder: %s and/or %s", pre, post)
        logger.error("Create them and drop matching image files inside, e.g.")
        logger.error("  pre/wafer_1.jpg   post/wafer_1.jpg")
        sys.exit(1)

    pairs = discover_pairs(pre, post)
    logger.info("Found %d wafer pair(s):", len(pairs))
    for bf, af, name in pairs:
        logger.info("  %s  :  %s  ↔  %s", name, bf.name, af.name)

    for i, (bf, af, name) in enumerate(pairs, 1):
        out_dir = results_root / name
        out_dir.mkdir(parents=True, exist_ok=True)

        # per-wafer log file
        fh = logging.FileHandler(out_dir / "inspector.log", mode="w")
        fh.setFormatter(logging.Formatter(
            "%(asctime)s  %(levelname)-8s  %(message)s",
            datefmt="%H:%M:%S"))
        logging.getLogger().addHandler(fh)

        logger.info("═" * 60)
        logger.info("WAFER %d/%d : %s", i, len(pairs), name)
        logger.info("  before: %s", bf)
        logger.info("  after : %s", af)
        logger.info("═" * 60)

        try:
            process_wafer(bf, af, out_dir, cfg)
        except Exception:
            logger.exception("FAILED on %s — continuing to next wafer", name)
        finally:
            logging.getLogger().removeHandler(fh)
            fh.close()

    logger.info("All wafers processed.  Results in %s", results_root.resolve())


if __name__ == "__main__":
    main()
