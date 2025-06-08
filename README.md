# MicroMouse Robot

A Raspberry Pi Pico-based MicroMouse robot implementation.

## Project Structure

```
MicroMouse_Robot/
├── src/                    # Source files
│   ├── tof/               # Time of Flight sensor code
│   ├── motor/             # Motor control code
│   ├── encoder/           # Encoder handling code
│   └── main.c             # Main program
├── include/               # Header files
│   ├── tof.h             # ToF sensor definitions
│   ├── motor.h           # Motor control definitions
│   └── encoder.h         # Encoder definitions
├── docs/                  # Documentation
│   ├── setup.md          # Setup instructions
│   ├── pinout.md         # Pin assignments
│   └── troubleshooting.md # Common issues and solutions
└── CMakeLists.txt        # Build configuration
```

## Hardware Setup

### Time of Flight Sensors
- 3x VL6180X ToF sensors
- I2C pins: GPIO0 (SDA), GPIO1 (SCL)
- Shutdown pins: GPIO18 (Right), GPIO19 (Left), GPIO20 (Front)

### Motors
- 2x DC motors with H-bridge
- PWM pins: GPIO12-15
- PWM frequency: 20kHz

### Encoders
- 2x Quadrature encoders
- Pins: GPIO8-11

## Building and Flashing

1. Install the Raspberry Pi Pico SDK
2. Build the project:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```
3. Flash to Pico:
   - Hold BOOTSEL button
   - Press and release RESET
   - Release BOOTSEL
   - Copy build/MicroMouse_Robot.uf2 to the Pico

## Usage

1. Power on the robot
2. Press the button (GPIO6) to start motor test sequence
3. Monitor ToF sensor readings and encoder values

## Troubleshooting

See [troubleshooting.md](docs/troubleshooting.md) for common issues and solutions.

