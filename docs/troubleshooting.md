# Troubleshooting Guide

## Time of Flight Sensors

### Ambient Light Overflow (Error 0x04)
- **Symptoms**: Sensors report error 0x04, readings show 255mm
- **Solutions**:
  1. Move robot to a darker area
  2. Add small pieces of tape or paper to shade sensors
  3. Adjust sensor angles to avoid direct light
  4. Check sensor initialization sequence

### I2C Communication Issues
- **Symptoms**: No sensor readings, initialization failures
- **Solutions**:
  1. Verify I2C connections (SDA: GPIO0, SCL: GPIO1)
  2. Check for proper pull-up resistors (4.7kΩ to 3.3V)
  3. Measure voltage on SDA/SCL (should be ~3.3V when idle)
  4. Verify shutdown pin connections (GPIO18-20)

## Motors

### Motor Not Moving
- **Symptoms**: No motor response, no movement
- **Solutions**:
  1. Check motor power supply
  2. Verify H-bridge connections
  3. Check PWM pin connections (GPIO12-15)
  4. Verify motor initialization in code

### Uneven Movement
- **Symptoms**: Robot veers to one side
- **Solutions**:
  1. Calibrate motor speeds
  2. Check encoder readings
  3. Verify motor connections
  4. Check for mechanical issues

## Encoders

### Incorrect Counts
- **Symptoms**: Encoder values don't match movement
- **Solutions**:
  1. Verify encoder pin connections (GPIO8-11)
  2. Check for loose connections
  3. Verify encoder initialization
  4. Reset encoder counts if needed

### No Counts
- **Symptoms**: Encoder values don't change
- **Solutions**:
  1. Check encoder power supply
  2. Verify pin connections
  3. Check for proper pull-up resistors
  4. Verify interrupt handling

## General Issues

### Build Problems
- **Symptoms**: Compilation errors
- **Solutions**:
  1. Verify Pico SDK installation
  2. Check CMake configuration
  3. Clean build directory and rebuild
  4. Check for missing dependencies

### Flashing Issues
- **Symptoms**: Can't load program to Pico
- **Solutions**:
  1. Verify BOOTSEL button operation
  2. Check USB connection
  3. Try different USB cable
  4. Verify .uf2 file generation 