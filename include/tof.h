#ifndef TOF_H
#define TOF_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
#define TOF_PORT i2c0
#define TOF_SDA 0
#define TOF_SCL 1
#define TOF1_SHDN 18  // Right sensor
#define TOF2_SHDN 19  // Left sensor
#define TOF3_SHDN 20  // Front sensor
#define TOF_DEFAULT_ADDR 0x29  // Default VL6180X address
#define TOF1_ADDR 0x2A  // Right sensor address
#define TOF2_ADDR 0x2B  // Left sensor address
#define TOF3_ADDR 0x2C  // Front sensor address

// Function declarations
void init_tof(void);
void reset_i2c_bus(void);
bool verify_i2c_bus(void);
uint8_t vl6180x_read_range(uint8_t addr);
void scan_i2c_bus(void);

#endif // TOF_H 