#ifndef TOF_H
#define TOF_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdbool.h>
#include <stdint.h>

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
bool vl6180x_init(uint8_t addr);
bool vl6180x_change_addr(uint8_t old_addr, uint8_t new_addr);
uint8_t vl6180x_read_range(uint8_t addr);
void scan_i2c_bus(void);
void vl6180x_write8(uint8_t addr, uint16_t reg, uint8_t value);
uint8_t vl6180x_read8(uint8_t addr, uint16_t reg);
bool vl6180x_is_present(uint8_t addr);
uint8_t vl6180x_read_model_id(uint8_t addr);

#endif // TOF_H 