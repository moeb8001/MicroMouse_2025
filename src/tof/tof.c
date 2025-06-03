#include "../include/tof.h"
#include <stdio.h>

// Initialize ToF sensors
void init_tof(void) {
    // Initialize I2C
    i2c_init(TOF_PORT, 50 * 1000);  // 50 kHz
    gpio_set_function(TOF_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA);
    gpio_pull_up(TOF_SCL);

    // Configure shutdown pins
    gpio_init(TOF1_SHDN);
    gpio_init(TOF2_SHDN);
    gpio_init(TOF3_SHDN);
    gpio_set_dir(TOF1_SHDN, GPIO_OUT);
    gpio_set_dir(TOF2_SHDN, GPIO_OUT);
    gpio_set_dir(TOF3_SHDN, GPIO_OUT);

    // Initialize sensors one at a time
    // Right sensor
    gpio_put(TOF1_SHDN, 1);
    sleep_ms(10);
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize right sensor\n");
        return;
    }
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF1_ADDR)) {
        printf("Failed to change right sensor address\n");
        return;
    }
    sleep_ms(50);

    // Left sensor
    gpio_put(TOF2_SHDN, 1);
    sleep_ms(10);
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize left sensor\n");
        return;
    }
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF2_ADDR)) {
        printf("Failed to change left sensor address\n");
        return;
    }
    sleep_ms(50);

    // Front sensor
    gpio_put(TOF3_SHDN, 1);
    sleep_ms(10);
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize front sensor\n");
        return;
    }
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF3_ADDR)) {
        printf("Failed to change front sensor address\n");
        return;
    }
    sleep_ms(50);
}

// Reset I2C bus
void reset_i2c_bus(void) {
    i2c_deinit(TOF_PORT);
    sleep_ms(10);
    i2c_init(TOF_PORT, 50 * 1000);
    gpio_set_function(TOF_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA);
    gpio_pull_up(TOF_SCL);
}

// Verify I2C bus
bool verify_i2c_bus(void) {
    uint8_t data;
    return i2c_read_blocking(TOF_PORT, TOF_DEFAULT_ADDR, &data, 1, false) > 0;
}

// Initialize VL6180X sensor
bool vl6180x_init(uint8_t addr) {
    printf("Starting ToF initialization for address 0x%02x...\n", addr);
    
    // Check if sensor is responding
    uint8_t model_id = vl6180x_read8(addr, 0x0000);
    printf("Model ID: 0x%02x (should be 0xB4)\n", model_id);
    
    if (model_id != 0xB4) {
        printf("Error: Invalid model ID. Sensor not responding correctly.\n");
        return false;
    }

    // Configure sensor with more conservative settings for ambient light
    vl6180x_write8(addr, 0x0207, 0x01);
    vl6180x_write8(addr, 0x0208, 0x01);
    vl6180x_write8(addr, 0x0096, 0x00);
    vl6180x_write8(addr, 0x0097, 0xFD);
    vl6180x_write8(addr, 0x00E3, 0x00);
    vl6180x_write8(addr, 0x00E4, 0x04);
    vl6180x_write8(addr, 0x00E5, 0x02);
    vl6180x_write8(addr, 0x00E6, 0x01);
    vl6180x_write8(addr, 0x00E7, 0x03);
    vl6180x_write8(addr, 0x00F5, 0x02);
    vl6180x_write8(addr, 0x00D9, 0x05);
    vl6180x_write8(addr, 0x00DB, 0xCE);
    vl6180x_write8(addr, 0x00DC, 0x03);
    vl6180x_write8(addr, 0x00DD, 0xF8);
    vl6180x_write8(addr, 0x009F, 0x00);
    vl6180x_write8(addr, 0x00A3, 0x3C);
    vl6180x_write8(addr, 0x00B7, 0x00);
    vl6180x_write8(addr, 0x00BB, 0x3C);
    vl6180x_write8(addr, 0x00B2, 0x09);
    vl6180x_write8(addr, 0x00CA, 0x09);
    vl6180x_write8(addr, 0x0198, 0x01);
    vl6180x_write8(addr, 0x01B0, 0x17);
    vl6180x_write8(addr, 0x01AD, 0x00);
    vl6180x_write8(addr, 0x00FF, 0x05);
    vl6180x_write8(addr, 0x0100, 0x05);
    vl6180x_write8(addr, 0x0199, 0x05);
    vl6180x_write8(addr, 0x01A6, 0x1B);
    vl6180x_write8(addr, 0x01AC, 0x3E);
    vl6180x_write8(addr, 0x01A7, 0x1F);
    vl6180x_write8(addr, 0x0030, 0x00);

    // More conservative ambient light handling
    vl6180x_write8(addr, 0x0011, 0x10);  // Enable range and ambient light measurement
    vl6180x_write8(addr, 0x010A, 0x10);  // Set ALS integration time to 25ms (reduced from 50ms)
    vl6180x_write8(addr, 0x003F, 0x20);  // Set ALS gain to 5 (reduced from 10)
    vl6180x_write8(addr, 0x0031, 0xFF);  // Set ALS threshold to maximum
    vl6180x_write8(addr, 0x0040, 0x40);  // Set ALS analog gain to 0.5 (reduced from 1.0)
    vl6180x_write8(addr, 0x002E, 0x01);  // Set ALS auto gain to enabled
    vl6180x_write8(addr, 0x001B, 0x09);  // Set range check to enabled
    vl6180x_write8(addr, 0x003E, 0x31);  // Set range timing to 100ms
    vl6180x_write8(addr, 0x0014, 0x24);  // Set range interrupt to enabled
    vl6180x_write8(addr, 0x0016, 0x00);  // Set range error threshold to 0
    
    printf("ToF initialization complete for address 0x%02x\n", addr);
    return true;
}

// Change VL6180X address
bool vl6180x_change_addr(uint8_t old_addr, uint8_t new_addr) {
    printf("Changing address from 0x%02x to 0x%02x\n", old_addr, new_addr);
    
    // First verify the sensor is responding at old address
    uint8_t model_id = vl6180x_read8(old_addr, 0x0000);
    if (model_id != 0xB4) {
        printf("Error: Sensor not responding at old address 0x%02x\n", old_addr);
        return false;
    }
    
    // Change the address
    vl6180x_write8(old_addr, 0x0212, new_addr);
    sleep_ms(100);  // Give sensor more time to update address
    
    // Verify the sensor is responding at new address
    model_id = vl6180x_read8(new_addr, 0x0000);
    if (model_id != 0xB4) {
        printf("Error: Sensor not responding at new address 0x%02x\n", new_addr);
        return false;
    }
    
    printf("Successfully changed address to 0x%02x\n", new_addr);
    return true;
}

// Read VL6180X range
uint8_t vl6180x_read_range(uint8_t addr) {
    static uint8_t error_count[3] = {0, 0, 0};  // Track errors for each sensor
    static uint8_t last_valid_range[3] = {0, 0, 0};  // Store last valid range for each sensor
    
    // Get sensor index (0=right, 1=left, 2=front)
    uint8_t sensor_idx = (addr == TOF1_ADDR) ? 0 : (addr == TOF2_ADDR) ? 1 : 2;
    
    // Check if sensor is ready
    uint8_t status = vl6180x_read8(addr, 0x04F);
    if (status & 0x01) {
        error_count[sensor_idx]++;
        printf("Error: Sensor at 0x%02x reports error status: 0x%02x (count: %d)\n", 
               addr, status, error_count[sensor_idx]);
        
        if (status == 0x04) {  // Ambient light overflow
            printf("Ambient light overflow detected. Adjusting sensor configuration...\n");
            // More aggressive ambient light handling
            vl6180x_write8(addr, 0x010A, 0x08);  // Reduce ALS integration time to 12.5ms
            vl6180x_write8(addr, 0x003F, 0x10);  // Reduce ALS gain to 2.5
            vl6180x_write8(addr, 0x0040, 0x20);  // Reduce ALS analog gain to 0.25
            sleep_ms(50);  // Give sensor time to adjust
            
            // If we have a last valid range, return it instead of 255
            if (last_valid_range[sensor_idx] > 0) {
                return last_valid_range[sensor_idx];
            }
        }
        
        // If we've had too many errors, try to reinitialize the sensor
        if (error_count[sensor_idx] > 10) {
            printf("Too many errors, attempting to reinitialize sensor at 0x%02x\n", addr);
            vl6180x_init(addr);
            error_count[sensor_idx] = 0;
        }
        
        return 255;
    }

    // Start measurement
    vl6180x_write8(addr, 0x018, 0x01);
    
    // Wait for measurement to complete
    uint32_t timeout = 0;
    while ((vl6180x_read8(addr, 0x04f) & 0x07) == 0x00) {
        sleep_ms(1);
        timeout++;
        if (timeout > 100) {  // 100ms timeout
            printf("Error: Sensor at 0x%02x measurement timeout\n", addr);
            return 255;
        }
    }
    
    // Read result
    uint8_t range = vl6180x_read8(addr, 0x062);
    
    // If we got a valid reading, update our last valid range
    if (range != 255) {
        last_valid_range[sensor_idx] = range;
        error_count[sensor_idx] = 0;  // Reset error count on successful reading
    }
    
    return range;
}

// Scan I2C bus
void scan_i2c_bus(void) {
    printf("Scanning I2C bus...\n");
    for (int addr = 0x08; addr <= 0x77; addr++) {
        uint8_t data;
        if (i2c_read_blocking(TOF_PORT, addr, &data, 1, false) > 0) {
            printf("Device found at address 0x%02x\n", addr);
        }
    }
}

// Helper function to write to VL6180X registers
void vl6180x_write8(uint8_t addr, uint16_t reg, uint8_t value) {
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, value };
    int ret = i2c_write_blocking(TOF_PORT, addr, buf, 3, false);
    if (ret != 3) {
        printf("Write error: addr=0x%02x, reg=0x%04x, ret=%d\n", addr, reg, ret);
    }
}

// Helper function to read from VL6180X registers
uint8_t vl6180x_read8(uint8_t addr, uint16_t reg) {
    uint8_t reg_buf[2] = { reg >> 8, reg & 0xFF };
    uint8_t val;
    int ret = i2c_write_blocking(TOF_PORT, addr, reg_buf, 2, true);
    if (ret != 2) {
        printf("Read setup error: addr=0x%02x, reg=0x%04x, ret=%d\n", addr, reg, ret);
        return 0xFF;
    }
    ret = i2c_read_blocking(TOF_PORT, addr, &val, 1, false);
    if (ret != 1) {
        printf("Read error: addr=0x%02x, reg=0x%04x, ret=%d\n", addr, reg, ret);
        return 0xFF;
    }
    return val;
} 