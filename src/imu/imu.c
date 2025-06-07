#include "../../include/imu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <math.h>

// Global variables for IMU
static float reference_heading = 0.0f;
static float current_heading = 0.0f;
static uint32_t last_update_time = 0;
static bool imu_initialized = false;

// Gyroscope calibration offsets (bias removal)
static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;
static bool gyro_calibrated = false;

// Forward declaration
static void calibrate_gyro_bias(void);

// Helper function to write to IMU register
static bool write_imu_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_blocking(IMU_I2C_PORT, MPU6500_ADDR, data, 2, false) == 2;
}

// Helper function to read from IMU register
static bool read_imu_register(uint8_t reg, uint8_t *buffer, uint8_t len) {
    if (i2c_write_blocking(IMU_I2C_PORT, MPU6500_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(IMU_I2C_PORT, MPU6500_ADDR, buffer, len, false) == len;
}

// Initialize IMU
bool init_imu(void) {
    // Initialize I2C
    i2c_init(IMU_I2C_PORT, 400 * 1000); // 400kHz
    gpio_set_function(IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA_PIN);
    gpio_pull_up(IMU_SCL_PIN);
    
    // Setup optional pins
    gpio_init(IMU_FSYNC_PIN);
    gpio_set_dir(IMU_FSYNC_PIN, GPIO_OUT);
    gpio_put(IMU_FSYNC_PIN, 0);
    
    gpio_init(IMU_INT_PIN);
    gpio_set_dir(IMU_INT_PIN, GPIO_IN);
    gpio_pull_up(IMU_INT_PIN);
    
    sleep_ms(100); // Let IMU boot up
    
    // Check WHO_AM_I register
    uint8_t who_am_i;
    if (!read_imu_register(MPU6500_WHO_AM_I, &who_am_i, 1)) {
        printf("IMU: Failed to read WHO_AM_I register\n");
        return false;
    }
    
    if (who_am_i != MPU6500_WHO_AM_I_VALUE) {
        printf("IMU: Wrong WHO_AM_I value: 0x%02x (expected 0x%02x)\n", who_am_i, MPU6500_WHO_AM_I_VALUE);
        // Send to OLED via UART - need to declare function
        extern void send_debug_to_esp32(const char *format, ...);
        send_debug_to_esp32("IMU: Wrong WHO_AM_I: 0x%02x", who_am_i);
        return false;
    }
    
    // Reset the device
    if (!write_imu_register(MPU6500_PWR_MGMT_1, 0x80)) {
        printf("IMU: Failed to reset device\n");
        extern void send_debug_to_esp32(const char *format, ...);
        send_debug_to_esp32("IMU: Reset failed");
        return false;
    }
    sleep_ms(100);
    
    // Wake up the device (disable sleep mode)
    if (!write_imu_register(MPU6500_PWR_MGMT_1, 0x00)) {
        printf("IMU: Failed to wake up device\n");
        return false;
    }
    
    // Configure gyroscope (±250 deg/s)
    if (!write_imu_register(MPU6500_GYRO_CONFIG, 0x00)) {
        printf("IMU: Failed to configure gyroscope\n");
        return false;
    }
    
    // Configure accelerometer (±2g)
    if (!write_imu_register(MPU6500_ACCEL_CONFIG, 0x00)) {
        printf("IMU: Failed to configure accelerometer\n");
        return false;
    }
    
    // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
    if (!write_imu_register(MPU6500_CONFIG, 0x07)) {
        printf("IMU: Failed to set sample rate\n");
        return false;
    }
    
    imu_initialized = true;
    last_update_time = to_ms_since_boot(get_absolute_time());
    printf("IMU: MPU-6500 initialized successfully\n");
    extern void send_debug_to_esp32(const char *format, ...);
    send_debug_to_esp32("IMU: MPU-6500 OK");
    
    // Perform gyroscope calibration
    calibrate_gyro_bias();
    
    return true;
}

// Read IMU data
bool read_imu_data(IMUData *data) {
    if (!imu_initialized || !data) {
        return false;
    }
    
    uint8_t raw_data[14];
    if (!read_imu_register(MPU6500_ACCEL_XOUT_H, raw_data, 14)) {
        return false;
    }
    
    // Parse accelerometer data (±2g range)
    int16_t accel_raw_x = (raw_data[0] << 8) | raw_data[1];
    int16_t accel_raw_y = (raw_data[2] << 8) | raw_data[3];
    int16_t accel_raw_z = (raw_data[4] << 8) | raw_data[5];
    
    data->accel_x = accel_raw_x / 16384.0f; // Convert to g
    data->accel_y = accel_raw_y / 16384.0f;
    data->accel_z = accel_raw_z / 16384.0f;
    
    // Parse temperature data
    int16_t temp_raw = (raw_data[6] << 8) | raw_data[7];
    data->temperature = (temp_raw / 340.0f) + 36.53f; // Convert to °C
    
    // Parse gyroscope data (±250 deg/s range)
    int16_t gyro_raw_x = (raw_data[8] << 8) | raw_data[9];
    int16_t gyro_raw_y = (raw_data[10] << 8) | raw_data[11];
    int16_t gyro_raw_z = (raw_data[12] << 8) | raw_data[13];
    
    // Convert to deg/s and remove bias if calibrated
    data->gyro_x = (gyro_raw_x / 131.0f) - (gyro_calibrated ? gyro_offset_x : 0.0f);
    data->gyro_y = (gyro_raw_y / 131.0f) - (gyro_calibrated ? gyro_offset_y : 0.0f);
    data->gyro_z = (gyro_raw_z / 131.0f) - (gyro_calibrated ? gyro_offset_z : 0.0f);
    
    // Integrate gyro Z (yaw) to get heading
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    float dt = (current_time - last_update_time) / 1000.0f; // Convert to seconds
    
    // Minimal debug logging (reduced frequency)
    static int debug_counter = 0;
    debug_counter++;
    
    // Very infrequent logging (every 250th reading = ~5 seconds)
    if (debug_counter >= 250) { 
        printf("IMU: gyro_z=%.1f deg/s, heading=%.1f\n", data->gyro_z, current_heading);
        debug_counter = 0;
    }
    
    last_update_time = current_time;
    
    if (dt > 0 && dt < 0.5f) { // Reasonable time delta (10ms to 500ms) - increased for ToF delays
        float heading_change = data->gyro_z * dt;
        current_heading += heading_change;
        
        // Debug heading integration issues
        static int integration_debug = 0;
        integration_debug++;
        if (integration_debug >= 100) { // Every 2 seconds
            printf("IMU INTEGRATION: heading_change=%.2f, current_heading=%.2f, gyro_z=%.1f\n", 
                   heading_change, current_heading, data->gyro_z);
            integration_debug = 0;
        }
        
        // Debug large heading changes
        if (fabs(heading_change) > 5.0f) {
            printf("IMU: Large heading change: %.1f deg (gyro_z=%.1f, dt=%.3f)\n", 
                   heading_change, data->gyro_z, dt);
        }
        
        // Keep heading in -180 to +180 range
        while (current_heading > 180.0f) current_heading -= 360.0f;
        while (current_heading < -180.0f) current_heading += 360.0f;
    } else if (dt >= 0.5f) {
        printf("IMU: Warning - very large dt=%.3fs, skipping integration\n", dt);
    }
    
    data->heading = current_heading - reference_heading;
    
    return true;
}

// Reset heading reference (call this when robot should be "straight")
void reset_heading_reference(void) {
    reference_heading = current_heading;
    printf("IMU: Heading reference reset (current: %.2f)\n", current_heading);
    extern void send_debug_to_esp32(const char *format, ...);
    send_debug_to_esp32("IMU: Heading reset %.1f", current_heading);
}

// Get current heading relative to reference
float get_current_heading(void) {
    if (!imu_initialized) return 0.0f;
    return current_heading - reference_heading;
}

// Check if IMU is connected
bool imu_is_connected(void) {
    if (!imu_initialized) return false;
    
    uint8_t who_am_i;
    return read_imu_register(MPU6500_WHO_AM_I, &who_am_i, 1) && 
           (who_am_i == MPU6500_WHO_AM_I_VALUE);
}

// Calibrate IMU (for future use)
void calibrate_imu_heading(void) {
    if (!imu_initialized) return;
    
    printf("IMU: Calibrating... Keep robot still for 3 seconds\n");
    sleep_ms(3000);
    reset_heading_reference();
    printf("IMU: Calibration complete\n");
}

// Debug heading calculation - call this to diagnose heading issues
void debug_imu_heading(void) {
    if (!imu_initialized) return;
    
    printf("\n=== IMU HEADING DEBUG ===\n");
    printf("Current heading: %.2f deg\n", current_heading);
    printf("Reference heading: %.2f deg\n", reference_heading);
    printf("Heading error: %.2f deg\n", current_heading - reference_heading);
    printf("Gyro calibrated: %s\n", gyro_calibrated ? "YES" : "NO");
    printf("Gyro Z offset: %.2f deg/s\n", gyro_offset_z);
    
    // Read raw gyro data for 10 samples
    printf("Testing gyro readings (10 samples):\n");
    for (int i = 0; i < 10; i++) {
        uint8_t raw_data[6];
        if (read_imu_register(MPU6500_GYRO_XOUT_H, raw_data, 6)) {
            int16_t gyro_raw_z = (raw_data[4] << 8) | raw_data[5];
            float gyro_z_scaled = (gyro_raw_z / 131.0f) - gyro_offset_z;
            printf("  Sample %d: raw=%d, scaled=%.2f deg/s\n", i+1, gyro_raw_z, gyro_z_scaled);
        }
        sleep_ms(100);
    }
    printf("========================\n");
    
    extern void send_debug_to_esp32(const char *format, ...);
    send_debug_to_esp32("IMU DEBUG: H=%.1f R=%.1f E=%.1f", 
                       current_heading, reference_heading, current_heading - reference_heading);
}

// Calibrate gyroscope bias (call this when robot is stationary)  
static void calibrate_gyro_bias(void) {
    printf("IMU: Calibrating gyroscope bias... Keep robot completely still!\n");
    extern void send_debug_to_esp32(const char *format, ...);
    send_debug_to_esp32("IMU: Calibrating gyro...");
    
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int samples = 100;
    
    for (int i = 0; i < samples; i++) {
        uint8_t raw_data[6];
        if (read_imu_register(MPU6500_GYRO_XOUT_H, raw_data, 6)) {
            int16_t gyro_raw_x = (raw_data[0] << 8) | raw_data[1];
            int16_t gyro_raw_y = (raw_data[2] << 8) | raw_data[3];
            int16_t gyro_raw_z = (raw_data[4] << 8) | raw_data[5];
            
            sum_x += gyro_raw_x / 131.0f;
            sum_y += gyro_raw_y / 131.0f;
            sum_z += gyro_raw_z / 131.0f;
        }
        sleep_ms(10);
    }
    
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
    gyro_calibrated = true;
    
    printf("IMU: Gyro offsets: X=%.2f Y=%.2f Z=%.2f deg/s\n", 
           gyro_offset_x, gyro_offset_y, gyro_offset_z);
    send_debug_to_esp32("IMU: Gyro cal done Z=%.1f", gyro_offset_z);
} 