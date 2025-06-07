#ifndef IMU_H
#define IMU_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdbool.h>

// I2C Configuration for IMU
#define IMU_I2C_PORT i2c1
#define IMU_SDA_PIN 2
#define IMU_SCL_PIN 3
#define IMU_FSYNC_PIN 4
#define IMU_INT_PIN 5

// MPU-6500 I2C Address
#define MPU6500_ADDR 0x68

// MPU-6500 Register Addresses
#define MPU6500_WHO_AM_I     0x75
#define MPU6500_PWR_MGMT_1   0x6B
#define MPU6500_PWR_MGMT_2   0x6C
#define MPU6500_CONFIG       0x1A
#define MPU6500_GYRO_CONFIG  0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_GYRO_XOUT_H  0x43
#define MPU6500_TEMP_OUT_H   0x41

// Expected WHO_AM_I value for MPU-6500
#define MPU6500_WHO_AM_I_VALUE 0x70

// Data structure for IMU readings
typedef struct {
    float accel_x, accel_y, accel_z;    // Accelerometer (g)
    float gyro_x, gyro_y, gyro_z;       // Gyroscope (deg/s)
    float temperature;                   // Temperature (°C)
    float heading;                       // Integrated heading (degrees)
} IMUData;

// Function declarations
bool init_imu(void);
bool read_imu_data(IMUData *data);
void calibrate_imu_heading(void);
float get_current_heading(void);
void reset_heading_reference(void);
bool imu_is_connected(void);
void debug_imu_heading(void); // For debugging heading issues

#endif // IMU_H 