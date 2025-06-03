#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "include/tof.h"
#include "include/motor.h"
#include "include/encoder.h"

// Button pin
#define BUTTON_PIN 6

// PWM settings
#define PWM_FREQ 20000  // 20 kHz
#define PWM_WRAP 255    // 8-bit resolution
#define DUTY_50 128     // 50% of 255

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("MicroMouse Robot Starting...\n");

    // Initialize components
    init_tof();
    init_motors();
    init_encoders();

    // Configure button
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Main loop
    while (true) {
        // Read ToF sensors
        uint8_t right_dist = vl6180x_read_range(TOF1_ADDR);
        uint8_t left_dist = vl6180x_read_range(TOF2_ADDR);
        uint8_t front_dist = vl6180x_read_range(TOF3_ADDR);

        // Print sensor readings
        printf("ToF: R=%d L=%d F=%d mm\n", right_dist, left_dist, front_dist);
        printf("Enc: L=%ld R=%ld\n", leftEncoderCount, rightEncoderCount);

        // Check button press
        if (!gpio_get(BUTTON_PIN)) {
            printf("Button pressed - Running motor test\n");
            
            // Run motors forward
            setMotors(true, DUTY_50);
            sleep_ms(1000);
            
            // Turn left
            turnLeft();
            sleep_ms(500);
            
            // Turn right
            turnRight();
            sleep_ms(500);
            
            // Stop motors
            setMotors(false, 0);
            
            // Reset encoders
            reset_encoders();
            
            // Wait for button release
            while (!gpio_get(BUTTON_PIN)) {
                sleep_ms(10);
            }
        }

        sleep_ms(100);  // 10Hz update rate
    }

    return 0;
}
