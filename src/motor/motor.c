#include "../include/motor.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Forward declaration for the helper function
static void set_motor_directions(int8_t left_dir, int8_t right_dir);

// Initialize motors and PWM
void init_motors(void) {
    // Configure PWM pins
    gpio_set_function(MOTOR_L_IN1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_L_IN2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_IN1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_IN2, GPIO_FUNC_PWM);

    // Get PWM slices
    uint slice_l = pwm_gpio_to_slice_num(MOTOR_L_IN1);
    uint slice_r = pwm_gpio_to_slice_num(MOTOR_R_IN1);

    // Set PWM frequency
    pwm_set_wrap(slice_l, PWM_WRAP);
    pwm_set_wrap(slice_r, PWM_WRAP);
    pwm_set_clkdiv(slice_l, 125.0f);  // 125MHz / 125 = 1MHz
    pwm_set_clkdiv(slice_r, 125.0f);

    // Enable PWM
    pwm_set_enabled(slice_l, true);
    pwm_set_enabled(slice_r, true);

    // Initialize motors to stop
    setMotors(false, 0);
}

/**
 * @brief Sets the direction of the motors.
 * @param left_dir -1 for reverse, 0 for stop, 1 for forward
 * @param right_dir -1 for reverse, 0 for stop, 1 for forward
 *
 * NOTE: The right motor is physically assembled backward, so its logic is inverted.
 *       A "forward" command (1) for the right motor means setting its IN2 pin.
 */
static void set_motor_directions(int8_t left_dir, int8_t right_dir) {
    // Left motor
    if (left_dir > 0) { // Forward
        pwm_set_gpio_level(MOTOR_L_IN1, PWM_WRAP);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
    } else if (left_dir < 0) { // Reverse
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, PWM_WRAP);
    } else { // Stop
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
    }

    // Right motor (inverted due to physical assembly)
    if (right_dir > 0) { // Forward
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, PWM_WRAP);
    } else if (right_dir < 0) { // Reverse
        pwm_set_gpio_level(MOTOR_R_IN1, PWM_WRAP);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
    } else { // Stop
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
    }
}

// Sets both motors to the same speed and direction.
// This is the known-good function used for Button 1.
void setMotors(bool run, int speed) {
    if (!run) {
        // Stop motors
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
        return;
    }

    // Set left motor (reversed direction logic)
    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, speed);
    } else {
        pwm_set_gpio_level(MOTOR_L_IN1, -speed);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
    }

    // Set right motor (reversed direction logic)
    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_R_IN1, speed);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
    } else {
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, -speed);
    }
}

// Perform a point turn to the left.
void turnLeft(void) {
    setIndividualMotors(-DUTY_50, DUTY_50); // Left backward, Right forward
}

// Perform a point turn to the right.
void turnRight(void) {
    setIndividualMotors(DUTY_50, -DUTY_50); // Left forward, Right backward
}

/**
 * @brief Sets the speed and direction of each motor individually.
 *
 * @param left_speed Speed for the left motor. Positive is forward, negative is backward.
 * @param right_speed Speed for the right motor. Positive is forward, negative is backward.
 * NOTE: This function accounts for the physically inverted right motor.
 */
void setIndividualMotors(int16_t left_speed, int16_t right_speed) {
    // --- Left Motor (Reversed direction logic is correct for this hardware) ---
    if (left_speed >= 0) { // Forward
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, left_speed);
    } else { // Backward
        pwm_set_gpio_level(MOTOR_L_IN1, -left_speed);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
    }

    // --- Right Motor (Reversed direction logic is correct for this hardware) ---
    if (right_speed >= 0) { // Forward
        pwm_set_gpio_level(MOTOR_R_IN1, right_speed);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
    } else { // Backward
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, -right_speed);
    }
} 