#include "../include/motor.h"

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

// Set motor speeds
void setMotors(bool run, int speed) {
    if (!run) {
        // Stop motors
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
        return;
    }

    // Set left motor
    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_L_IN1, speed);
        pwm_set_gpio_level(MOTOR_L_IN2, 0);
    } else {
        pwm_set_gpio_level(MOTOR_L_IN1, 0);
        pwm_set_gpio_level(MOTOR_L_IN2, -speed);
    }

    // Set right motor
    if (speed > 0) {
        pwm_set_gpio_level(MOTOR_R_IN1, speed);
        pwm_set_gpio_level(MOTOR_R_IN2, 0);
    } else {
        pwm_set_gpio_level(MOTOR_R_IN1, 0);
        pwm_set_gpio_level(MOTOR_R_IN2, -speed);
    }
}

// Turn left
void turnLeft(void) {
    // Stop left motor, run right motor
    pwm_set_gpio_level(MOTOR_L_IN1, 0);
    pwm_set_gpio_level(MOTOR_L_IN2, 0);
    pwm_set_gpio_level(MOTOR_R_IN1, DUTY_50);
    pwm_set_gpio_level(MOTOR_R_IN2, 0);
}

// Turn right
void turnRight(void) {
    // Run left motor, stop right motor
    pwm_set_gpio_level(MOTOR_L_IN1, DUTY_50);
    pwm_set_gpio_level(MOTOR_L_IN2, 0);
    pwm_set_gpio_level(MOTOR_R_IN1, 0);
    pwm_set_gpio_level(MOTOR_R_IN2, 0);
} 