#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Pin definitions
#define MOTOR_L_IN1 14 
#define MOTOR_L_IN2 15 
#define MOTOR_R_IN1 12 
#define MOTOR_R_IN2 13 

// PWM settings
#define PWM_FREQ 20000  // 20 kHz
#define PWM_WRAP 255    // 8-bit resolution
#define DUTY_50 128     // 50% of 255

// PWM slice assignments
#define PWM_L_IN1_SLICE 7  // GPIO14 -> PWM slice 7
#define PWM_L_IN2_SLICE 7  // GPIO15 -> PWM slice 7
#define PWM_R_IN1_SLICE 6  // GPIO12 -> PWM slice 6
#define PWM_R_IN2_SLICE 6  // GPIO13 -> PWM slice 6

// Function declarations
void init_motors(void);
void setMotors(bool run, int speed);
void turnLeft(void);
void turnRight(void);

#endif // MOTOR_H 