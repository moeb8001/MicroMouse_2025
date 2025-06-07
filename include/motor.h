#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Pin definitions - SWAPPED to match physical wiring
#define MOTOR_L_IN1 12  // Was 14, now using right pins for left motor
#define MOTOR_L_IN2 13  // Was 15, now using right pins for left motor  
#define MOTOR_R_IN1 14  // Was 12, now using left pins for right motor
#define MOTOR_R_IN2 15  // Was 13, now using left pins for right motor

// PWM settings
#define PWM_FREQ 20000  // 20 kHz
#define PWM_WRAP 255    // 8-bit resolution
#define DUTY_50 100     // Increased from 80, was originally 128 - reasonable middle ground

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
void setIndividualMotors(int16_t left_speed, int16_t right_speed);

#endif // MOTOR_H 