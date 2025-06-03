#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

// Encoder pins
#define ENCODER_L_A 8
#define ENCODER_L_B 9
#define ENCODER_R_A 10
#define ENCODER_R_B 11

// Encoder state variables
extern volatile int32_t leftEncoderCount;
extern volatile int32_t rightEncoderCount;
extern volatile uint8_t lastLeftState;
extern volatile uint8_t lastRightState;

// Function declarations
void init_encoders(void);
void encoder_callback(uint gpio, uint32_t events);
void reset_encoders(void);

#endif // ENCODER_H 