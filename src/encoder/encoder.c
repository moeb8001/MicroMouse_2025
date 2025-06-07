#include "../include/encoder.h"

// Encoder state variables
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint8_t lastLeftState = 0;
volatile uint8_t lastRightState = 0;

// Initialize encoders
void init_encoders(void) {
    // Configure encoder pins
    gpio_set_function(ENCODER_L_A, GPIO_FUNC_SIO);
    gpio_set_function(ENCODER_L_B, GPIO_FUNC_SIO);
    gpio_set_function(ENCODER_R_A, GPIO_FUNC_SIO);
    gpio_set_function(ENCODER_R_B, GPIO_FUNC_SIO);

    // Set pins as inputs with pull-ups
    gpio_set_dir(ENCODER_L_A, GPIO_IN);
    gpio_set_dir(ENCODER_L_B, GPIO_IN);
    gpio_set_dir(ENCODER_R_A, GPIO_IN);
    gpio_set_dir(ENCODER_R_B, GPIO_IN);
    gpio_pull_up(ENCODER_L_A);
    gpio_pull_up(ENCODER_L_B);
    gpio_pull_up(ENCODER_R_A);
    gpio_pull_up(ENCODER_R_B);

    // Set up interrupts
    gpio_set_irq_enabled(ENCODER_L_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_L_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_R_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_R_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Set up callback
    gpio_set_irq_callback(encoder_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Initialize last states
    lastLeftState = (gpio_get(ENCODER_L_A) << 1) | gpio_get(ENCODER_L_B);
    lastRightState = (gpio_get(ENCODER_R_A) << 1) | gpio_get(ENCODER_R_B);
}

// Encoder interrupt callback
void encoder_callback(uint gpio, uint32_t events) {
    uint8_t newState;
    
    if (gpio == ENCODER_L_A || gpio == ENCODER_L_B) {
        newState = (gpio_get(ENCODER_L_A) << 1) | gpio_get(ENCODER_L_B);
        if (newState != lastLeftState) {
            if ((lastLeftState == 0 && newState == 1) ||
                (lastLeftState == 1 && newState == 3) ||
                (lastLeftState == 3 && newState == 2) ||
                (lastLeftState == 2 && newState == 0)) {
                leftEncoderCount++;
            } else {
                leftEncoderCount--;
            }
            lastLeftState = newState;
        }
    }
    
    if (gpio == ENCODER_R_A || gpio == ENCODER_R_B) {
        newState = (gpio_get(ENCODER_R_A) << 1) | gpio_get(ENCODER_R_B);
        if (newState != lastRightState) {
            if ((lastRightState == 0 && newState == 1) ||
                (lastRightState == 1 && newState == 3) ||
                (lastRightState == 3 && newState == 2) ||
                (lastRightState == 2 && newState == 0)) {
                rightEncoderCount++;
            } else {
                rightEncoderCount--;
            }
            lastRightState = newState;
        }
    }
}

// Reset encoder counts
void reset_encoders(void) {
    leftEncoderCount = 0;
    rightEncoderCount = 0;
}

int32_t get_left_encoder_count(void) {
    return leftEncoderCount;
}

int32_t get_right_encoder_count(void) {
    return rightEncoderCount;
} 