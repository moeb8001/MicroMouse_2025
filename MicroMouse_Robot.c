#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/structs/pwm.h"

// PWM channel definitions
#define PWM_CHAN_A 0
#define PWM_CHAN_B 1

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// Using I2C0 on GPIO0 (SDA) and GPIO1 (SCL) running at 100KHz
#define TOF_PORT i2c0
#define TOF_SDA 0
#define TOF_SCL 1
#define TOF1_SHDN 18  // Left sensor
#define TOF2_SHDN 19  // Front sensor
#define TOF3_SHDN 20  // Right sensor
#define TOF_DEFAULT_ADDR 0x29  // Default VL6180X address
#define TOF1_ADDR 0x2A  // Left sensor address
#define TOF2_ADDR 0x2B  // Front sensor address
#define TOF3_ADDR 0x2C  // Right sensor address

// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}


int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    return 0;
}



// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Pin definitions - User confirmed absolute correct pins
#define MOTOR_L_IN1 14 
#define MOTOR_L_IN2 15 
#define MOTOR_R_IN1 12 
#define MOTOR_R_IN2 13 

// Encoder pins
#define ENCODER_L_A 8
#define ENCODER_L_B 9
#define ENCODER_R_A 10
#define ENCODER_R_B 11

// Button pin
#define BTN1 6

// PWM settings
#define PWM_FREQ 20000  // 20 kHz
#define PWM_WRAP 255    // 8-bit resolution
#define DUTY_50 128     // 50% of 255

// PWM slice assignments
#define PWM_L_IN1_SLICE 7  // GPIO14 -> PWM slice 7
#define PWM_L_IN2_SLICE 7  // GPIO15 -> PWM slice 7
#define PWM_R_IN1_SLICE 6  // GPIO12 -> PWM slice 6
#define PWM_R_IN2_SLICE 6  // GPIO13 -> PWM slice 6

// Encoder state variables
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;
volatile uint8_t lastLeftState = 0;
volatile uint8_t lastRightState = 0;

// ToF Sensor functions
void vl6180x_write8(uint8_t addr, uint16_t reg, uint8_t value) {
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, value };
    int ret = i2c_write_blocking(TOF_PORT, addr, buf, 3, false);
    if (ret != 3) {
        printf("Write error: addr=0x%02x, reg=0x%04x, ret=%d\n", addr, reg, ret);
    }
}

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

bool vl6180x_init(uint8_t addr) {
    printf("Starting ToF initialization for address 0x%02x...\n", addr);
    
    // Check if sensor is responding
    uint8_t model_id = vl6180x_read8(addr, 0x0000);
    printf("Model ID: 0x%02x (should be 0xB4)\n", model_id);
    
    if (model_id != 0xB4) {
        printf("Error: Invalid model ID. Sensor not responding correctly.\n");
        return false;
    }

    // Mandatory private registers initialization
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

    // Recommended public register configuration
    vl6180x_write8(addr, 0x0011, 0x10);  // Enable range and ambient light measurement
    vl6180x_write8(addr, 0x010A, 0x30);  // Set ALS integration time to 100ms
    vl6180x_write8(addr, 0x003F, 0x46);  // Set ALS gain to 20
    vl6180x_write8(addr, 0x0031, 0xFF);  // Set ALS threshold to maximum
    vl6180x_write8(addr, 0x0040, 0x63);  // Set ALS analog gain to 1.0
    vl6180x_write8(addr, 0x002E, 0x01);  // Set ALS auto gain to enabled
    vl6180x_write8(addr, 0x001B, 0x09);  // Set range check to enabled
    vl6180x_write8(addr, 0x003E, 0x31);  // Set range timing to 100ms
    vl6180x_write8(addr, 0x0014, 0x24);  // Set range interrupt to enabled
    vl6180x_write8(addr, 0x0016, 0x00);  // Set range error threshold to 0
    
    printf("ToF initialization complete for address 0x%02x\n", addr);
    return true;
}

// Function to scan I2C bus
void scan_i2c_bus() {
    printf("\nScanning I2C bus...\n");
    for (uint8_t addr = 0x01; addr < 0x7F; addr++) {
        uint8_t reg_buf[2] = { 0x00, 0x00 };  // Try to read model ID register
        int ret = i2c_write_blocking(TOF_PORT, addr, reg_buf, 2, true);
        if (ret == 2) {
            uint8_t val;
            ret = i2c_read_blocking(TOF_PORT, addr, &val, 1, false);
            if (ret == 1) {
                printf("Found device at address 0x%02x (Model ID: 0x%02x)\n", addr, val);
            }
        }
    }
    printf("I2C scan complete\n");
}

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

// Function to reset I2C bus
void reset_i2c_bus() {
    printf("Resetting I2C bus...\n");
    
    // Deinitialize I2C
    i2c_deinit(TOF_PORT);
    sleep_ms(10);
    
    // Set pins as GPIO
    gpio_set_function(TOF_SDA, GPIO_FUNC_SIO);
    gpio_set_function(TOF_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(TOF_SDA, GPIO_OUT);
    gpio_set_dir(TOF_SCL, GPIO_OUT);
    
    // Generate I2C reset sequence
    gpio_put(TOF_SDA, 1);
    gpio_put(TOF_SCL, 1);
    sleep_ms(1);
    
    // Clock 9 times with SDA high
    for(int i = 0; i < 9; i++) {
        gpio_put(TOF_SCL, 0);
        sleep_ms(1);
        gpio_put(TOF_SCL, 1);
        sleep_ms(1);
    }
    
    // Generate STOP condition
    gpio_put(TOF_SDA, 0);
    sleep_ms(1);
    gpio_put(TOF_SCL, 1);
    sleep_ms(1);
    gpio_put(TOF_SDA, 1);
    sleep_ms(1);
    
    // Reinitialize I2C
    i2c_init(TOF_PORT, 50*1000);
    gpio_set_function(TOF_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA);
    gpio_pull_up(TOF_SCL);
    sleep_ms(10);
}

// Function to verify I2C bus health
bool verify_i2c_bus() {
    printf("Verifying I2C bus...\n");
    
    // Try to read from default address
    uint8_t model_id = vl6180x_read8(TOF_DEFAULT_ADDR, 0x0000);
    if (model_id == 0xB4) {
        printf("I2C bus is healthy (found sensor at default address)\n");
        return true;
    }
    
    printf("I2C bus verification failed\n");
    return false;
}

void init_tof() {
    printf("\n=== ToF Sensor Initialization ===\n");

    // 1) Initialize I2C peripheral
    i2c_init(TOF_PORT, 50 * 1000);  // 50 kHz (slower but more reliable)
    printf("I2C initialized at 50kHz\n");

    // 2) Configure SDA/SCL pins for I2C + pull-ups
    gpio_set_function(TOF_SDA, GPIO_FUNC_I2C);
    gpio_set_function(TOF_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(TOF_SDA);
    gpio_pull_up(TOF_SCL);
    printf("I2C pins configured (SDA: GPIO %d, SCL: GPIO %d)\n", TOF_SDA, TOF_SCL);

    // 3) Initialize all three XSHUT (shutdown) pins as outputs
    gpio_init(TOF1_SHDN);
    gpio_init(TOF2_SHDN);
    gpio_init(TOF3_SHDN);
    gpio_set_dir(TOF1_SHDN, GPIO_OUT);
    gpio_set_dir(TOF2_SHDN, GPIO_OUT);
    gpio_set_dir(TOF3_SHDN, GPIO_OUT);
    printf("Shutdown pins initialized (GPIO %d, %d, %d)\n", TOF1_SHDN, TOF2_SHDN, TOF3_SHDN);

    // 4) DRIVE ALL SHUTDOWN PINS LOW to disable all three sensors initially
    gpio_put(TOF1_SHDN, false);
    gpio_put(TOF2_SHDN, false);
    gpio_put(TOF3_SHDN, false);
    sleep_ms(10);  // let them power down fully

    // ----- Initialize LEFT sensor first -----
    printf("\nInitializing left sensor...\n");
    // Turn ON left sensor only:
    gpio_put(TOF1_SHDN, true);
    sleep_ms(100);  // wait for it to boot
    
    // Check its model ID at the default address (0x29)
    uint8_t model_id = vl6180x_read8(TOF_DEFAULT_ADDR, 0x0000);
    printf("Left sensor model ID: 0x%02x (should be 0xB4)\n", model_id);
    if (model_id != 0xB4) {
        printf("Error: Left sensor not responding at default address\n");
        return;
    }
    // Run the standard VL6180 init sequence (writing all priv/pub regs)
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize left sensor\n");
        return;
    }
    // Change its I²C address to 0x2A
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF1_ADDR)) {
        printf("Failed to change left sensor address\n");
        return;
    }
    // Verify
    model_id = vl6180x_read8(TOF1_ADDR, 0x0000);
    printf("Left sensor model ID at new address (0x%02x): 0x%02x\n", TOF1_ADDR, model_id);
    sleep_ms(100);

    // ----- Initialize FRONT sensor next -----
    printf("\nInitializing front sensor...\n");
    // Turn ON front sensor (others that respond at default 0x29 are still off;
    // left is ON but now lives at 0x2A, not 0x29, so no address conflict)
    gpio_put(TOF2_SHDN, true);
    sleep_ms(100);

    // Read at default address (0x29), since only the newly-enabled front is at 0x29
    model_id = vl6180x_read8(TOF_DEFAULT_ADDR, 0x0000);
    printf("Front sensor model ID: 0x%02x (should be 0xB4)\n", model_id);
    if (model_id != 0xB4) {
        printf("Error: Front sensor not responding at default address\n");
        return;
    }
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize front sensor\n");
        return;
    }
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF2_ADDR)) {
        printf("Failed to change front sensor address\n");
        return;
    }
    model_id = vl6180x_read8(TOF2_ADDR, 0x0000);
    printf("Front sensor model ID at new address (0x%02x): 0x%02x\n", TOF2_ADDR, model_id);
    sleep_ms(100);

    // ----- Initialize RIGHT sensor last -----
    printf("\nInitializing right sensor...\n");
    // Turn ON right sensor
    gpio_put(TOF3_SHDN, true);
    sleep_ms(100);

    // Again, only "right" is at 0x29 now
    model_id = vl6180x_read8(TOF_DEFAULT_ADDR, 0x0000);
    printf("Right sensor model ID: 0x%02x (should be 0xB4)\n", model_id);
    if (model_id != 0xB4) {
        printf("Error: Right sensor not responding at default address\n");
        return;
    }
    if (!vl6180x_init(TOF_DEFAULT_ADDR)) {
        printf("Failed to initialize right sensor\n");
        return;
    }
    if (!vl6180x_change_addr(TOF_DEFAULT_ADDR, TOF3_ADDR)) {
        printf("Failed to change right sensor address\n");
        return;
    }
    model_id = vl6180x_read8(TOF3_ADDR, 0x0000);
    printf("Right sensor model ID at new address (0x%02x): 0x%02x\n", TOF3_ADDR, model_id);

    printf("\n=== ToF Initialization Complete ===\n");
}

uint8_t vl6180x_read_range(uint8_t addr) {
    // Check if sensor is ready
    uint8_t status = vl6180x_read8(addr, 0x04F);
    if (status & 0x01) {
        printf("Error: Sensor at 0x%02x reports error status: 0x%02x\n", addr, status);
        if (status == 0x04) {
            printf("Ambient light overflow detected. Adjusting sensor configuration...\n");
            // Try to reduce sensitivity to ambient light
            vl6180x_write8(addr, 0x010A, 0x20);  // Reduce ALS integration time
            vl6180x_write8(addr, 0x003F, 0x40);  // Reduce ALS gain
            sleep_ms(50);  // Give sensor time to adjust
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
    
    // Check for error conditions
    if (range == 255) {
        uint8_t error = vl6180x_read8(addr, 0x04F);
        printf("Error: Sensor at 0x%02x reports error: 0x%02x\n", addr, error);
        if (error == 0x04) {
            printf("Ambient light overflow detected. Adjusting sensor configuration...\n");
            // Try to reduce sensitivity to ambient light
            vl6180x_write8(addr, 0x010A, 0x20);  // Reduce ALS integration time
            vl6180x_write8(addr, 0x003F, 0x40);  // Reduce ALS gain
            sleep_ms(50);  // Give sensor time to adjust
        }
    }
    
    return range;
}

// Helper function to set both motors
void setMotors(bool run, int speed) {
    if (run) {
        // Both motors forward (matching Arduino implementation)
        pwm_set_chan_level(PWM_L_IN1_SLICE, PWM_CHAN_A, speed);   // Left motor: IN1 = PWM, IN2 = 0
        pwm_set_chan_level(PWM_L_IN2_SLICE, PWM_CHAN_B, 0);
        pwm_set_chan_level(PWM_R_IN1_SLICE, PWM_CHAN_A, 0);       // Right motor: IN1 = 0, IN2 = PWM
        pwm_set_chan_level(PWM_R_IN2_SLICE, PWM_CHAN_B, speed);
    } else {
        pwm_set_chan_level(PWM_L_IN1_SLICE, PWM_CHAN_A, 0);
        pwm_set_chan_level(PWM_L_IN2_SLICE, PWM_CHAN_B, 0);
        pwm_set_chan_level(PWM_R_IN1_SLICE, PWM_CHAN_A, 0);
        pwm_set_chan_level(PWM_R_IN2_SLICE, PWM_CHAN_B, 0);
    }
}

void turnLeft() {
    // Pivot left: left motor reverse, right motor forward
    pwm_set_chan_level(PWM_L_IN1_SLICE, PWM_CHAN_A, 0);
    pwm_set_chan_level(PWM_L_IN2_SLICE, PWM_CHAN_B, DUTY_50);
    pwm_set_chan_level(PWM_R_IN1_SLICE, PWM_CHAN_A, 0);
    pwm_set_chan_level(PWM_R_IN2_SLICE, PWM_CHAN_B, DUTY_50);
    sleep_ms(400); // Adjust for your robot's turn speed
    setMotors(false, 0);
}

void turnRight() {
    // Pivot right: left motor forward, right motor reverse
    pwm_set_chan_level(PWM_L_IN1_SLICE, PWM_CHAN_A, DUTY_50);
    pwm_set_chan_level(PWM_L_IN2_SLICE, PWM_CHAN_B, 0);
    pwm_set_chan_level(PWM_R_IN1_SLICE, PWM_CHAN_A, DUTY_50);
    pwm_set_chan_level(PWM_R_IN2_SLICE, PWM_CHAN_B, 0);
    sleep_ms(400); // Adjust for your robot's turn speed
    setMotors(false, 0);
}

// Encoder interrupt handler
void encoder_callback(uint gpio, uint32_t events) {
    uint8_t leftState = (gpio_get(ENCODER_L_A) << 1) | gpio_get(ENCODER_L_B);
    uint8_t rightState = (gpio_get(ENCODER_R_A) << 1) | gpio_get(ENCODER_R_B);
    
    // Left encoder
    if (gpio == ENCODER_L_A || gpio == ENCODER_L_B) {
        if ((lastLeftState == 0b00 && leftState == 0b01) ||
            (lastLeftState == 0b01 && leftState == 0b11) ||
            (lastLeftState == 0b11 && leftState == 0b10) ||
            (lastLeftState == 0b10 && leftState == 0b00)) {
            leftEncoderCount++;
        } else {
            leftEncoderCount--;
        }
        lastLeftState = leftState;
    }
    
    // Right encoder
    if (gpio == ENCODER_R_A || gpio == ENCODER_R_B) {
        if ((lastRightState == 0b00 && rightState == 0b01) ||
            (lastRightState == 0b01 && rightState == 0b11) ||
            (lastRightState == 0b11 && rightState == 0b10) ||
            (lastRightState == 0b10 && rightState == 0b00)) {
            rightEncoderCount++;
        } else {
            rightEncoderCount--;
        }
        lastRightState = rightState;
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("MicroMouse Robot Starting...\n");

    // Initialize ToF sensors
    init_tof();
    printf("ToF sensors initialized\n");

    // Initialize motor pins as GPIO first
    gpio_init(MOTOR_L_IN1);
    gpio_init(MOTOR_L_IN2);
    gpio_init(MOTOR_R_IN1);
    gpio_init(MOTOR_R_IN2);
    gpio_set_dir(MOTOR_L_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_L_IN2, GPIO_OUT);
    gpio_set_dir(MOTOR_R_IN1, GPIO_OUT);
    gpio_set_dir(MOTOR_R_IN2, GPIO_OUT);

    // Then set them up for PWM
    gpio_set_function(MOTOR_L_IN1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_L_IN2, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_IN1, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_R_IN2, GPIO_FUNC_PWM);

    // Configure PWM
    pwm_set_wrap(PWM_L_IN1_SLICE, PWM_WRAP);
    pwm_set_wrap(PWM_L_IN2_SLICE, PWM_WRAP);
    pwm_set_wrap(PWM_R_IN1_SLICE, PWM_WRAP);
    pwm_set_wrap(PWM_R_IN2_SLICE, PWM_WRAP);
    pwm_set_clkdiv(PWM_L_IN1_SLICE, 125.0f / PWM_FREQ);
    pwm_set_clkdiv(PWM_L_IN2_SLICE, 125.0f / PWM_FREQ);
    pwm_set_clkdiv(PWM_R_IN1_SLICE, 125.0f / PWM_FREQ);
    pwm_set_clkdiv(PWM_R_IN2_SLICE, 125.0f / PWM_FREQ);
    pwm_set_enabled(PWM_L_IN1_SLICE, true);
    pwm_set_enabled(PWM_L_IN2_SLICE, true);
    pwm_set_enabled(PWM_R_IN1_SLICE, true);
    pwm_set_enabled(PWM_R_IN2_SLICE, true);

    // Initialize encoder pins
    gpio_set_irq_enabled_with_callback(ENCODER_L_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled(ENCODER_L_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_R_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_R_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // Initialize button
    gpio_set_function(BTN1, GPIO_FUNC_SIO);
    gpio_set_dir(BTN1, GPIO_IN);
    gpio_pull_up(BTN1);

    // Initialize onboard LED
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    printf("Setup complete. Press button to start motor test.\n");

    bool lastButtonState = true;
    bool motorTestRunning = false;
    absolute_time_t testStartTime;
    int testStep = 0;

    while (true) {
        // Read button state (LOW when pressed due to pull-up)
        bool currentButtonState = gpio_get(BTN1);
        
        // Button press detection (falling edge)
        if (!currentButtonState && lastButtonState) {
            if (!motorTestRunning) {
                printf("\nStarting Motor Test Sequence...\n");
                motorTestRunning = true;
                testStartTime = get_absolute_time();
                testStep = 0;
                setMotors(false, 0);
            }
        }
        lastButtonState = currentButtonState;

        // Motor test sequence
        if (motorTestRunning) {
            absolute_time_t currentTime = get_absolute_time();
            int64_t elapsedTime = absolute_time_diff_us(testStartTime, currentTime) / 1000; // Convert to ms
            
            switch (testStep) {
                case 0: // Forward
                    if (elapsedTime < 2000) {
                        setMotors(true, DUTY_50);
                        printf("Forward\n");
                    } else {
                        testStep++;
                        testStartTime = currentTime;
                    }
                    break;
                    
                case 1: // Stop
                    if (elapsedTime < 1000) {
                        setMotors(false, 0);
                        printf("Stop\n");
                    } else {
                        testStep++;
                        testStartTime = currentTime;
                    }
                    break;
                    
                case 2: // Turn Left
                    if (elapsedTime < 2000) {
                        turnLeft();
                        printf("Turn Left\n");
                    } else {
                        testStep++;
                        testStartTime = currentTime;
                    }
                    break;
                    
                case 3: // Stop
                    if (elapsedTime < 1000) {
                        setMotors(false, 0);
                        printf("Stop\n");
                    } else {
                        testStep++;
                        testStartTime = currentTime;
                    }
                    break;
                    
                case 4: // Turn Right
                    if (elapsedTime < 2000) {
                        turnRight();
                        printf("Turn Right\n");
                    } else {
                        testStep++;
                        testStartTime = currentTime;
                    }
                    break;
                    
                case 5: // Final Stop
                    setMotors(false, 0);
                    printf("Motor Test Complete!\n");
                    motorTestRunning = false;
                    break;
            }
        }

        // Read all ToF sensors with error handling
        uint8_t left_range = vl6180x_read_range(TOF1_ADDR);
        if (left_range == 255) {
            printf("Left sensor read failed, retrying...\n");
            reset_i2c_bus();
            left_range = vl6180x_read_range(TOF1_ADDR);
        }
        
        uint8_t front_range = vl6180x_read_range(TOF2_ADDR);
        if (front_range == 255) {
            printf("Front sensor read failed, retrying...\n");
            reset_i2c_bus();
            front_range = vl6180x_read_range(TOF2_ADDR);
        }
        
        uint8_t right_range = vl6180x_read_range(TOF3_ADDR);
        if (right_range == 255) {
            printf("Right sensor read failed, retrying...\n");
            reset_i2c_bus();
            right_range = vl6180x_read_range(TOF3_ADDR);
        }
        
        printf("ToF Readings - Left: %d mm, Front: %d mm, Right: %d mm\n", 
               left_range, front_range, right_range);

        // Print encoder values and heartbeat
        printf("L Enc: %ld, R Enc: %ld\n", leftEncoderCount, rightEncoderCount);
        gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
        sleep_ms(1000);
    }

    return 0;
}
