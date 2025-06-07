#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h> // For abs()
#include <math.h>   // For fabs()
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "include/tof.h"
#include "include/motor.h"
#include "include/encoder.h"
#include "include/controller.h"
#include "include/imu.h"
#include "pico/time.h"

// --- DEBUG CONTROL ---
// Uncomment the line below to enable UART communication with the ESP32 for debugging
// #define ENABLE_UART_DEBUG

#ifdef ENABLE_UART_DEBUG
#include "hardware/uart.h"
#endif

// Helper function to constrain a value within a range
static inline int16_t constrain(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Global to track which side the robot got stuck on
static bool g_stuck_on_left_wall;

// Button pins
#define BUTTON_1_PIN 6
#define BUTTON_2_PIN 7
#define BUTTON_3_PIN 21
#define BUTTON_4_PIN 22

// PWM settings
#define PWM_FREQ 20000  // 20 kHz
#define PWM_WRAP 255    // 8-bit resolution

#ifdef ENABLE_UART_DEBUG
// UART settings for ESP32 communication
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#endif

// --- WALL FOLLOWER CONSTANTS ---
#define JUNCTION_FORWARD_DURATION_MS 300 // Time to move forward into a junction before turning
#define POST_TURN_ADJUST_DURATION_MS 250 // Time to move forward after a turn to secure the corridor
#define WALL_FOLLOW_LEFT_THRESHOLD 120   // mm, distance to consider left wall "gone"
#define WALL_FOLLOW_FRONT_THRESHOLD 70   // mm, distance to consider front wall "present"

// Robot states
typedef enum {
    STATE_IDLE,
    STATE_FORWARD,
    STATE_TURNING,
    STATE_TESTING_TOF,
    
    // Wall follower states
    STATE_WALL_FOLLOWING,
    STATE_JUNCTION_FORWARD,
    STATE_POST_TURN_ADJUST
} RobotState;

// Global variables
RobotState currentState = STATE_IDLE;
absolute_time_t stateStartTime;
bool operationActive = false;
bool button2_prev_state = true;  // Track button 2 state for toggle detection (true = not pressed)
PIDController front_pid;  // PID controller for front distance
static bool turn_direction_is_left; // True for left turn, false for right

// Function declarations
void handleForwardMovement(void);
void handleTurning(void);
void handleKillSwitch(void);
void handleToFTest(void);
void send_debug_to_esp32(const char *format, ...);
void handleWallFollowing(void);
void handleJunctionForward(void);
void handlePostTurnAdjust(void);

// Function to send debug data to ESP32
void send_debug_to_esp32(const char *format, ...) {
#ifdef ENABLE_UART_DEBUG
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uart_puts(UART_ID, buffer);
    uart_puts(UART_ID, "\n"); // Add newline for parsing on ESP32
#endif
}

int main() {
    // Initialize stdio
    stdio_init_all();
    printf("MicroMouse Robot Starting...\n");

#ifdef ENABLE_UART_DEBUG
    // Initialize UART for ESP32 communication
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    printf("UART for ESP32 initialized\n");
#endif

    // Initialize components
    init_tof();
    init_motors();
    init_encoders();
    init_pid_controller(&front_pid);
    
    // Initialize IMU
    if (init_imu()) {
        printf("IMU initialized successfully\n");
        send_debug_to_esp32("IMU: Init Success");
    } else {
        printf("Warning: IMU initialization failed - continuing without IMU\n");
        send_debug_to_esp32("IMU: Init FAILED");
    }

    // Configure button pins
    gpio_set_dir(BUTTON_1_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_2_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_3_PIN, GPIO_IN);
    gpio_set_dir(BUTTON_4_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_1_PIN);
    gpio_pull_up(BUTTON_2_PIN);
    gpio_pull_up(BUTTON_3_PIN);
    gpio_pull_up(BUTTON_4_PIN);

    printf("Initialization complete. Ready for commands.\n");

    while (1) {
        // Check kill switch first (highest priority)
        if (!gpio_get(BUTTON_3_PIN)) {
            handleKillSwitch();
            continue;  // Skip all other operations if kill switch is active
        }

        // Only process other buttons if no operation is active
        if (!operationActive) {
            // Check button 1 (forward movement)
            if (!gpio_get(BUTTON_1_PIN)) {
                currentState = STATE_FORWARD;
                stateStartTime = get_absolute_time();
                operationActive = true;
                printf("Button 1 pressed: Starting forward movement\n");
            }

            // Check button 2 (wall follower toggle)
            bool button2_current_state = gpio_get(BUTTON_2_PIN);
            if (!button2_current_state && button2_prev_state) {
                // Button 2 was just pressed (falling edge)
                if (currentState == STATE_WALL_FOLLOWING) {
                    // Currently following - stop it
                    setMotors(false, 0);
                    currentState = STATE_IDLE;
                    operationActive = false;
                    printf("Button 2 pressed: Stopping Wall Following\n");
                    send_debug_to_esp32("BTN2: STOP Wall Follow");
                } else {
                    // Start wall following
                    reset_heading_reference(); // Capture whatever direction robot is facing
                    currentState = STATE_WALL_FOLLOWING;
                    operationActive = true;
                    printf("Button 2 pressed: Starting Left-Hand Wall Following\n");
                    send_debug_to_esp32("BTN2: START Wall Follow");
                    
                    // Re-initialize the PID controller for a fresh start
                    init_pid_controller(&front_pid);
                }
            }
            button2_prev_state = button2_current_state;

            // Check button 4 (ToF test)
            if (!gpio_get(BUTTON_4_PIN)) {
                currentState = STATE_TESTING_TOF;
                operationActive = true;
                printf("Button 4 pressed: Starting ToF sensor test\n");
            }
        }

        // Handle current state
        switch (currentState) {
            case STATE_IDLE:
                setMotors(false, 0);  // Ensure motors are stopped
                operationActive = false; // Ensure we can start a new operation
                break;
            case STATE_FORWARD:
                handleForwardMovement();
                break;
            case STATE_WALL_FOLLOWING:
                handleWallFollowing();
                break;
            case STATE_JUNCTION_FORWARD:
                handleJunctionForward();
                break;
            case STATE_POST_TURN_ADJUST:
                handlePostTurnAdjust();
                break;
            case STATE_TURNING:
                handleTurning();
                break;
            case STATE_TESTING_TOF:
                handleToFTest();
                break;
        }

        // The main sensor reading and debug output can happen once per loop
        uint8_t front_distance = vl6180x_read_range(TOF3_ADDR);
        uint8_t left_distance = vl6180x_read_range(TOF2_ADDR);
        uint8_t right_distance = vl6180x_read_range(TOF1_ADDR);

        // Reduce ToF output frequency - only send every 10th loop (200ms)
        static int tof_counter = 0;
        tof_counter++;
        if (tof_counter >= 10) {
            tof_counter = 0;
            send_debug_to_esp32("ToF F:%d L:%d R:%d", front_distance, left_distance, right_distance);
        printf("F:%d L:%d R:%d\n", front_distance, left_distance, right_distance);
        }
        
        // Send IMU data with static layout - each line has consistent content
        if (imu_is_connected()) {
            IMUData imu_data;
            if (read_imu_data(&imu_data)) {
                // Static line 2: Always show heading
                send_debug_to_esp32("Head: %.1f deg", imu_data.heading);
                
                // Static line 3: Always show gyro Z (turning rate)
                send_debug_to_esp32("Turn: %.1f deg/s", imu_data.gyro_z);
                
                // Static line 4: Always show accelerometer X,Y
                send_debug_to_esp32("Acc: X%.2f Y%.2f", imu_data.accel_x, imu_data.accel_y);
                
                // Debug for Button 2 operation - show if heading is actually changing
                static int debug_counter = 0;
                debug_counter++;
                if (debug_counter >= 25 && currentState == STATE_WALL_FOLLOWING) { // Every 500ms during Button 2
                    printf("BTN2 DEBUG: heading=%.1f, gyro_z=%.1f, state=%s\n", 
                           imu_data.heading, imu_data.gyro_z, "WALL-FOLLOW");
                    debug_counter = 0;
                }
            }
        } else {
            // If IMU not connected, show static system status
            send_debug_to_esp32("Head: IMU OFFLINE");
            send_debug_to_esp32("Turn: IMU OFFLINE");
            send_debug_to_esp32("State: %s", 
                currentState == STATE_IDLE ? "IDLE" :
                currentState == STATE_FORWARD ? "FORWARD" :
                currentState == STATE_WALL_FOLLOWING ? "WALL-FOLLOW" :
                currentState == STATE_JUNCTION_FORWARD ? "JUNCTION-FWD" :
                currentState == STATE_POST_TURN_ADJUST ? "POST-TURN-ADJ" :
                currentState == STATE_TURNING ? "TURNING" : "TOF-TEST");
        }

        sleep_ms(20);  // Loop delay
    }
}

void handleForwardMovement(void) {
    absolute_time_t currentTime = get_absolute_time();
    int64_t elapsed = absolute_time_diff_us(stateStartTime, currentTime) / 1000;  // Convert to ms

    if (elapsed >= 5000) {  // 5 seconds elapsed
        setMotors(false, 0);
        currentState = STATE_IDLE;
        operationActive = false;
        printf("Forward movement complete\n");
        return;
    }

    setMotors(true, DUTY_50);
}

void handleTurning(void) {
    // This function now uses a PID controller for accurate turns.
    float current_heading = get_current_heading();
    float target_heading = turn_direction_is_left ? -90.0f : 90.0f; // Target is exactly 90 degrees now

    // --- PID Calculation ---
    float error = target_heading - current_heading;
    
    // Proportional term
    float p_term = front_pid.kP_turn * error;
    
    // Derivative term
    float derivative = error - front_pid.last_turn_error;
    float d_term = front_pid.kD_turn * derivative;
    
    // Integral term (with anti-windup)
    front_pid.integral_turn += error;
    // Anti-windup: cap the integral term to prevent it from growing too large
    if (front_pid.integral_turn > 100) front_pid.integral_turn = 100;
    if (front_pid.integral_turn < -100) front_pid.integral_turn = -100;
    float i_term = front_pid.kI_turn * front_pid.integral_turn;
    
    // Update last error for next derivative calculation
    front_pid.last_turn_error = error;

    // --- Check for Turn Completion ---
    // The turn is complete if the error is small AND the rate of change is small.
    if (fabs(error) < TURN_PID_THRESHOLD && fabs(derivative) < 0.1f) {
        printf("Turn complete. Target: %.1f, Actual: %.1f, Error: %.1f\n", target_heading, current_heading, error);
        setIndividualMotors(0, 0); // Stop motors
        sleep_ms(100); // Shorter pause

        reset_heading_reference(); // Set the new forward direction as 0 degrees
        front_pid.integral_turn = 0; // Reset integral for the next turn
        
        // Transition to post-turn adjustment instead of directly to wall following
        currentState = STATE_POST_TURN_ADJUST;
        stateStartTime = get_absolute_time(); // Start timer for the adjustment phase

        printf("Resuming forward (post-turn adjust).\n");
        send_debug_to_esp32("Turn Done. Adjusting...");
        return;
    }

    // --- Calculate Motor Speed from PID ---
    int16_t turn_speed = (int16_t)(p_term + i_term + d_term);

    // Constrain the turn speed to be within motor limits
    turn_speed = constrain(turn_speed, -TURN_SPEED, TURN_SPEED);
    
    // Apply motor speeds for pivot turn
    // If turn_speed is positive, we want to turn right (L forward, R backward)
    // If turn_speed is negative, we want to turn left (R forward, L backward)
    setIndividualMotors(turn_speed, -turn_speed);

    // Log progress
    static uint32_t last_log_time = 0;
    if (to_ms_since_boot(get_absolute_time()) - last_log_time > 100) {
        printf("Turning... Target: %.1f, Curr: %.1f, Err: %.1f, Speed: %d\n", target_heading, current_heading, error, turn_speed);
        send_debug_to_esp32("TURN: Err:%.1f Speed:%d", error, turn_speed);
        last_log_time = to_ms_since_boot(get_absolute_time());
    }
}

void handleKillSwitch(void) {
    printf("Kill switch engaged. Stopping all motors.\n");
    if (operationActive) {
        printf("KILL SWITCH ACTIVATED! Stopping current operation.\n");
        operationActive = false;
        currentState = STATE_IDLE;
        setMotors(false, 0);  // Stop all motors immediately
    }
}

void handleToFTest(void) {
    static uint32_t lastReadingTime = 0;
    static uint8_t consecutiveErrors = 0;
    static uint8_t lastReadings[3] = {0, 0, 0};
    static uint8_t frontSensorRetries = 0;
    static bool motor_test_mode = false;
    static uint32_t motor_test_start = 0;
    
    // MOTOR TESTING: Hold button 4 for 3+ seconds to enter motor test mode
    if (!motor_test_mode) {
        if (to_ms_since_boot(get_absolute_time()) - motor_test_start == 0) {
            motor_test_start = to_ms_since_boot(get_absolute_time());
        }
        if (to_ms_since_boot(get_absolute_time()) - motor_test_start > 3000) {
            motor_test_mode = true;
            motor_test_start = to_ms_since_boot(get_absolute_time());
            printf("\n=== MOTOR TEST MODE ACTIVATED ===\n");
            printf("Testing individual motors to verify left/right assignment\n");
        }
    }
    
    if (motor_test_mode) {
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - motor_test_start;
        if (elapsed < 2000) {
            printf("Testing LEFT motor only...\n");
            setIndividualMotors(100, 0);  // Increased power from 80 to 100
        } else if (elapsed < 4000) {
            printf("Testing RIGHT motor only...\n");
            setIndividualMotors(0, 100);  // Increased power from 80 to 100
        } else if (elapsed < 6000) {
            printf("Testing BOTH motors forward...\n");
            setIndividualMotors(100, 100);  // Increased power from 80 to 100
        } else if (elapsed < 8000) {
            printf("IMU HEADING DEBUG MODE - Keep robot still...\n");
            setIndividualMotors(0, 0);  // Stop
            if (elapsed > 6000 && elapsed < 6100) {  // Run debug once at ~6 seconds
                debug_imu_heading();
            }
        } else if (elapsed < 10000) {
            // SIMPLE 90-DEGREE CALIBRATION - Press 4, turn robot, see angle
            static bool calibration_started = false;
            static uint32_t last_display_time = 0;
            
            if (!calibration_started) {
                // Start calibration immediately when entering this mode
                reset_heading_reference();  // Set current position as 0 degrees
                calibration_started = true;
                printf("\n=== 90-DEGREE CALIBRATION STARTED ===\n");
                printf("Heading reset to 0.0 degrees\n");
                printf("Turn robot 90 degrees clockwise and observe angle\n");
                send_debug_to_esp32("CAL: 0.0 deg - Turn 90");
                last_display_time = to_ms_since_boot(get_absolute_time());
            }
            
            // Show current angle every 500ms (reduced frequency)
            float current_heading = get_current_heading();
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            
            if (current_time - last_display_time > 500) {
                printf("Angle: %.1f deg\n", current_heading);
                send_debug_to_esp32("Angle: %.1f deg", current_heading);
                last_display_time = current_time;
            }
        } else {
            printf("All tests complete. Release button to exit.\n");
            setIndividualMotors(0, 0);  // Stop
            if (gpio_get(BUTTON_4_PIN)) {
                motor_test_mode = false;
                currentState = STATE_IDLE;
                operationActive = false;
                return;
            }
        }
        return;  // Skip normal ToF testing during motor test
    }
    
    // Read all three sensors
    uint8_t rightDistance = vl6180x_read_range(TOF1_ADDR);
    uint8_t leftDistance = vl6180x_read_range(TOF2_ADDR);
    
    // Special handling for front sensor
    uint8_t frontDistance = 255;  // Default to error value
    int ret = vl6180x_read_range(TOF3_ADDR);
    if (ret >= 0) {
        frontDistance = ret;
        frontSensorRetries = 0;  // Reset retry counter on success
    } else {
        frontSensorRetries++;
        if (frontSensorRetries >= 3) {
            printf("\nFront sensor (0x2c) diagnostic:\n");
            printf("1. Checking sensor presence...\n");
            if (vl6180x_is_present(TOF3_ADDR)) {
                printf("   - Sensor is present\n");
                printf("2. Attempting sensor reset...\n");
                reset_i2c_bus();
                sleep_ms(100);
                if (vl6180x_is_present(TOF3_ADDR)) {
                    printf("   - Sensor still present after reset\n");
                    printf("3. Checking sensor configuration...\n");
                    uint8_t model_id = vl6180x_read_model_id(TOF3_ADDR);
                    printf("   - Model ID: 0x%02x\n", model_id);
                } else {
                    printf("   - Sensor not responding after reset\n");
                }
            } else {
                printf("   - Sensor not detected\n");
            }
            frontSensorRetries = 0;
        }
    }
    
    // Check if readings have changed
    bool readingsChanged = (rightDistance != lastReadings[0] ||
                          leftDistance != lastReadings[1] ||
                          frontDistance != lastReadings[2]);
    
    // Update last readings
    lastReadings[0] = rightDistance;
    lastReadings[1] = leftDistance;
    lastReadings[2] = frontDistance;
    
    // Check for frozen readings
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());
    if (!readingsChanged && (currentTime - lastReadingTime > 1000)) {
        consecutiveErrors++;
        printf("Warning: Readings appear to be frozen. Attempting recovery...\n");
        
        if (consecutiveErrors >= 3) {
            printf("Too many consecutive errors. Resetting I2C bus...\n");
            reset_i2c_bus();
            sleep_ms(100);  // Give bus time to stabilize
            consecutiveErrors = 0;
        }
    } else {
        consecutiveErrors = 0;
    }
    
    lastReadingTime = currentTime;
    
    // Print readings with status indicators
    printf("\nToF Sensor Readings:\n");
    printf("Right sensor (0x%02x): %d mm [%s]\n", 
           TOF1_ADDR, rightDistance, 
           rightDistance < 255 ? "OK" : "ERROR");
    printf("Left sensor (0x%02x): %d mm [%s]\n", 
           TOF2_ADDR, leftDistance, 
           leftDistance < 255 ? "OK" : "ERROR");
    printf("Front sensor (0x%02x): %d mm [%s]\n", 
           TOF3_ADDR, frontDistance, 
           frontDistance < 255 ? "OK" : "ERROR");
    printf("-------------------\n");
    
    // Check if button 4 is still pressed
    if (gpio_get(BUTTON_4_PIN)) {
        currentState = STATE_IDLE;
        operationActive = false;
        printf("ToF test complete\n");
    }
    
    // Add a small delay between readings
    sleep_ms(100);
}

void handleWallFollowing(void) {
    // Implements the Left-Hand Wall Follower algorithm.
    uint8_t left_dist = vl6180x_read_range(TOF2_ADDR);
    uint8_t front_dist = vl6180x_read_range(TOF3_ADDR);

    // Use default values for invalid readings to make logic cleaner
    if (left_dist == 255) left_dist = 50; // Assume wall is present if sensor fails
    if (front_dist == 255) front_dist = 200; // Assume wall is not present if sensor fails

    bool left_opening = (left_dist > WALL_FOLLOW_LEFT_THRESHOLD);
    bool front_wall_present = (front_dist < WALL_FOLLOW_FRONT_THRESHOLD);

    // PRIORITY 1: Left opening detected.
    if (left_opening) {
        printf("WALL_FOLLOW: Left opening. Preparing for left turn.\n");
        send_debug_to_esp32("L-TURN: Prep");
        setIndividualMotors(0, 0); // Stop briefly
        stateStartTime = get_absolute_time();
        currentState = STATE_JUNCTION_FORWARD;
        return;
    }
    
    // PRIORITY 2: Front wall detected.
    if (front_wall_present) {
        printf("WALL_FOLLOW: Front wall. Turning right.\n");
        send_debug_to_esp32("R-TURN: Now");
        setIndividualMotors(0, 0); // Stop
        turn_direction_is_left = false; // Set for a right turn
        reset_heading_reference(); // Turn is relative to current direction
        currentState = STATE_TURNING;
        return;
    }

    // PRIORITY 3 (DEFAULT): Follow the left wall.
    float heading_error = get_current_heading();
    
    // Correction to stay parallel to the wall using IMU
    int16_t heading_correction = -(int16_t)(front_pid.kP_heading * heading_error);
    
    // Correction to stay at the target distance from the left wall
    int32_t left_error = left_dist - TARGET_WALL_DISTANCE_MM; // TARGET_WALL_DISTANCE_MM is from controller.h
    int16_t wall_correction = (int16_t)(front_pid.kP_wall * left_error);
    
    int16_t total_correction = wall_correction + heading_correction;
    total_correction = constrain(total_correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
    
    int16_t left_speed = WALL_FOLLOW_SPEED - total_correction;
    int16_t right_speed = WALL_FOLLOW_SPEED + total_correction;

    setIndividualMotors(left_speed, right_speed);

    static uint32_t last_log = 0;
    if (to_ms_since_boot(get_absolute_time()) - last_log > 200) {
        printf("WALL_FOLLOW: Following. L_dist:%d, H_err:%.1f\n", left_dist, heading_error);
        send_debug_to_esp32("FOLLOW: L:%d H:%.1f", left_dist, heading_error);
        last_log = to_ms_since_boot(get_absolute_time());
    }
}

void handleJunctionForward(void) {
    int64_t elapsed_ms = absolute_time_diff_us(stateStartTime, get_absolute_time()) / 1000;

    if (elapsed_ms < JUNCTION_FORWARD_DURATION_MS) {
        // Move forward to position the robot in the center of the junction
        setIndividualMotors(WALL_FOLLOW_SPEED, WALL_FOLLOW_SPEED);
    } else {
        // Time elapsed, now execute the left turn
        printf("WALL_FOLLOW: Centered in junction. Initiating left turn.\n");
        setIndividualMotors(0, 0); // Stop
        turn_direction_is_left = true; // Set for a left turn
        reset_heading_reference(); // Turn is relative to current direction
        currentState = STATE_TURNING;
    }
}

void handlePostTurnAdjust(void) {
    int64_t elapsed_ms = absolute_time_diff_us(stateStartTime, get_absolute_time()) / 1000;

    if (elapsed_ms < POST_TURN_ADJUST_DURATION_MS) {
        // Move straight forward to exit the junction and enter the new corridor.
        // We use the same PID logic as the wall follower to go straight.
        uint8_t left_dist = vl6180x_read_range(TOF2_ADDR);
        if (left_dist == 255) left_dist = 50;
        
        float heading_error = get_current_heading();
        
        int16_t heading_correction = -(int16_t)(front_pid.kP_heading * heading_error);
        int32_t left_error = left_dist - TARGET_WALL_DISTANCE_MM;
        int16_t wall_correction = (int16_t)(front_pid.kP_wall * left_error);
        
        int16_t total_correction = wall_correction + heading_correction;
        total_correction = constrain(total_correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
        
        int16_t left_speed = WALL_FOLLOW_SPEED - total_correction;
        int16_t right_speed = WALL_FOLLOW_SPEED + total_correction;

        setIndividualMotors(left_speed, right_speed);

        static uint32_t last_log = 0;
        if (to_ms_since_boot(get_absolute_time()) - last_log > 100) {
            printf("POST-TURN ADJUST: Moving forward.\n");
            send_debug_to_esp32("ADJUST: Fwd...");
            last_log = to_ms_since_boot(get_absolute_time());
        }

    } else {
        // Adjustment complete, now begin normal wall following.
        printf("Post-turn adjustment complete. Resuming wall following.\n");
        currentState = STATE_WALL_FOLLOWING;
    }
}
