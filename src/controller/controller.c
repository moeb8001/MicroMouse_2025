#include "include/controller.h"
#include "include/motor.h"
#include "include/encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Helper function to constrain a value within a range
static inline int16_t constrain(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Initializes the PID controller for wall following with IMU heading control.
 */
void init_pid_controller(PIDController *pid) {
    // Wall following PID gains
    pid->kP_wall = KP_WALL;
    pid->kD_wall = KD_WALL;
    pid->last_wall_error = 0;
    
    // Heading control PID gains
    pid->kP_heading = KP_HEADING;
    pid->kD_heading = KD_HEADING;
    pid->last_heading_error = 0.0f;
    
    // Turning PID gains
    pid->kP_turn = KP_TURN;
    pid->kD_turn = KD_TURN;
    pid->kI_turn = KI_TURN;
    pid->integral_turn = 0.0f;
    pid->last_turn_error = 0.0f;
    
    // Front obstacle threshold
    pid->front_obstacle_threshold = FRONT_OBSTACLE_DISTANCE_MM;
    
    printf("Wall-following + IMU heading PID controller initialized.\n");
}

/**
 * @brief Combined wall-following and IMU heading control.
 * @param pid Pointer to the PIDController structure.
 * @param left_distance Distance to left wall (mm).
 * @param right_distance Distance to right wall (mm).
 * @param front_distance Distance to front obstacle (mm).
 * @param heading_error Current heading error from IMU (degrees).
 */
void apply_wall_following_with_imu_control(PIDController *pid, uint8_t left_distance, uint8_t right_distance, uint8_t front_distance, float heading_error) {
    
    // Wall following logic
    int16_t base_speed = WALL_FOLLOW_SPEED;
    int16_t wall_correction = 0; // The correction value from wall sensors
    
    bool left_valid = (left_distance != 255 && left_distance < MAX_WALL_DISTANCE_MM);
    bool right_valid = (right_distance != 255 && right_distance < MAX_WALL_DISTANCE_MM);
    
    // --- Wall Correction PID ---
    // This logic now runs continuously to maintain the target distance.
    if (left_valid && right_valid) {
        // Both walls are visible: center the robot by minimizing the difference.
        int32_t wall_difference = left_distance - right_distance;
        int32_t wall_derivative = wall_difference - pid->last_wall_error;
        pid->last_wall_error = wall_difference;
        
        wall_correction = (int16_t)(pid->kP_wall * wall_difference + pid->kD_wall * wall_derivative);
        printf("WALLS_BOTH: L:%d R:%d diff:%ld corr:%d\n", left_distance, right_distance, wall_difference, wall_correction);
        
    } else if (left_valid) {
        // Only left wall is visible: maintain target distance.
        int32_t left_error = left_distance - TARGET_WALL_DISTANCE_MM;
        pid->last_wall_error = left_error; // Use single wall error for derivative next time
        wall_correction = (int16_t)(pid->kP_wall * left_error);
        printf("WALL_LEFT: L:%d target:%d err:%ld corr:%d\n", left_distance, TARGET_WALL_DISTANCE_MM, left_error, wall_correction);
        
    } else if (right_valid) {
        // Only right wall is visible: maintain target distance.
        int32_t right_error = TARGET_WALL_DISTANCE_MM - right_distance;
        pid->last_wall_error = -right_error; // Use single wall error for derivative next time
        wall_correction = (int16_t)(pid->kP_wall * right_error);
        printf("WALL_RIGHT: R:%d target:%d err:%ld corr:%d\n", right_distance, TARGET_WALL_DISTANCE_MM, right_error, wall_correction);
        
    } else {
        // No walls detected. Wall correction is zero.
        wall_correction = 0;
        pid->last_wall_error = 0;
        printf("NO_WALLS: Heading control only.\n");
    }

    // Check for recovery situations (getting too close to walls) and apply aggressive correction.
    if ((left_valid && left_distance < MIN_WALL_DISTANCE_MM) ||
        (right_valid && right_distance < MIN_WALL_DISTANCE_MM)) {
        printf("RECOVERY: Too close to walls. L:%d R:%d. Doubling correction.\n", left_distance, right_distance);
        wall_correction *= 2; // Double the correction when too close
    }

    // Constrain the wall correction to a reasonable max value.
    wall_correction = constrain(wall_correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);

    // --- IMU HEADING CONTROL (Always active) ---
    float heading_derivative = heading_error - pid->last_heading_error;
    pid->last_heading_error = heading_error;
    
    int16_t heading_correction = -(int16_t)(pid->kP_heading * heading_error + pid->kD_heading * heading_derivative);
    heading_correction = constrain(heading_correction, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
    
    // --- COMBINE CORRECTIONS ---
    int16_t total_correction = wall_correction + heading_correction;
    total_correction = constrain(total_correction, -MAX_STEERING_CORRECTION - 10, MAX_STEERING_CORRECTION + 10);
    
    printf("CORRECTIONS: wall:%d + heading:%d = total:%d (heading_err:%.1f)\n", 
           wall_correction, heading_correction, total_correction, heading_error);
    
    // --- Speed Adjustments ---
    if (front_distance < FRONT_OBSTACLE_DISTANCE_MM && front_distance != 255) {
        base_speed = base_speed * front_distance / FRONT_OBSTACLE_DISTANCE_MM;
        base_speed = constrain(base_speed, MIN_MOTOR_SPEED, WALL_FOLLOW_SPEED);
    }
    
    // Apply combined steering
    int16_t left_speed = base_speed - total_correction;
    int16_t right_speed = base_speed + total_correction;
    
    // Apply minimum speed thresholds
    if (left_speed > 0 && left_speed < MIN_MOTOR_SPEED) left_speed = MIN_MOTOR_SPEED;
    if (right_speed > 0 && right_speed < MIN_MOTOR_SPEED) right_speed = MIN_MOTOR_SPEED;
    
    // Constrain to maximum speeds
    left_speed = constrain(left_speed, 0, 150);
    right_speed = constrain(right_speed, 0, 150);
    
    printf("MOTORS: L:%d R:%d\n", left_speed, right_speed);
    
    // Send debug data to OLED
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 15) {
        debug_counter = 0;
        extern void send_debug_to_esp32(const char *format, ...);
        send_debug_to_esp32("W:%d H:%d=T:%d HE:%.1f", 
                           wall_correction, heading_correction, total_correction, heading_error);
    }
    
    setIndividualMotors(left_speed, right_speed);
} 