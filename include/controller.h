#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

// --- Configuration Constants ---

// --- WALL FOLLOWING CONTROL ---
#define TARGET_WALL_DISTANCE_MM 40    // Target distance from walls, as requested (was 55)
#define SAFE_ZONE_DISTANCE_MM 45      // If both walls > this distance, no correction needed (safe ribbon)
#define MIN_WALL_DISTANCE_MM 40       // New minimum distance (was 30)
#define MAX_WALL_DISTANCE_MM 150      // Maximum expected wall distance
#define WALL_FOLLOW_SPEED 130        // Base speed for wall following
#define MIN_MOTOR_SPEED 35            // Minimum PWM speed to keep motors moving
#define EMERGENCY_STOP_DISTANCE_MM 40 // New minimum distance (was 30)

// --- WALL FOLLOWING PID GAINS ---
#define KP_WALL 0.35f     // Proportional gain for wall distance error (reduced from 0.5f)
#define KD_WALL 0.5f    // Derivative gain for wall distance error (reduced from 0.1f)
#define MAX_STEERING_CORRECTION 10    // Maximum steering correction (reduced from 30)

// --- HEADING CONTROL (IMU) ---
#define KP_HEADING 2.0f   // Proportional gain for heading error
#define KD_HEADING 0.1f   // Derivative gain for heading error  
#define MAX_HEADING_CORRECTION 15  // Maximum heading correction
#define TURN_SPEED 90             // PWM duty cycle for turning (Reduced from 100)

// --- TURNING PID GAINS (NEW) ---
#define KP_TURN 2.2f      // Proportional gain for turning. (Reduced from 3.0)
#define KD_TURN 2.5f      // Derivative gain to dampen overshoot. (Increased from 0.5)
#define KI_TURN 0.0f      // Integral gain to correct for steady-state error. (Disabled from 0.01)
#define TURN_PID_THRESHOLD 2.0f // Target accuracy in degrees to complete the turn.

// --- FRONT OBSTACLE AVOIDANCE ---
#define FRONT_OBSTACLE_DISTANCE_MM 15 // Stop/turn when front obstacle this close
#define FRONT_STOP_DISTANCE_MM 10     // Emergency stop distance (reduced from 50)

// Structure for the PID Controller
typedef struct {
    // Wall following PID
    float kP_wall;
    float kD_wall;
    int32_t last_wall_error;
    
    // Heading control PID
    float kP_heading;
    float kD_heading;
    float last_heading_error;
    
    // Turning PID
    float kP_turn;
    float kD_turn;
    float kI_turn;
    float integral_turn;
    float last_turn_error;
    
    // Front obstacle detection
    uint8_t front_obstacle_threshold;

} PIDController;

// Function Declarations
void init_pid_controller(PIDController *pid);
void apply_wall_following_with_imu_control(PIDController *pid, uint8_t left_distance, uint8_t right_distance, uint8_t front_distance, float heading_error);

#endif // CONTROLLER_H 