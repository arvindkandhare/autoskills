#include "high_scoring_control.hpp"

// External references to the motor and sensor defined in main.cpp
extern pros::Motor High_scoring;
extern pros::Rotation rotational_sensor;

/**
 * Gets the current angle of the High Scoring mechanism
 * 
 * @return The current angle in degrees
 */
double getHighScoringAngle() {
    return rotational_sensor.get_position();
}

/**
 * Moves the High Scoring mechanism to a specific angle using PID control
 * 
 * @param target_angle The target angle in degrees
 * @param precise If true, uses more precise PID constants (for capture position)
 * @param wait_for_completion If true, the function will block until the position is reached
 * @return True if the position was reached successfully, false otherwise
 */
bool moveHighScoringToAngle(double target_angle, bool precise, bool wait_for_completion) {
    // Select PID constants based on precision requirement
    double kP = precise ? HighScoringPID::KP_CAPTURE : HighScoringPID::KP_GENERAL;
    double kI = precise ? HighScoringPID::KI_CAPTURE : HighScoringPID::KI_GENERAL;
    double kD = precise ? HighScoringPID::KD_CAPTURE : HighScoringPID::KD_GENERAL;
    double threshold = precise ? HighScoringPID::POSITION_THRESHOLD_CAPTURE : HighScoringPID::POSITION_THRESHOLD_GENERAL;
    
    // PID control variables
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;
    double motor_power = 0;
    
    // Timing variables
    uint32_t start_time = pros::millis();
    uint32_t current_time = start_time;
    
    // If not waiting for completion, just start the motor moving in the right direction
    if (!wait_for_completion) {
        error = target_angle - getHighScoringAngle();
        motor_power = error * kP;
        
        // Limit motor power to prevent excessive speed
        if (motor_power > 100) motor_power = 100;
        if (motor_power < -100) motor_power = -100;
        
        High_scoring.move_velocity(motor_power);
        return true;
    }
    
    // Main PID control loop
    while (true) {
        current_time = pros::millis();
        
        // Check for timeout
        if (current_time - start_time > HighScoringPID::TIMEOUT_MS) {
            High_scoring.brake();
            return false;  // Timeout occurred
        }
        
        // Calculate error
        error = target_angle - getHighScoringAngle();
        
        // Check if we've reached the target
        if (std::abs(error) < threshold) {
            High_scoring.brake();
            return true;  // Target reached
        }
        
        // Calculate PID components
        integral += error;
        derivative = error - prev_error;
        
        // Calculate motor power using PID formula
        motor_power = kP * error + kI * integral + kD * derivative;
        
        // Limit motor power to prevent excessive speed
        if (motor_power > 100) motor_power = 100;
        if (motor_power < -100) motor_power = -100;
        
        // Apply power to the motor
        High_scoring.move_velocity(motor_power);
        
        // Save current error for next iteration
        prev_error = error;
        
        // Small delay to prevent hogging CPU
        pros::delay(10);
    }
}

/**
 * Moves the High Scoring mechanism to one of the predetermined positions
 * 
 * @param position The target position to move to
 * @param wait_for_completion If true, the function will block until the position is reached
 * @return True if the position was reached successfully, false otherwise
 */
bool moveHighScoringToPosition(HighScoringPosition position, bool wait_for_completion) {
    double target_angle = 0.0;
    bool precise = false;
    
    // Determine the target angle and precision based on the requested position
    switch (position) {
        case HighScoringPosition::GROUND:
            target_angle = HighScoringPositions::GROUND;
            precise = false;
            break;
        case HighScoringPosition::CAPTURE:
            target_angle = HighScoringPositions::CAPTURE;
            precise = true;  // Capture position needs to be exact
            break;
        case HighScoringPosition::WAIT:
            target_angle = HighScoringPositions::WAIT;
            precise = false;
            break;
        case HighScoringPosition::SCORE:
            target_angle = HighScoringPositions::SCORE;
            precise = false;
            break;
    }
    
    // Move to the target angle with the appropriate precision
    return moveHighScoringToAngle(target_angle, precise, wait_for_completion);
}
