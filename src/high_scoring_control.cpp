#include "high_scoring_control.hpp"

// External references to the motor and sensor defined in main.cpp
extern pros::Motor High_scoring;
extern pros::Rotation rotational_sensor;

// Global variables for the background task
double g_target_angle = 0.0;
bool g_precise_mode = false;
bool g_task_running = false;
pros::Task* g_high_scoring_task = nullptr;

// Function prototypes
void highScoringTaskFn(void* param);
void stopHighScoringTask();

/**
 * Gets the current angle of the High Scoring mechanism
 * 
 * @return The current angle in degrees
 */
double getHighScoringAngle() {
    return rotational_sensor.get_position();
}

/**
 * Stops the high scoring task if it's running
 */
void stopHighScoringTask() {
    if (g_high_scoring_task != nullptr) {
        g_high_scoring_task->remove();
        delete g_high_scoring_task;
        g_high_scoring_task = nullptr;
        g_task_running = false;
        
        // Ensure the motor stops
        High_scoring.brake();
    }
}

/**
 * Task function that continuously adjusts the motor to reach the target angle
 * 
 * @param param Unused parameter (required by PROS task API)
 */
void highScoringTaskFn(void* param) {
    // Mark the task as running
    g_task_running = true;
    
    // PID control variables
    double error = 0;
    double prev_error = 0;
    double integral = 0;
    double derivative = 0;
    double motor_power = 0;
    
    // Local copy of the target to detect changes
    double current_target = g_target_angle;
    bool current_precise = g_precise_mode;
    
    while (true) {
        // Check if target has changed
        if (current_target != g_target_angle || current_precise != g_precise_mode) {
            // Update local copies
            current_target = g_target_angle;
            current_precise = g_precise_mode;
            
            // Reset PID variables when target changes
            integral = 0;
            prev_error = 0;
            
            // If not in precise mode, use direct move_absolute instead of PID
            if (!current_precise) {
                // Use move_absolute with a reasonable velocity (100)
                High_scoring.move_absolute(current_target, 100);
            }
        }
        
        // Only use PID control when in precise mode
        if (current_precise) {
            // Select PID constants for precise mode
            double kP = HighScoringPID::KP_CAPTURE;
            double kI = HighScoringPID::KI_CAPTURE;
            double kD = HighScoringPID::KD_CAPTURE;
            
            // Calculate error
            error = current_target - getHighScoringAngle();
            
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
        }
        
        // Small delay to prevent hogging CPU
        pros::delay(10);
    }
}

/**
 * Moves the High Scoring mechanism to a specific angle
 * 
 * @param target_angle The target angle in degrees
 * @param precise If true, uses PID control for precise positioning. If false, uses direct motor control.
 * @param wait_for_completion If true, the function will block until the position is reached
 * @return True if the position was reached successfully, false otherwise
 */
bool moveHighScoringToAngle(double target_angle, bool precise, bool wait_for_completion) {
    // Update the global target angle and precision mode
    g_target_angle = target_angle;
    g_precise_mode = precise;
    
    // If not using precise mode, we can directly command the motor to move to the absolute position
    if (!precise) {
        // Stop any existing task to avoid conflicts
        if (g_task_running) {
            stopHighScoringTask();
        }
        
        // Use move_absolute with a reasonable velocity (100)
        High_scoring.move_absolute(target_angle, 100);
    } else {
        // For precise mode, start the PID control task if it's not already running
        if (!g_task_running) {
            if (g_high_scoring_task != nullptr) {
                delete g_high_scoring_task;
            }
            g_high_scoring_task = new pros::Task(highScoringTaskFn, nullptr, "HighScoringTask");
        }
    }
    
    // If wait_for_completion is true, wait until the target is reached
    if (wait_for_completion) {
        double threshold = precise ? HighScoringPID::POSITION_THRESHOLD_CAPTURE : HighScoringPID::POSITION_THRESHOLD_GENERAL;
        uint32_t start_time = pros::millis();
        
        while (true) {
            // Check for timeout
            if (pros::millis() - start_time > HighScoringPID::TIMEOUT_MS) {
                return false;  // Timeout occurred
            }
            
            // Calculate error
            double error = target_angle - getHighScoringAngle();
            
            // Check if we've reached the target
            if (std::abs(error) < threshold) {
                return true;  // Target reached
            }
            
            // Small delay to prevent hogging CPU
            pros::delay(10);
        }
    }
    
    // If not waiting for completion, return immediately
    return true;
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
