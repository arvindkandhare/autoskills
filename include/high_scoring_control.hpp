#ifndef HIGH_SCORING_CONTROL_HPP
#define HIGH_SCORING_CONTROL_HPP

#include "main.h"

// Define the four predetermined positions (angles) for the rotational sensor
// These values will need to be calibrated based on the actual robot setup
namespace HighScoringPositions {
    // Position values in degrees
    const double GROUND = 0.0;     // Position when the mechanism is at ground level
    const double CAPTURE = 15.0;   // Position for capturing game objects (needs to be exact)
    const double WAIT = 120.0;     // Waiting position
    const double SCORE = 380.0;    // Position for scoring
}

// Enum to make position selection more readable
enum class HighScoringPosition {
    GROUND,
    CAPTURE,
    WAIT,
    SCORE
};

// PID Controller constants for the High Scoring mechanism
// These values will need to be tuned based on the actual robot behavior
namespace HighScoringPID {
    // PID constants for general movement (less precise)
    const double KP_GENERAL = 0.5;
    const double KI_GENERAL = 0.0;
    const double KD_GENERAL = 0.1;
    
    // PID constants for capture position (more precise)
    const double KP_CAPTURE = 0.8;
    const double KI_CAPTURE = 0.01;
    const double KD_CAPTURE = 0.2;
    
    // Error thresholds
    const double POSITION_THRESHOLD_GENERAL = 5.0;  // Degrees
    const double POSITION_THRESHOLD_CAPTURE = 1.0;  // Degrees
    
    // Maximum time to wait for positioning (milliseconds)
    const int TIMEOUT_MS = 2000;
}

/**
 * Moves the High Scoring mechanism to one of the predetermined positions
 * 
 * @param position The target position to move to
 * @param wait_for_completion If true, the function will block until the position is reached
 * @return True if the position was reached successfully, false otherwise
 */
bool moveHighScoringToPosition(HighScoringPosition position, bool wait_for_completion = true);

/**
 * Moves the High Scoring mechanism to a specific angle
 * 
 * @param target_angle The target angle in degrees
 * @param precise If true, uses more precise PID constants (for capture position)
 * @param wait_for_completion If true, the function will block until the position is reached
 * @return True if the position was reached successfully, false otherwise
 */
bool moveHighScoringToAngle(double target_angle, bool precise = false, bool wait_for_completion = true);

/**
 * Gets the current angle of the High Scoring mechanism
 * 
 * @return The current angle in degrees
 */
double getHighScoringAngle();

#endif // HIGH_SCORING_CONTROL_HPP
