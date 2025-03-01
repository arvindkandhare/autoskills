# High Scoring Mechanism Control System

This project implements a control system for a High Scoring mechanism using a rotational sensor and PID control. The system allows the mechanism to move to four predetermined positions: ground, capture, wait, and score.

## Features

- **Four Predetermined Positions**:
  - **Ground**: The lowest position (0 degrees)
  - **Capture**: Position for capturing game objects (90 degrees) - requires precise positioning
  - **Wait**: Intermediate waiting position (180 degrees)
  - **Score**: Position for scoring (270 degrees)

- **PID Control**:
  - Precise PID control for the capture position
  - General PID control for other positions
  - Configurable PID constants for tuning

- **Controller Integration**:
  - Button mappings for easy control during operation
  - Manual override using the right joystick

- **Autonomous Support**:
  - Functions for use in autonomous routines
  - Blocking and non-blocking operation modes

## Usage

### Controller Mapping

The system uses a dual-controller setup to separate driving controls from the High Scoring mechanism controls:

**Main Controller (Master):**
- Used for driving the robot with standard arcade control

**Partner Controller:**
- **UP Arrow**: Move to GROUND position
- **DOWN Arrow**: Move to CAPTURE position (precise)
- **LEFT Arrow**: Move to WAIT position
- **RIGHT Arrow**: Move to SCORE position
- **Right Joystick Y-Axis**: Manual control (overrides automatic positioning)

This separation allows one driver to focus on robot movement while another controls the High Scoring mechanism independently.

### Autonomous Usage

In autonomous mode, you can use the following functions:

```cpp
// Move to a predefined position
moveHighScoringToPosition(HighScoringPosition::GROUND, true);  // Blocking call
moveHighScoringToPosition(HighScoringPosition::CAPTURE, false); // Non-blocking call

// Move to a specific angle
moveHighScoringToAngle(45.0, true, true);  // Move to 45 degrees with precise control, blocking
moveHighScoringToAngle(120.0, false, false); // Move to 120 degrees with general control, non-blocking
```

## Calibration

The system requires calibration to work properly. The following values may need to be adjusted:

1. **Position Values**: In `high_scoring_control.hpp`, adjust the angle values for each position:
   ```cpp
   namespace HighScoringPositions {
       const double GROUND = 0.0;     // Adjust as needed
       const double CAPTURE = 90.0;   // Adjust as needed
       const double WAIT = 180.0;     // Adjust as needed
       const double SCORE = 270.0;    // Adjust as needed
   }
   ```

2. **PID Constants**: In `high_scoring_control.hpp`, tune the PID constants for optimal performance:
   ```cpp
   namespace HighScoringPID {
       // PID constants for general movement
       const double KP_GENERAL = 0.5;  // Adjust as needed
       const double KI_GENERAL = 0.0;  // Adjust as needed
       const double KD_GENERAL = 0.1;  // Adjust as needed
       
       // PID constants for capture position
       const double KP_CAPTURE = 0.8;  // Adjust as needed
       const double KI_CAPTURE = 0.01; // Adjust as needed
       const double KD_CAPTURE = 0.2;  // Adjust as needed
       
       // Error thresholds
       const double POSITION_THRESHOLD_GENERAL = 5.0;  // Adjust as needed
       const double POSITION_THRESHOLD_CAPTURE = 1.0;  // Adjust as needed
   }
   ```

## Implementation Details

The system consists of the following components:

1. **high_scoring_control.hpp**: Header file defining the interface and constants
2. **high_scoring_control.cpp**: Implementation of the control functions
3. **Integration in main.cpp**: Usage in operator control and autonomous routines

The rotational sensor is used to track the position of the High Scoring mechanism, and the PID controller adjusts the motor power to reach and maintain the target position.

## Troubleshooting

If the mechanism is not reaching the desired positions accurately:

1. Check the rotational sensor connection and orientation
2. Verify that the rotational sensor is properly calibrated (reset to zero in the initialize function)
3. Adjust the PID constants for better performance
4. Increase the timeout value if the mechanism needs more time to reach the target position
5. Adjust the position thresholds if the mechanism is oscillating around the target position
