#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "high_scoring_control.hpp"

pros::MotorGroup left_drive_smart({-2, -3, -4}, pros::v5::MotorGears::blue);
pros::MotorGroup right_drive_smart({5, 12, 17}, pros::v5::MotorGears::blue);

pros::Motor High_scoring(20);
pros::Motor intake_lower(21);
pros::Motor intake_upper(13);

pros::adi::DigitalOut mogo_p('F');
pros::adi::DigitalOut ejection_p('A');

pros::adi::DigitalOut donker('H');

pros::adi::DigitalOut intake_p('D');

pros::Rotation rotational_sensor(-19);

pros::Rotation vertical_rotational_sensor(7);
pros::Rotation horizontal_rotational_sensor(-6);

pros::Imu imu(9);

// horizontal tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotational_sensor, lemlib::Omniwheel::NEW_2, 0.5);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotational_sensor, lemlib::Omniwheel::NEW_2, 2.5);


lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    &horizontal_tracking_wheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

// ...existing code...
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
		ejection_p.set_value(false);
	}

// lateral PID controller
lemlib::ControllerSettings lateral_controller(30, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              20, // small error range timeout, in milliseconds
                                              10, // large error range, in inches
                                              1000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              33, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              00, // small error range timeout, in milliseconds
                                              00, // large error range, in inches
                                              0000, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_drive_smart, // left motor group
    &right_drive_smart, // right motor group
    13.5, // 10 inch track width
    lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 450
    6 // horizontal drift is 2 (for now)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odometry sensors
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn1_cb(on_center_button);
	donker.set_value(false);
	rotational_sensor.reset_position();
	vertical_rotational_sensor.reset_position();
	horizontal_rotational_sensor.reset_position();
	chassis.calibrate(); // calibrate sensors
	printf("Calibrated\n");
	// print position to brain screen
	pros::Task screen_task([&]()
						   {
		while (true) {
			// print robot location to the brain screen
			 printf("X: %f, Y:%f, @:%f @:%f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, imu.get_heading());		   // x
			// delay to save resources
			pros::delay(1000);
		} });

		//autonomous();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// path file name is "example.txt".
// "." is replaced with "_" to overcome c++ limitations
ASSET(example_txt);
ASSET(emptymogo1_txt);
ASSET(crazypaathx1_txt);
ASSET(lemlibsample_txt);
ASSET(right45_txt);
ASSET(left90_txt);
ASSET(p1_txt);
ASSET(p2_txt);

void autonomous() {
    // Set chassis pose
    //chassis.setPose(-59.007, -1.12, 90);
    chassis.setPose(0,0,0);
	chassis.turnToHeading(90, 100000, {}, false);
    
    /* Start with the High Scoring mechanism in the GROUND position
    moveHighScoringToPosition(HighScoringPosition::GROUND, true);
    pros::lcd::print(2, "Moving to GROUND position");
    *
    // Follow the first path
	// Follow the first path
    printf("Following first path\n");
	printf("Starting position: X: %f, Y: %f, Theta: %f\n", 
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    
    chassis.follow(p1_txt, 3, 60000, true, false);
    printf("Finished first path\n");
    printf("Position after p1: X: %f, Y: %f, Theta: %f\n", 
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
     
    
    /* Move to CAPTURE position (precise positioning)
    moveHighScoringToPosition(HighScoringPosition::CAPTURE, true);
    pros::lcd::print(2, "Moving to CAPTURE position");
    */
    // Wait for a moment to simulate capturing a game object
    pros::delay(15000);
    
    /** Move to WAIT position
    moveHighScoringToPosition(HighScoringPosition::WAIT, true);
    pros::lcd::print(2, "Moving to WAIT position");
	*
    
    // Follow the second path
	printf("Following second path\n");
    chassis.follow(p2_txt, 5, 60000, false, false);
    printf("Finished second path\n");
	 printf("Final position: X: %f, Y: %f, Theta: %f\n", 
           chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
   
	/*
    // Move to SCORE position
    moveHighScoringToPosition(HighScoringPosition::SCORE, true);
    pros::lcd::print(2, "Moving to SCORE position");
    // Wait for a moment to simulate scoring
    pros::delay(500);
    
    // Return to GROUND position
    moveHighScoringToPosition(HighScoringPosition::GROUND, true);
    pros::lcd::print(2, "Moving to GROUND position");
   */ 
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// Main controller for driving
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	
	// Variables to track button states to detect button presses
	bool btnUp_pressed = false;
	bool btnDown_pressed = false;
	bool btnLeft_pressed = false;
	bool btnRight_pressed = false;

	// Print current position of the High Scoring mechanism
	pros::Task position_display_task([&]() {
		while (true) {
			// Print the current angle of the High Scoring mechanism
			printf("High Scoring Angle: %.2f\n", getHighScoringAngle());
			pros::delay(100);
		}
	});

	while (true) {
		/*printf("%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
						 */

		// Tank control scheme for driving (using master controller)
		int left_power = master.get_analog(ANALOG_LEFT_Y);    // Gets left side power from left joystick
		int right_power = master.get_analog(ANALOG_RIGHT_Y);  // Gets right side power from right joystick
		left_drive_smart.move(left_power);                    // Sets left motor voltage
		right_drive_smart.move(right_power);                  // Sets right motor voltage

		// Check for button presses to control the High Scoring mechanism (using master controller)
		
		// UP button: Move to GROUND position
		if (master.get_digital(DIGITAL_UP) && !btnUp_pressed) {
			btnUp_pressed = true;
			moveHighScoringToPosition(HighScoringPosition::GROUND, false);
			printf("Moving to GROUND position\n");
		} else if (!master.get_digital(DIGITAL_UP)) {
			btnUp_pressed = false;
		}
		
		// DOWN button: Move to CAPTURE position (precise)
		if (master.get_digital(DIGITAL_DOWN) && !btnDown_pressed) {
			btnDown_pressed = true;
			moveHighScoringToPosition(HighScoringPosition::CAPTURE, false);
			printf("Moving to CAPTURE position");
		} else if (!master.get_digital(DIGITAL_DOWN)) {
			btnDown_pressed = false;
		}
		
		// LEFT button: Move to WAIT position
		if (master.get_digital(DIGITAL_LEFT) && !btnLeft_pressed) {
			btnLeft_pressed = true;
			moveHighScoringToPosition(HighScoringPosition::WAIT, false);
			printf("Moving to WAIT position");
		} else if (!master.get_digital(DIGITAL_LEFT)) {
			btnLeft_pressed = false;
		}
		
		// RIGHT button: Move to SCORE position
		if (master.get_digital(DIGITAL_RIGHT) && !btnRight_pressed) {
			btnRight_pressed = true;
			moveHighScoringToPosition(HighScoringPosition::SCORE, false);
			printf("Moving to SCORE position");
		} else if (!master.get_digital(DIGITAL_RIGHT)) {
			btnRight_pressed = false;
		}

		/* Manual control of the High Scoring mechanism using the master controller's right joystick Y axis
		int high_scoring_manual = master.get_analog(ANALOG_RIGHT_Y);
		if (std::abs(high_scoring_manual) > 10) {  // Small deadzone
			// Stop the automatic control task when manual control is detected
			stopHighScoringTask();
			
			// Apply manual control
			High_scoring.move(high_scoring_manual);
			printf("Manual control: %d", high_scoring_manual);
		}*/
		
		pros::delay(20);  // Run for 20 ms then update
	}
}
