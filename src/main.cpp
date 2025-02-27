#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Motor left_motor_a(2);
pros::Motor left_motor_b(3);
pros::Motor left_motor_c(4);
pros::Motor right_motor_a(5);
pros::Motor right_motor_b(12);
pros::Motor right_motor_c(17);

pros::MotorGroup left_drive_smart({-2, -3, -4});
pros::MotorGroup right_drive_smart({5, 12, 17});

pros::Motor High_scoring(20);
pros::Motor intake_lower(21);
pros::Motor intake_upper(13);

pros::adi::DigitalOut mogo_p('F');
pros::adi::DigitalOut ejection_p('A');

pros::adi::DigitalOut donker('H');

pros::adi::DigitalOut intake_p('D');

pros::Rotation rotational_sensor(-19);

pros::Rotation left_rotational_sensor(7);
pros::Rotation right_rotational_sensor(6);


pros::Imu imu(11);

// horizontal tracking wheel
lemlib::TrackingWheel left_tracking_wheel(&left_rotational_sensor, lemlib::Omniwheel::NEW_275, -7.75);
// vertical tracking wheel
lemlib::TrackingWheel right_tracking_wheel(&right_rotational_sensor, lemlib::Omniwheel::NEW_275, 7.75);


lemlib::OdomSensors sensors(&left_tracking_wheel, // vertical tracking wheel 1, set to null
    &right_tracking_wheel, // vertical tracking wheel 2, set to nullptr as we are using IMEs
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
                                              15, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(25, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              9, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_drive_smart, // left motor group
    &right_drive_smart, // right motor group
    13.5, // 10 inch track width
    lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
    450, // drivetrain rpm is 450
    2 // horizontal drift is 2 (for now)
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
	left_rotational_sensor.reset_position();
	right_rotational_sensor.reset_position();
	chassis.calibrate(); // calibrate sensors
	printf("Calibrated\n");
	// print position to brain screen
	pros::Task screen_task([&]()
						   {
		while (true) {
			// print robot location to the brain screen
			printf("X: %f, Y:%f, @:%f \n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);		   // x
			// delay to save resources
			pros::delay(20);
		} });
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

void autonomous() {
	// set position to x:0, y:0, heading:0
    chassis.setPose(0, 200, 0);
    // move 48" forwards
    chassis.moveToPoint(0, 150, 10000, {.forwards = false});
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_drive_smart.move(dir - turn);                      // Sets left motor voltage
		right_drive_smart.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}