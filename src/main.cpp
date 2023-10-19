#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"
using namespace okapi;
pros::Controller master(pros::E_CONTROLLER_MASTER);
std::shared_ptr<ChassisController> chassis;

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
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	selector::init();
	Logger::setDefaultLogger(
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
			"/ser/sout", // Output to the PROS terminal
			Logger::LogLevel::error
		)
	);
	
	//Cata limit switch
	pros::ADIDigitalIn cataSwitch(pros::DIGITAL_A);

	//Catapult motor
	Motor cata(10,true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

    //left side motor group
    Motor leftMotor1(18, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor leftMotor2(19, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor leftMotor3(20, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    MotorGroup leftMotorGroup ({leftMotor1, leftMotor2, leftMotor3});

    //right side motor group
    Motor rightMotor1(11, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor rightMotor2(12, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    Motor rightMotor3(13, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    MotorGroup rightMotorGroup ({rightMotor1, rightMotor2, rightMotor3});

	//Chassis scale 
	// TODO - Get correct dimensions
	ChassisScales scale({3.25_in, 12.5_in, 3_in, 3.25_in}, imev5BlueTPR);

// Chassis Controller - lets us chassis the robot around with open- or closed-loop control
	chassis =
		ChassisControllerBuilder()
			.withMotors(leftMotorGroup, rightMotorGroup)
			// Blue gearset, 3.25 in wheel diam, 11.7 in wheel track
			.withDimensions(AbstractMotor::gearset::blue, scale)
			.withMaxVoltage((double) 12000) //Motor's max voltage
			.withGains(
				{1.5, 0, 10}, //Driving PID
				{0, 0, 0}) //Turning PID
			.withSensors(RotationSensor(17), RotationSensor(14), RotationSensor(15))
			.withOdometry()
			.buildOdometry();
    leftMotorGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightMotorGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
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
void autonomous() {
	switch(selector::auton) {
		case 1:
			break;
		case 2:
			// code block
			break;
		case 3:
			// code block
			break;
		case -1:
			// code block
			break;
		case -2:
			// code block
			break;
		case -3:
			// code block
			break;
		case 0:
			auton::test(chassis, cataSwitch, cata);
			break;
		default:
			// code block
			break;
	}
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
    while(true) {
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1{
			cata.moveVoltage(2000);
		}
        //naturalize input to a range between -1 and 1
        double leftInput = (double) master.get_analog(ANALOG_LEFT_Y)/127;
        double rightInput = (double) master.get_analog(ANALOG_RIGHT_X)/127;
		chassis->getModel()->arcade(leftInput, rightInput);
        pros::delay(20);
    }



	// while (true) {
	// 	pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
	// 	                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
	// 	int left = master.get_analog(ANALOG_LEFT_Y);
	// 	int right = master.get_analog(ANALOG_RIGHT_Y);

	// 	left_mtr = left;
	// 	right_mtr = right;

	// 	pros::delay(20);
	// }
}
