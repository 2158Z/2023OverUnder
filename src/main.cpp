#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"
using namespace okapi;
pros::Controller master(pros::E_CONTROLLER_MASTER);
std::shared_ptr<OdomChassisController> chassis;
pros::ADIDigitalIn cataSwitch('A');
pros::ADIDigitalOut leftPiston('C');
pros::ADIDigitalOut rightPiston('B');
pros::ADIDigitalOut odomLift('E');
Motor cata(2, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor intakeMotor(16, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
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

MotorGroup fullMotorGroup({rightMotor1, rightMotor2, rightMotor3, leftMotor1, leftMotor2, leftMotor3});

IMU inertial(1);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void lowerCata(okapi::Motor cata, pros::ADIDigitalIn limit, int volts){
	while (1){
		if (limit.get_value()) {
			break;
		}
		cata.moveVoltage(volts);
	}
}

void on_center_button() { 
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

//async tasks. this is used for running multiple functions at once.
void task_limit(void* param) {
	while(true){
		if (cataSwitch.get_value() == 1){
			cata.tarePosition();
		}
		pros::delay(20);
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

	//Chassis scale 
	// TODO - Get correct dimensions
	ChassisScales scale({3.25_in, 12.5_in, 7.5_in, 3.25_in}, imev5BlueTPR);
	auto middle = RotationSensor(7);
// Chassis Controller - lets us chassis the robot around with open- or closed-loop control
	chassis =
		ChassisControllerBuilder()
			.withMotors(leftMotorGroup, rightMotorGroup)
			// Blue gearset, 3.25 in wheel diam, 11.7 in wheel track
			.withDimensions(AbstractMotor::gearset::blue, scale)
			.withMaxVoltage((double) 12000) //Motor's max voltage
			.withGains(
				{0.0025, 0, 0}, //Driving PID
				{0.0025, 0, 0}) //Turning PID
			// .withSensors(middle, middle)
			.withSensors(leftMotorGroup.getEncoder(),rightMotorGroup.getEncoder())
			.withOdometry()
			.buildOdometry();

    leftMotorGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightMotorGroup.setBrakeMode(AbstractMotor::brakeMode::coast);
	fullMotorGroup.setBrakeMode(AbstractMotor::brakeMode::coast);

	// std::unique_ptr<okapi::IterativePosPIDController> distance = IterativePosPIDController({0,0,0},TimeUtilFactory::createDefault().getTimer());
	// std::unique_ptr<okapi::IterativePosPIDController> turn = IterativePosPIDController({0,0,0},TimeUtilFactory::createDefault().getTimer());
	// std::unique_ptr<okapi::IterativePosPIDController> angle = IterativePosPIDController({0,0,0},TimeUtilFactory::createDefault().getTimer());

	// chassisPID = ChassisControllerPID(
	// 	TimeUtilFactory::createDefault().getTimer(),
	// 	chassis->getModel(),
	// 	distance,
	// 	turn,
	// 	angle,
	// 	AbstractMotor::gearset::blue,
	// 	scale
	// );
	inertial.calibrate();
	inertial.reset();
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
	chassis -> setMaxVelocity(280);
	switch(selector::auton) {
		case 1:	
			fullMotorGroup.moveVoltage(-9000);
			pros::delay(2000);
			fullMotorGroup.moveVoltage(0);
			fullMotorGroup.moveVoltage(9000);
			pros::delay(250);
			fullMotorGroup.moveVoltage(0);
			break;
			// Far side
			auton::turnAngle(chassis, 46.8);
			auton::moveDistance(chassis, 55.7);
			auton::turnAngle(chassis, 0.0);
			auton::moveDistance(chassis, 58.4);
			auton::turnAngle(chassis, 180.0);
			auton::moveDistance(chassis, 20.3);
			auton::turnAngle(chassis, -92.9);
			auton::moveDistance(chassis, 99.2);
			auton::turnAngle(chassis, -62.2);
			auton::moveDistance(chassis, 54.5);
			auton::turnAngle(chassis, 68.2);
			auton::moveDistance(chassis, 109.4);
			auton::turnAngle(chassis, -65.6);
			auton::moveDistance(chassis, 61.4);
			auton::turnAngle(chassis, 92.6);
			auton::moveDistance(chassis, 55.9);
			auton::turnAngle(chassis, -88.6);
			auton::moveDistance(chassis, 106.7);
			auton::turnAngle(chassis, 76.6);
			auton::moveDistance(chassis, 109.7);
		case 2:
			chassis -> moveDistance(36_in); //Push matchload in
			chassis -> moveDistance(-24_in);
			auton::wings(leftPiston, 2000);
			chassis -> turnAngle(90_deg); //Spin matchload out
			chassis -> moveDistance(-52_in); //Push triballs over
			chassis -> moveDistance(36_in);
			chassis -> turnAngle(-90_deg);
			chassis -> moveDistance(24_in);
			chassis -> turnAngle(-90_deg);
			chassis -> moveDistance(36_in);
			chassis -> turnAngle(90_deg);
			chassis -> moveDistance(-24_in);
			break;
		case 3:
			lowerCata(cata, cataSwitch, 4000);
			break;
		case -1:
			//Close Side
			auton::turnAngle(chassis, 88.3);
			auton::moveDistance(chassis, 86.4);
			auton::turnAngle(chassis, -96.3);
			auton::moveDistance(chassis, 115.0);
			auton::turnAngle(chassis, -46.4);
			auton::moveDistance(chassis, 73.7);
			auton::turnAngle(chassis, -3.2);
			auton::moveDistance(chassis, 45.8);
			break;
		case -2:
			break;
		case -3:

			break;
		case 0:
			auton::progSkills(chassis, cata);
			break;
		default:
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
	chassis -> setMaxVelocity(360);
	// odomLift.set_value(true);
	cata.setBrakeMode(AbstractMotor::brakeMode::hold);
	bool leftToggle = false;
	bool rightToggle = false;
    while(true) {
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intakeMotor.moveVoltage(-8000);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intakeMotor.moveVoltage(8000);
		} else {
			intakeMotor.moveVoltage(0);
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) == 1){
			rightToggle = !rightToggle;
		}

		rightPiston.set_value(rightToggle);

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) == 1){
			leftToggle = !leftToggle;
		}
		leftPiston.set_value(leftToggle);

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 1){
			cata.moveVoltage(9000);
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) == 0){
			cata.moveVoltage(0);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
			lowerCata(cata,cataSwitch,10000);
		}

        //naturalize input to a range between -1 and 1
        double leftInput = (double) master.get_analog(ANALOG_LEFT_Y)/127;
        double rightInput = (double) master.get_analog(ANALOG_RIGHT_X)/127;
		chassis->getModel()->arcade(leftInput, rightInput);
        pros::delay(20);
    }
/*
 R1 - Intake in
 L1 - intake out
 R2 - Right wing
 L2 - Left wing
 A - Cata
*/
}