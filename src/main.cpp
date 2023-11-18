#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"
using namespace okapi;
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalIn cataSwitch('A');
pros::ADIDigitalOut leftPiston('C');
pros::ADIDigitalOut rightPiston('B');
pros::ADIDigitalOut odomLift('E');

pros::Motor driveLeftMotorBottom(18, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMotorMiddle(19, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMotorTop(20, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

//individual motors for drive right side
pros::Motor driveRightMotorBottom(11, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMotorMiddle(12, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMotorTop(13, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
	
//motor groups for drive left and rights sides
pros::Motor_Group leftMotorGroup( {driveLeftMotorBottom, driveLeftMotorMiddle, driveLeftMotorTop} );
pros::Motor_Group rightMotorGroup( {driveRightMotorBottom, driveRightMotorMiddle, driveRightMotorTop} );
pros::Motor_Group fullMotorGroup({driveRightMotorBottom, driveRightMotorMiddle, driveRightMotorTop, driveLeftMotorBottom, driveLeftMotorMiddle, driveLeftMotorTop});

//motors for intake and catapult
pros::Motor intakeMotor(16, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata(2, pros::E_MOTOR_GEAR_RED, 1, pros::E_MOTOR_ENCODER_DEGREES);

pros::IMU inertial(1);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void lowerCata(pros::Motor cata, pros::ADIDigitalIn limit, int volts){
	while (1){
		if (limit.get_value()) {
			break;
		}
		cata.move_voltage(volts);
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
			cata.tare_position();
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

    leftMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    rightMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

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
	
	driveLeftMotorBottom.set_zero_position(0);
	driveLeftMotorMiddle.set_zero_position(0);
	driveLeftMotorTop.set_zero_position(0);

	//individual motors for drive right side
	driveRightMotorBottom.set_zero_position(0);
	driveRightMotorMiddle.set_zero_position(0);
	driveRightMotorTop.set_zero_position(0);

	inertial.reset();
	// auton::position_track();
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
			auton::turn_to_angle(-45 + 180);
			auton::drive_distance(-62.2);
			auton::turn_to_angle(0+180);
			auton::drive_distance(-50.0);
			auton::drive_distance(14);
			auton::turn_to_angle(90);
			auton::turn_to_angle(0);
			auton::drive_distance(-20.2);
			rightPiston.set_value(true);
			auton::turn_to_angle(133.4+180);
			auton::drive_distance(-62.9);
			rightPiston.set_value(false);
			auton::turn_to_angle(87.3 + 180);
			auton::drive_distance(-100.3);
			auton::turn_to_angle(-90);
			// fullMotorGroup.moveVoltage(-9000);
			// pros::delay(2000);
			// fullMotorGroup.moveVoltage(0);
			// fullMotorGroup.moveVoltage(9000);
			// pros::delay(250);
			// fullMotorGroup.moveVoltage(0);
			// break;
			// // Far side
			// auton::turnAngle(chassis, 46.8);
			// auton::moveDistance(chassis, 55.7);
			// auton::turnAngle(chassis, 0.0);
			// auton::moveDistance(chassis, 58.4);
			// auton::turnAngle(chassis, 180.0);
			// auton::moveDistance(chassis, 20.3);
			// auton::turnAngle(chassis, -92.9);
			// auton::moveDistance(chassis, 99.2);
			// auton::turnAngle(chassis, -62.2);
			// auton::moveDistance(chassis, 54.5);
			// auton::turnAngle(chassis, 68.2);
			// auton::moveDistance(chassis, 109.4);
			// auton::turnAngle(chassis, -65.6);
			// auton::moveDistance(chassis, 61.4);
			// auton::turnAngle(chassis, 92.6);
			// auton::moveDistance(chassis, 55.9);
			// auton::turnAngle(chassis, -88.6);
			// auton::moveDistance(chassis, 106.7);
			// auton::turnAngle(chassis, 76.6);
			// auton::moveDistance(chassis, 109.7);
		case 2:
			break;
		case 3:
			lowerCata(cata, cataSwitch, 4000);
			break;
		case -1:
			//Close Side
			// auton::turnAngle(chassis, 88.3);
			// auton::moveDistance(chassis, 86.4);
			// auton::turnAngle(chassis, -96.3);
			// auton::moveDistance(chassis, 115.0);
			// auton::turnAngle(chassis, -46.4);
			// auton::moveDistance(chassis, 73.7);
			// auton::turnAngle(chassis, -3.2);
			// auton::moveDistance(chassis, 45.8);
			break;
		case -2:
			break;
		case -3:

			break;
		case 0:
			auton::progSkills(cata);
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
	// odomLift.set_value(true);
	cata.set_brake_mode(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
	bool leftToggle = false;
	bool rightToggle = false;
    while(true) {
		// Intake control
		intakeMotor.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? -8000 : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 8000 : 0));

		// Trigger right piston
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) == 1) {
			rightPiston.set_value(true);
		} else {
			rightPiston.set_value(false);
		}
		

		// Trigger left piston
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) == 1) {
			leftPiston.set_value(true);
		} else {
			leftPiston.set_value(false);
		}
		

		// Catapult control
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			cata.move_voltage(11000);
		} else {
			cata.move_voltage(0);
		}
		

		// Lower catapult
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) lowerCata(cata, cataSwitch, 10000);


        //naturalize input to a range between -1 and 1
        double leftInput = (double) master.get_analog(ANALOG_LEFT_Y) / 127;
        double rightInput = (double) 0.75 * master.get_analog(ANALOG_RIGHT_X) / 127;
		
		rightMotorGroup.move_voltage(12000 * (rightInput - leftInput));
		leftMotorGroup.move_voltage(12000 * (rightInput + leftInput));

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