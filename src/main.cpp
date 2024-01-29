#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"
using namespace okapi;
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Solenoids for wings
pros::ADIDigitalOut wingFrontLeft(wingFrontLeftID);
pros::ADIDigitalOut wingFrontRight(wingFrontRightID);
pros::ADIDigitalOut wingBackLeft(wingBackLeftID);
pros::ADIDigitalOut wingBackRight(wingBackRightID);

// Individual motors for drive left side
pros::Motor driveLeftMotorFront(driveLeftMotorFrontID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMotorMiddle(driveLeftMotorMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMotorBack(driveLeftMotorBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

// Individual motors for drive right side
pros::Motor driveRightMotorFront(driveRightMotorFrontID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMotorMiddle(driveRightMotorMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMotorBack(driveRightMotorBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
	
// Motor groups for drive left and rights sides
pros::Motor_Group leftMotorGroup( {driveLeftMotorBack, driveLeftMotorMiddle, driveLeftMotorFront} );
pros::Motor_Group rightMotorGroup( {driveRightMotorBack, driveRightMotorMiddle, driveRightMotorFront} );
pros::Motor_Group fullMotorGroup( {driveRightMotorBack, driveRightMotorMiddle, driveRightMotorFront, driveLeftMotorBack, driveLeftMotorMiddle, driveLeftMotorFront} );

// Motors for intake and catapult
pros::Motor intakeMotor(intakeMotorID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata1(cata1MotorID, pros::E_MOTOR_GEAR_GREEN, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata2(cata2MotorID, pros::E_MOTOR_GEAR_RED, 1, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group cataMotorGroup( {cata1, cata2} );

pros::IMU inertial(inertialID);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	selector::init();

    leftMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    rightMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	cataMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	
	driveLeftMotorBack.set_zero_position(0);
	driveLeftMotorMiddle.set_zero_position(0);
	driveLeftMotorFront.set_zero_position(0);
	driveRightMotorBack.set_zero_position(0);
	driveRightMotorMiddle.set_zero_position(0);
	driveRightMotorFront.set_zero_position(0);

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
	switch(selector::auton) {
		case 1:	
			

			break;
		case 2: //Close Side Elim
		
			break;
		case 3:
			
			break;
		case -1: //  Farside-0.
			
			break;
		case -2:
			
			break;
		case -3:
			
			break;
		case 0:
			
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
    while(true) {
		// Intake control
		int shiftKey = pros::E_CONTROLLER_DIGITAL_L1;
		//intakeMotor.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? -12000 : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 12000 : 0));

		// Trigger pistons
		if(pros::E_CONTROLLER_DIGITAL_R1) {
			if(shiftKey) {
				intakeMotor.move_voltage(12000);
			} else {
				intakeMotor.move_voltage(-12000);
			}
		} else {
				intakeMotor.move_voltage(0);
		}

		printf("%f", pros::E_CONTROLLER_DIGITAL_R2);

		// Catapult control
		cataMotorGroup.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ? 11000 : 0);

        // Naturalize input to a range between -1 and 1
        double leftInput = (double) master.get_analog(ANALOG_LEFT_Y) / 127; // Drive
        double rightInput = (double) 0.5 * master.get_analog(ANALOG_RIGHT_X) / 127; // Turn
		
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
