#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"



pros::Controller master(pros::E_CONTROLLER_MASTER);

// Solenoids for wings
pros::ADIDigitalOut wingFrontLeft(wingFrontLeftID);
pros::ADIDigitalOut wingFrontRight(wingFrontRightID);
pros::ADIDigitalOut wingBackLeft(wingBackLeftID);
pros::ADIDigitalOut wingBackRight(wingBackRightID);

// Individual motors for drive left side
pros::Motor driveLeftFront(driveLeftFrontID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMiddle(driveLeftMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftBack(driveLeftBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

// Individual motors for drive right side
pros::Motor driveRightFront(driveRightFrontID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMiddle(driveRightMiddleID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightBack(driveRightBackID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
	
// Motor groups for drive left and rights sides
pros::Motor_Group driveLeft( {driveLeftFront, driveLeftMiddle, driveLeftBack} );
pros::Motor_Group driveRight( {driveRightFront, driveRightMiddle, driveRightBack} );
pros::Motor_Group fullMotorGroup( {driveRightBack, driveRightMiddle, driveRightFront, driveLeftBack, driveLeftMiddle, driveLeftFront} );

// Motors for intake and catapult
pros::Motor intakeMotor(intakeMotorID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
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

    driveLeft.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    driveRight.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	cataMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	
	driveLeftBack.set_zero_position(0);
	driveLeftMiddle.set_zero_position(0);
	driveLeftFront.set_zero_position(0);
	driveRightBack.set_zero_position(0);
	driveRightMiddle.set_zero_position(0);
	driveRightFront.set_zero_position(0);

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

			auton::driveDistance(24);

			// frontPistons.set_value(true);
			// pros::delay(1500);
			// frontPistons.set_value(false);
			// auton::driveTurn(47, -45, 0.2);
			// auton::driveTurn(0, 45, 1);
			// auton::turn_to_angle(45);
			// frontPistons.set_value(true);
			// auton::driveTurn(30, 0, 0, {11000, 1.5, 0, 0.85, 0, 100, 1.5, 500});
			// frontPistons.set_value(false);
			//auton::drive_with_voltage(0,0);

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

std::vector<float> arcadeControl() {
		// output voltages of left and right in vector
		std::vector<float> voltages = {0, 0};

		//normalize inputs to [-1,1]
		float leftInput = (float)master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127;
		float rightInput = ((float)master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127) * ((float)3/4);

		float max = std::max(fabs(leftInput), fabs(rightInput));
		float difference = leftInput - rightInput;
		float total = leftInput + rightInput;

		if(leftInput >= 0) {
			if(rightInput >= 0) {
				voltages = std::vector<float> {max, difference};
			} else {
				voltages = std::vector<float> {total, max};
			}
		} else {
			if(rightInput >= 0) {
				voltages = std::vector<float> {total, -max};
			} else {
				voltages = std::vector<float> {-max, difference};
			}
		}

		return voltages;
}

void opcontrol() {
    while(true) {
		// Intake control
		int shiftKey = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		//intakeMotor.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? -12000 : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 12000 : 0));

		// Trigger pistons
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if(shiftKey) {
				intakeMotor.move_voltage(-12000);
			} else {
				intakeMotor.move_voltage(12000);
			}
		} else {
				intakeMotor.move_voltage(0);
		}

		//left wings with shift key
		if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			if(shiftKey) {
				wingBackLeft.set_value(true);
			} else {
				wingFrontLeft.set_value(true);
			}
		} else {
			wingBackLeft.set_value(false);
			wingFrontLeft.set_value(false);
		}

		//right wings with shift key
		if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			if(shiftKey) {
				wingBackRight.set_value(true);
			} else {
				wingFrontRight.set_value(true);
			}
		} else {
			wingBackRight.set_value(false);
			wingFrontRight.set_value(false);
		}

		// Catapult control
		cataMotorGroup.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ? 11000 : 0);

		driveLeft.move_voltage(arcadeControl()[0] * 12000);
		driveRight.move_voltage(arcadeControl()[1] * 12000);

        // Naturalize input to a range between -1 and 1
        //double leftInput = (double) master.get_analog(ANALOG_LEFT_Y) / 127; // Drive
        //double rightInput = (double) 0.5 * master.get_analog(ANALOG_RIGHT_X) / 127; // Turn
		
		//rightMotorGroup.move_voltage(12000 * (rightInput - leftInput));
		//leftMotorGroup.move_voltage(12000 * (rightInput + leftInput));

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
