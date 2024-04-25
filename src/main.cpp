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

// Hang
pros::ADIDigitalOut hang(hangID);
// Individual motors for drive left side
pros::Motor driveLeftFront(driveLeftFrontID, pros::E_MOTOR_GEAR_600, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMiddle(driveLeftMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftBack(driveLeftBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

// Individual motors for drive right side
pros::Motor driveRightFront(driveRightFrontID, pros::E_MOTOR_GEAR_600, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMiddle(driveRightMiddleID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightBack(driveRightBackID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
	
// Motor groups for drive left and rights sides
pros::Motor_Group driveLeft( {driveLeftFront, driveLeftMiddle, driveLeftBack} );
pros::Motor_Group driveRight( {driveRightFront, driveRightMiddle, driveRightBack} );
pros::Motor_Group fullMotorGroup( {driveRightBack, driveRightMiddle, driveRightFront, driveLeftBack, driveLeftMiddle, driveLeftFront} );

// Motors for intake and catapult
pros::Motor intakeMotorLeft(intakeMotorLeftID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intakeMotorRight(intakeMotorRightID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group intakeMotorGroup( {intakeMotorLeft, intakeMotorRight});

pros::Motor cata1(cata1MotorID, pros::E_MOTOR_GEAR_GREEN, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor cata2(cata2MotorID, pros::E_MOTOR_GEAR_RED, 0, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group cataMotorGroup( {cata1, cata2} );

pros::IMU inertial(inertialID);

pros::ADIDigitalIn hangLimit(hangLimitID);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	selector::init();

	fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
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
void disabled() {

	// while(true) {
	// 	inertial.set_heading(0);
	// 	pros::delay(1000);
	// }

}

	

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
	while(inertial.is_calibrating()){
		pros::delay(10);
	}
	//inertial.set_heading(0);
	switch(selector::auton) {				// close safe wp
		case 1:

			//pushes the colored ball into goal
			intakeMotorGroup.move_voltage(-12000);
			auton::driveDistance(-20);
			auton::driveTurn(-7, 45, 0.4, 75, 2000);
		    auton::driveDistance(-20, 1000);
			intakeMotorGroup.move_voltage(0);
			auton::driveDistance(5);
			auton::driveDistance(-20);
			auton::absTurn(45);
			pros::delay(250);

			// moves towards the corner ball and pushes it
			auton::driveDistance(5);
			auton::driveTurn(22, -45, 0.6, 75, 2000);
			//auton::driveDistance(6);
			wingBackRight.set_value(true);
			delay(200);
			auton::turnAngle(-100);
			intakeMotorGroup.move_voltage(-12000);
			wingBackRight.set_value(false);
			pros::delay(250);

			// heads under the hang bar
			auton::absTurn(0);
			auton::driveDistance(5);
			auton::absTurn(-45);
			auton::driveDistance(35);

			break;
		case 2: 	//close side mid rush

		    // push preload with wing
			intakeMotorGroup.move_voltage(-12000);
			wingFrontLeft.set_value(true);


			// grab middle ball
			auton::turnAngle(20);
			wingFrontLeft.set_value(false);
			intakeMotorGroup.move_voltage(8000);
			auton::driveDistance(48);
			// return
			auton::driveDistance(-44);
			intakeMotorGroup.move_voltage(4000);
			// score preload
			auton::absTurn(125);
		    auton::driveDistance(-27);
			auton::absTurn(180);
			auton::driveDistance(-30,500);
			auton::driveDistance(10);
			auton::driveDistance(-30,500);

			// gets out corner ball
			auton::driveDistance(15.5);
			auton::driveTurn(15, -50, 0.6, 75, 2000);
			wingBackRight.set_value(true);
            pros::delay(250);
			auton::turnAngle(-100);
			wingBackRight.set_value(false);

			// heads under the hang bar
			auton::absTurn(135);
			auton::driveDistance(8);
			auton::absTurn(90);
			intakeMotorGroup.move_voltage(-12000);
			auton::driveDistance(33);

			break;
		case 3:
			break;
		case -1: //  far side safe

			// kicks the ball out of the corner
			intakeMotorGroup.move_voltage(-12000);
			pros::delay(100);
			intakeMotorGroup.move_voltage(8000);		
			auton::driveDistance(4, 750);
			auton::driveTurn(15, -45, 0.5, 75, 2000);
			wingBackRight.set_value(true);
			pros::delay(250);
			auton::turnAngle(-90);
			wingBackRight.set_value(false);
			intakeMotorGroup.move_voltage(0);

			// pushes both the balls into the goal
			auton::absTurn(-45);
			intakeMotorGroup.move_voltage(-6000);
			auton::driveTurn(10, -45, 0.25, 75, 2000);
			wingFrontLeft.set_value(true);
			auton::driveDistance(40);
			wingFrontLeft.set_value(false);
			intakeMotorGroup.move_voltage(0);

			// bumps the wall to line up
			auton::driveDistance(-7.5);
			auton::turnAngle(-90);
			auton::driveDistance(-10, 2000);

			// goes for first mid ball and score
			intakeMotorGroup.move_voltage(8000);
			auton::driveDistance(40);
			auton::driveTurn(12, 30, 0.6, 75, 2000);
			auton::turnAngle(125);
			auton::driveDistance(12, 750);
			intakeMotorGroup.move_voltage(-6000);
			pros::delay(500);

			// goes for second mid ball
			intakeMotorGroup.move_voltage(8000);
			auton::turnAngle(-100);
			auton::driveDistance(24);
			auton::absTurn(0);

			// scores the two balls
			wingFrontLeft.set_value(true);
			wingFrontRight.set_value(true);
			intakeMotorGroup.move_voltage(-6000);
			auton::driveDistance(54, 500, {9000, 0.17, 0.0005, 1, 2, 75, 0.25, 2000});
			pros::delay(250);
			auton::driveDistance(-20);
			break;
		case -2: // far side mid rush
            // rushes towards the far middle ball and scores both
			intakeMotorGroup.move_voltage(-12000);
			wingFrontRight.set_value(true);
			auton::driveTurn(10, -40, 0.35, 0);
			wingFrontRight.set_value(false);
			intakeMotorGroup.move_voltage(6000);
			auton::driveDistance(49);

			//Score into goal
			auton::absTurn(90);
			intakeMotorGroup.move_voltage(-12000);
			auton::driveDistance(33);
			intakeMotorGroup.move_voltage(0);

			// Intake third ball
			auton::driveTurn(-30, -90, 0.35);
			auton::absTurn(-90);
			intakeMotorGroup.move_voltage(6000);
			auton::driveDistance(14);

			// return to the start
			auton::driveTurn(-46, 65, 0.325);
			auton::absTurn(50);
			intakeMotorGroup.move_voltage(-6000);
			pros::delay(500);

			auton::absTurn(-90);
			intakeMotorGroup.move_voltage(6000);
			auton::driveDistance(26);
			auton::absTurn(90);

			auton::driveDistance(25);
			wingBackRight.set_value(true);
			auton::driveTurn(24, -90, 0.35);
			wingBackRight.set_value(false);
			intakeMotorGroup.move_voltage(-6000);
			auton::driveDistance(20);
			//saved kenny far safe
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveTurn(-18, -35, 0.35);
			// intakeMotorGroup.move_voltage(0);
			// wingBackLeft.set_value(true);
			// pros::delay(500);
			// auton::turnAngle(-50);
			// wingBackLeft.set_value(false);
			// auton::driveDistance(-24);
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveTurn(64.5, 105, 0.45, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.25, 3000});
			// auton::turnAngle(115);
			// intakeMotorGroup.move_voltage(-12000);
			// auton::driveDistance(10);
			// intakeMotorGroup.move_voltage(0);
			// auton::turnAngle(-75);
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveDistance(20);
			// // auton::turnAngle(112.5);
			// auton::absTurn(180);
			// intakeMotorGroup.move_voltage(-12000);
			// wingFrontLeft.set_value(true);
			// wingFrontRight.set_value(true);
			// auton::driveDistance(54);


			// auton::setDefaultDriveConstants({11000, 0.1, 0, 0, 0, 0, 0.1, 500});
			// intakeMotorGroup.move_voltage(9000);
			// wingBackLeft.set_value(true);
			// auton::driveTurn(-18, 315, 0.35);
			// wingBackLeft.set_value(false);
			// auton::driveTurn(-18, 270, 0.35);
			// intakeMotorGroup.move_voltage(0);
			// wingBackLeft.set_value(false);
			// auton::driveDistance(16);
			// auton::turnAngle(81);
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveDistance(60, {10000, 1.5125, 0, 0, 0, 100, 0.5, 1500});
			// auton::turnAngle(99);
			// wingFrontLeft.set_value(true);
			// auton::driveDistance(36, {12000, 0.1, 0, 0, 0, 0, 0.5, 1500});
			// intakeMotorGroup.move_voltage(-12000);
			// pros::delay(150);
			// intakeMotorGroup.move_voltage(0);
			// wingFrontLeft.set_value(false);
			// auton::driveDistance(-5);
			// auton::turnAngle(135);
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveDistance(25);
			// pros::delay(150);
			// auton::turnAngle(-180);
			// auton::driveDistance(25);
			// auton::turnAngle(45);
			// intakeMotorGroup.move_voltage(-12000);
			// auton::driveDistance(20);
			// intakeMotorGroup.move_voltage(0);
			// auton::driveDistance(-5);
			break;
		case -3:
			
			break;
		case 0:
			// // Skills (with front sweeps):

			// // pushes the triball into the goal
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveTurn(-24, 45, 0.1);
			// auton::absTurn(45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(-20);
			// intakeMotorGroup.move_voltage(0);

			// // positions the robot and starts kicker
			// auton::driveDistance(10);
			// auton::turnAngle(-115);
			// auton::driveDistance(-3);
			
			// wingBackRight.set_value(true);
			// cataMotorGroup.move_voltage(11000);
			// fullMotorGroup.move_voltage(250);
			// // delay(30000);
			// cataMotorGroup.move_voltage(0);
			
			// wingBackRight.set_value(false);

			// // goes to middle middle zone an picks up right 
			// intakeMotorGroup.move_voltage(12000);
			// auton::driveTurn(29, 45, 0.2625);
			// auton::driveDistance(22);

			// // pushes the other middle balls
			// auton::driveDistance(-5);
			// intakeMotorGroup.move_voltage(0);
			// auton::driveTurn(10, -90, 0.7);
			// wingFrontLeft.set_value(true);
			// intakeMotorGroup.move_voltage(-12000);
			// auton::driveDistance(75);

			// // crosses under the middle hang
			// auton::driveDistance(-10);
			// auton::absTurn(135);
			// intakeMotorGroup.move_voltage(0);
			// wingFrontLeft.set_value(false);
			// auton::driveTurn(56, 180, 0.25);
			// auton::driveDistance(55);

			// // rams into the far goal
			// auton::driveTurn(40, 90, 0.275);
			// auton::driveDistance(30);
			// auton::driveDistance(-7);
			// auton::absTurn(135);

			// // scores the first groupings
			// auton::driveDistance(24);
			// auton::driveTurn(36, -180, 0.3);
			// auton::absTurn(-45);
			// wingFrontRight.set_value(true);
			// auton::driveDistance(40);
			// auton::absTurn(-45);
			// auton::driveDistance(-10);
			// wingFrontRight.set_value(false);

			// // scores the second groupings
			// auton::turnAngle(-180);
			// wingFrontLeft.set_value(true);
			// auton::driveTurn(40, -180, 0.3);
			// wingFrontLeft.set_value(true);
			// wingFrontRight.set_value(true);
			// auton::absTurn(-45);
			// auton::driveDistance(40);



			// Skills (no front sweep):

			// pushes the triball into the goal
			cataMotorGroup.move_voltage(12000);
			auton::driveTurn(-24, 45, 0.25); //0.25
			cataMotorGroup.move_voltage(0);
			auton::absTurn(45);
			auton::driveDistance(-40);
			auton::driveDistance(5);

			// positions the robot and starts kicker
			// auton::driveDistance(11);
			// auton::turnAngle(-108);
			auton::driveTurn(10, -100, 0.6); //driveturn so it doesnt hit wall?
			auton::driveDistance(-20);
			wingBackRight.set_value(true);
			cataMotorGroup.move_voltage(12000);
			driveRight.move_voltage(-4000);
			driveLeft.move_voltage(-1000);
			// pros::delay(5000);
			delay(27000);
			cataMotorGroup.move_voltage(0); 
			driveRight.move_voltage(0);
			driveLeft.move_voltage(0);
			wingBackRight.set_value(false);

			// cross under right middle bar
            auton::driveDistance(4, 1000);
			auton::absTurn(0);
			auton::driveDistance(20, 1000);
			auton::driveTurn(24, -35, 0.3, 75, 1000);
			auton::driveDistance(44);

			// arc into the goal
			auton::driveTurn(48, -90, 0.225);
			auton::driveDistance(30);
			auton::driveDistance(-7);
			auton::absTurn(135);
			auton::driveDistance(-20);

			// first "curve"
			auton::driveDistance(32);
			auton::driveTurn(30, 135, 0.4, 75, 1000); // 26 , .25
			auton::absTurn(-45);
			//wingFrontLeft.set_value(true);
			auton::driveDistance(40);
			//wingFrontLeft.set_value(false);
			auton::absTurn(-45);
			// delay(1000);

			// reset position
			auton::driveTurn(-36, -90, 0.3);
			auton::absTurn(-135);

			// second "curve"
			auton::driveDistance(11);
			auton::driveTurn(20, 90, 0.2);
			auton::absTurn(-45); //-40
			wingFrontLeft.set_value(true);
			wingFrontRight.set_value(true);
			auton::driveDistance(36);
			auton::absTurn(-45);
			wingFrontLeft.set_value(false);
			wingFrontRight.set_value(false);
			delay(1000);

			// reset position
			auton::driveTurn(-30, -45, 0.3);
			auton::absTurn(-120); 

			// third "curve"
			auton::driveDistance(64);
			auton::absTurn(0);
			auton::driveTurn(40, 45, 0.25);
			auton::absTurn(45);
			wingFrontRight.set_value(true);
			auton::driveDistance(30);
			auton::driveDistance(-15);

			break;
		default:
			break;
	}
}

void skillsLineup() {
	cataMotorGroup.move_voltage(12000);
	auton::driveTurn(-24, 45, 0.25); //0.25
	cataMotorGroup.move_voltage(0);
	auton::absTurn(45);
	auton::driveDistance(-40);
	auton::driveDistance(5);

	// positions the robot and starts kicker
	// auton::driveDistance(11);
	// auton::turnAngle(-108);
	auton::driveTurn(10, -100, 0.6); //driveturn so it doesnt hit wall?
	auton::driveDistance(-20);
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
		float rightInput = ((float)master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127) * ((float) 5/8);

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
		//intakeMotorGroup.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? -12000 : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 12000 : 0));
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			hang.set_value(true);
		}


		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			if(fullMotorGroup.get_brake_modes()[0] == motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE) {
				fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
			} else {
				fullMotorGroup.set_brake_modes(motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
			}
		}

	    	if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			hang.set_value(false);
		} 
	    
		if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intakeMotorGroup.move_voltage(-12000);
		}
		if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intakeMotorGroup.move_voltage(10000);
		}
		if(!master.get_digital(E_CONTROLLER_DIGITAL_R1) && !master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intakeMotorGroup.move_voltage(0);
		}

		// checks limit switch
		if (hangLimit.get_new_press()) {		// digitalRead() will return LOW if PRESSED and HIGH if RELEASED
			hang.set_value(false);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			skillsLineup();
		}	

		// if(master.get_digital(E_CONTROLLER_DIGITAL_LEFT)){
		// 	while(inertial.is_calibrating()){
		// 		pros::delay(10);
		// 	}
		// 	inertial.set_heading(0);
		// 	auton::turnAngle(90);
		// }



		wingFrontLeft.set_value(master.get_digital(E_CONTROLLER_DIGITAL_L2));
		wingFrontRight.set_value(master.get_digital(E_CONTROLLER_DIGITAL_R2));

		//wingBackLeft.set_value(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));
		wingBackRight.set_value(master.get_digital(E_CONTROLLER_DIGITAL_Y));



		// Trigger pistons
		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		// 	if(shiftKey) {
		// 		intakeMotorGroup.move_voltage(-12000);
		// 	} else {
		// 		intakeMotorGroup.move_voltage(12000);
		// 	}
		// } else {
		// 		intakeMotorGroup.move_voltage(0);
		// }

		//left wings with shift key
		// if(master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
		// 	if(shiftKey) {
		// 		wingBackLeft.set_value(true);
		// 	} else {
		// 		wingFrontLeft.set_value(true);
		// 	}
		// } else {
		// 	wingBackLeft.set_value(false);
		// 	wingFrontLeft.set_value(false);
		// }

		//right wings with shift key
		// if(master.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		// 	if(shiftKey) {
		// 		wingBackRight.set_value(true);
		// 	} else {
		// 		wingFrontRight.set_value(true);
		// 	}
		// } else {
		// 	wingBackRight.set_value(false);
		// 	wingFrontRight.set_value(false);
		// }

		// Catapult control
		
		cataMotorGroup.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ? 12000 : 0);

		driveLeft.move_voltage(arcadeControl()[0] * 12000);
		driveRight.move_voltage(arcadeControl()[1] * 12000);

		//printf("%f", driveLeftBack.get_position() / 360 * M_PI * 2.75 * 0.75);

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
