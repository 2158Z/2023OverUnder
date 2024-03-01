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
pros::ADIDigitalOut highHangLeft(highHangLeftID);
pros::ADIDigitalOut highHangRight(highHangRightID);

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
	while(inertial.is_calibrating()){
		pros::delay(10);
	}
	switch(selector::auton) {
		case 1:

			// auton::driveDistance(5);
			// auton::turnAngle(90);
			// auton::driveDistance(5);
			// auton::turnAngle(-90);
			
			intakeMotor.move_voltage(12000);
				// auton::driveTurn(-80, 45, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.3, 1000});
			auton::driveTurn(-20, 45, 0.2, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.3, 1000});
		    auton::driveDistance(-15, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 500});
			auton::driveDistance(5);
			auton::driveDistance(-15);
			intakeMotor.move_voltage(0);
			auton::driveDistance(5);
			auton::turnAngle(180);
			auton::driveTurn(-8, -45, 0.25, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0.0021, 0.095, 2, 100, 0.3, 1000});
			wingBackLeft.set_value(true);
			auton::driveTurn(-12, -45, 0.25, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0.0021, 0.095, 2, 100, 0.3, 1000});
			wingBackLeft.set_value(false);
			pros::delay(250);
			auton::driveTurn(-10, -15, 0.5);
			auton::turnAngle(-192.5);
			intakeMotor.move_voltage(-12000);
			auton::driveDistance(36, {8000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});

			// auton::driveDistance(-22);
			// auton::turnAngle(35);
			// auton::driveDistance(-15.5);
			// auton::driveTurn(14, 10, 0.5);
			// auton::turnAngle(211);
			// wingBackLeft.set_value(true);
			// auton::driveTurn(-13.5, 180, 0.45); // 13.5 -> 14
			// // auton::turnAngle(200);
			// wingBackLeft.set_value(false);
			// pros::delay(250);
			// auton::turnAngle(-21.5); // 40
			// intakeMotor.move_voltage(-12000);
			// auton::driveDistance(33.5);
			// pros::delay(200);
			// auton::driveVoltage(0,0);
			break;
		case 2: //Close Side Elim
			intakeMotor.move_voltage(12000);
			wingFrontLeft.set_value(true);
			pros::delay(250);
			wingFrontLeft.set_value(false);
			auton::driveTurn(55, -45, 0.2,{12000, 0.15, 0.01, 0.9, 1, 0, 0.25, 3000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 3000});
			auton::turnAngle(85, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			wingFrontLeft.set_value(true);
			intakeMotor.move_voltage(-12000);
			delay(150);
			auton::driveDistance(24);
			wingFrontLeft.set_value(false);
			intakeMotor.move_voltage(0);
			auton::turnAngle(-44, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			auton::driveDistance(-55);
			auton::turnAngle(-245, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			auton::driveDistance(-50);
			auton::driveDistance(6);
			auton::turnAngle(180, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			auton::driveTurn(-8, -45, 0.25, {12000, 0.15, 0.01, 0.9, 1, 0, 0.25, 3000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			wingBackLeft.set_value(true);
			auton::driveTurn(-14, -45, 0.25, {12000, 0.15, 0.01, 0.9, 1, 0, 0.25, 3000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			wingBackLeft.set_value(false);
			pros::delay(250);
			auton::driveTurn(-10, -15, 0.5, {12000, 0.15, 0.01, 0.9, 1, 0, 0.25, 3000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::turnAngle(-185, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			intakeMotor.move_voltage(-12000);
			auton::driveDistance(36);
			break;
		case 3:
			break;
		case -1: //  Farside-0.
			wingBackLeft.set_value(true);
			intakeMotor.move_voltage(12000);
			auton::driveTurn(-15, -45, 0.45, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 100, 0.5, 1000});
			intakeMotor.move_voltage(0);
			//auton::turnAngle(-50);
			wingBackLeft.set_value(false);
			//auton::driveDistance(-24);
			// auton::driveTurn(-10, -45, 0.5, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.5, 1000}); // changing driveturn
			auton::driveDistance(-3);
			auton::driveTurn(-5, -45, 0.4, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.5, 1000});
			auton::driveDistance(-20, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 500});;
			intakeMotor.move_voltage(12000);
			//auton::driveTurn(64.5, 105, 0.45, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.25, 3000});
			auton::driveTurn(30, 105, 0.7, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.25, 1000});
			auton::driveDistance(30);
			//intakeMotor.move_voltage(0);
			auton::turnAngle(115);
			intakeMotor.move_voltage(-12000);
			auton::driveDistance(10);
			intakeMotor.move_voltage(0);
			auton::turnAngle(-75);
			intakeMotor.move_voltage(12000);
			auton::driveDistance(15);
			// auton::turnAngle(112.5); 
			auton::absTurn(180); //test
			intakeMotor.move_voltage(-12000);
			wingFrontLeft.set_value(true);
			wingFrontRight.set_value(true);
			auton::driveDistance(54, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 1500});
			break;
		case -2:

			//saved kenny far safe
			intakeMotor.move_voltage(12000);
			auton::driveTurn(-18, -35, 0.35);
			intakeMotor.move_voltage(0);
			wingBackLeft.set_value(true);
			pros::delay(500);
			auton::turnAngle(-50);
			wingBackLeft.set_value(false);
			auton::driveDistance(-24);
			intakeMotor.move_voltage(12000);
			auton::driveTurn(64.5, 105, 0.45, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 3000}, {12000, 0.015, 0.0021, 0.095, 2, 150, 0.25, 3000});
			auton::turnAngle(115);
			intakeMotor.move_voltage(-12000);
			auton::driveDistance(10);
			intakeMotor.move_voltage(0);
			auton::turnAngle(-75);
			intakeMotor.move_voltage(12000);
			auton::driveDistance(20);
			// auton::turnAngle(112.5);
			auton::absTurn(180);
			intakeMotor.move_voltage(-12000);
			wingFrontLeft.set_value(true);
			wingFrontRight.set_value(true);
			auton::driveDistance(54);


			// auton::setDefaultDriveConstants({11000, 0.1, 0, 0, 0, 0, 0.1, 500});
			// intakeMotor.move_voltage(9000);
			// wingBackLeft.set_value(true);
			// auton::driveTurn(-18, 315, 0.35);
			// wingBackLeft.set_value(false);
			// auton::driveTurn(-18, 270, 0.35);
			// intakeMotor.move_voltage(0);
			// wingBackLeft.set_value(false);
			// auton::driveDistance(16);
			// auton::turnAngle(81);
			// intakeMotor.move_voltage(12000);
			// auton::driveDistance(60, {10000, 1.5125, 0, 0, 0, 100, 0.5, 1500});
			// auton::turnAngle(99);
			// wingFrontLeft.set_value(true);
			// auton::driveDistance(36, {12000, 0.1, 0, 0, 0, 0, 0.5, 1500});
			// intakeMotor.move_voltage(-12000);
			// pros::delay(150);
			// intakeMotor.move_voltage(0);
			// wingFrontLeft.set_value(false);
			// auton::driveDistance(-5);
			// auton::turnAngle(135);
			// intakeMotor.move_voltage(12000);
			// auton::driveDistance(25);
			// pros::delay(150);
			// auton::turnAngle(-180);
			// auton::driveDistance(25);
			// auton::turnAngle(45);
			// intakeMotor.move_voltage(-12000);
			// auton::driveDistance(20);
			// intakeMotor.move_voltage(0);
			// auton::driveDistance(-5);
			break;
		case -3:
			
			break;
		case 0:
			// Skills (with front sweeps):

			// // pushes the triball into the goal
			// intakeMotor.move_voltage(12000);
			// auton::driveTurn(-80, 45, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			// intakeMotor.move_voltage(0);

			// // positions the robot and starts kicker
			// auton::driveDistance(10);
			// auton::turnAngle(-115);
			// auton::driveDistance(-1.5);
			// wingBackRight.set_value(true);
			// cataMotorGroup.move_voltage(11000);
			// fullMotorGroup.move_voltage(250);
			// cataMotorGroup.move_voltage(0);
			// wingBackRight.set_value(false);

			// // goes to middle middle zone an picks up right 
			// intakeMotor.move_voltage(12000);
			// auton::driveTurn(29, 45, 0.2625, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(22);

			// // pushes the other middle balls
			// auton::driveDistance(-5);
			// intakeMotor.move_voltage(0);
			// auton::driveTurn(10, -90, 0.7, {12000, 0.15, 0.01, 0.9, 1, 100, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			// wingFrontLeft.set_value(true);
			// intakeMotor.move_voltage(-12000);
			// auton::driveDistance(75);

			// // crosses under the middle hang
			// auton::driveDistance(-10, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 500});
			// auton::absTurn(135);
			// intakeMotor.move_voltage(0);
			// wingFrontLeft.set_value(false);
			// auton::driveTurn(56, 180, 0.25, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(55);

			// // rams into the far goal
			// auton::driveTurn(40, 90, 0.275, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(30);
			// auton::driveDistance(-7, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});
			// auton::absTurn(135);

			// // scores the first groupings
			// auton::driveDistance(24);
			// auton::driveTurn(36, -180, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			// auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			// wingFrontRight.set_value(true);
			// auton::driveDistance(40, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});
			// auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(-10, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});
			// wingFrontRight.set_value(false);

			// // scores the second groupings
			// auton::turnAngle(-180, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			// wingFrontLeft.set_value(true);
			// auton::driveTurn(40, -180, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// wingFrontLeft.set_value(true);
			// wingFrontRight.set_value(true);
			// auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// auton::driveDistance(40, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});



			// Skills (no front sweep):

			// pushes the triball into the goal
			intakeMotor.move_voltage(12000);
			auton::driveTurn(-24, 45, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});
			auton::absTurn(45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			auton::driveDistance(-20, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 1000});
			intakeMotor.move_voltage(0);

			// positions the robot and starts kicker
			auton::driveDistance(10);
			auton::turnAngle(-115);
			auton::driveDistance(-3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 1000});
			
			// wingBackRight.set_value(true);
			// cataMotorGroup.move_voltage(11000);
			// fullMotorGroup.move_voltage(250);
			// delay(30000);
			// cataMotorGroup.move_voltage(0);
			
			wingBackRight.set_value(false);
			delay(1000);
			auton::driveDistance(-3);

			// cross under right middle bar
			auton::absTurn(0);
			auton::driveTurn(60, -45, 0.25, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			auton::driveDistance(36);

			// arc into the goal
			auton::driveTurn(44, -90, 0.2, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			auton::driveDistance(30, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 700});
			auton::driveDistance(-12);
			auton::absTurn(135);
			auton::driveDistance(-20, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 200});

			// first "curve"
			auton::driveDistance(24);
			auton::driveTurn(36, 180, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			//wingFrontLeft.set_value(true);
			auton::driveDistance(40, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});
			//wingFrontLeft.set_value(false);
			auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			// delay(1000);

			// reset position
			auton::driveTurn(-36, -90, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::absTurn(-135, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 1000});

			// second "curve"
			auton::driveDistance(8);
			auton::driveTurn(20, 90, 0.25, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::absTurn(140, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500}); //-40
			wingFrontLeft.set_value(true);
			wingFrontRight.set_value(true);
			auton::driveDistance(40, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000});
			auton::absTurn(-45, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 500});
			wingFrontLeft.set_value(false);
			wingFrontRight.set_value(false);
			delay(1000);

			// reset position
			auton::driveTurn(-32, -45, 0.3, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			//auton::driveDistance(20);
			auton::absTurn(-100);

			// third "curve"
			auton::driveDistance(60);
			auton::absTurn(0);
			//auton::driveTurn(36, 45, 0.5, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			//auton::driveTurn(36, 45, 0.5, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::driveTurn(-40, -50, 0.5, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			//auton::absTurn(45);
			auton::driveTurn(55, 90, 0.15, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000}, {12000, 0.015, 0, 0.109, 2, 100, 0.3, 2000});
			auton::absTurn(45);
			auton::driveDistance(40, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 1000});
			auton::driveDistance(-15, {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 1000});

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
		if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			highHangLeft.set_value(true);
			highHangRight.set_value(true);
		}

	    	if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			highHangLeft.set_value(false);
			highHangRight.set_value(false);
		}
	    
		if(master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intakeMotor.move_voltage(-12000);
		}
		if(master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intakeMotor.move_voltage(12000);
		}
		if(!master.get_digital(E_CONTROLLER_DIGITAL_R1) && !master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
			intakeMotor.move_voltage(0);
		}


		wingFrontLeft.set_value(master.get_digital(E_CONTROLLER_DIGITAL_L2));
		wingFrontRight.set_value(master.get_digital(E_CONTROLLER_DIGITAL_R2));

		wingBackLeft.set_value(master.get_digital(E_CONTROLLER_DIGITAL_RIGHT));
		wingBackRight.set_value(master.get_digital(E_CONTROLLER_DIGITAL_Y));



		// Trigger pistons
		// if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		// 	if(shiftKey) {
		// 		intakeMotor.move_voltage(-12000);
		// 	} else {
		// 		intakeMotor.move_voltage(12000);
		// 	}
		// } else {
		// 		intakeMotor.move_voltage(0);
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
		cataMotorGroup.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ? 11000 : 0);

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
