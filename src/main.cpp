#include "main.h"
#include "selector.h"
#include "api.h"
#include "okapi/api.hpp"
#include "auton.h"
#include "util/odom.h"
#include <string>
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

Odom odom;
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
	
	pros::Task task{[=] {
		while(1){
			pros::delay(10);
			odom.update_position(odom.get_ForwardTracker_position((driveLeftMotorMiddle.get_position() + driveRightMotorMiddle.get_position())/2), 0, auton::get_absolute_heading());
			float x = odom.get_Xposition();
			float y = odom.get_Yposition();
			char str[1024];
			sprintf(str, "X: %s Y: %s", x, y);
			lv_label_set_text(selector::cordLabel, str);
	
		}
    }, "Odom Task"
	};

	odom.set_physical_distances(6.5, 6.5);
	odom.set_position(0, 0, auton::get_absolute_heading(), odom.get_ForwardTracker_position((driveLeftMotorMiddle.get_position() + driveRightMotorMiddle.get_position())/2), 0);

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
			auton::turn_to_angle(-55 + 180);
			auton::drive_distance(-62.2);
			auton::turn_to_angle(0+180);
			leftPiston.set_value(true);
			auton::turn_to_angle(133.4);
			auton::drive_distance(63/2);
			auton::turn_to_angle(133.4-90);
			auton::turn_to_angle(133.4-180);
			auton::turn_to_angle(133.4);
			leftPiston.set_value(false);
			auton::drive_distance(78/2);
			auton::turn_to_angle(90 + 180);
			auton::drive_distance(-50.3);
			// auton::drive_distance(-55.3);
			// auton::turn_to_angle(-90);
			break;
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
		case 2: //Close Side Elim
			auton::turn_to_angle(-55 + 180);
			auton::drive_distance(-62.2);
			auton::turn_to_angle(0+180);
			auton::drive_distance(-50.0);
			auton::drive_distance(14);
			auton::turn_to_angle(90.0);
			auton::drive_distance(55.9);
			auton::turn_to_angle(0.0);
			auton::drive_distance(81.3);
			auton::turn_to_angle(90.0+180);
			leftPiston.set_value(true);
			auton::drive_distance(-71.1);
			leftPiston.set_value(false);
			break;
		case 3:
			break;
		case -1: //  Farside
			auton::turn_to_angle(-180);
			auton::drive_distance(-56.6,8000);
			delay(250);
			auton::turn_to_angle(-70.6);
			intakeMotor.move_voltage(-10000);
			auton::drive_distance(61.4);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(21.0);
			auton::drive_distance(20);
			auton::turn_to_angle(90.0);
			intakeMotor.move_voltage(10000);
			delay(500);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(270.0);
			auton::setWingState(rightPiston,leftPiston,true);
			auton::drive_distance(-76.2);
			auton::setWingState(rightPiston, leftPiston, false);
			auton::turn_to_angle(-62.2);
			auton::drive_distance(54.5,8000);
			intakeMotor.move_voltage(-10000);
			auton::turn_to_angle(-90.0);
			auton::drive_distance(35.6);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(90.0);
			intakeMotor.move_voltage(12000);
			delay(500);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(270);
			auton::setWingState(rightPiston,leftPiston,true);
			auton::drive_distance(-80.8);
			auton::setWingState(rightPiston, leftPiston, false);
			break;
		case -2:
			auton::turn_to_angle(90.0);
			auton::drive_distance(8.9);
			auton::turn_to_angle(47.1);
			leftPiston.set_value(true);
			auton::drive_distance(48.5);
			leftPiston.set_value(false);
			auton::turn_to_angle(0.0);
			intakeMotor.move_voltage(10000);
			delay(1000);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(180.0);
			rightPiston.set_value(true);
			auton::drive_distance(-55.9);
			rightPiston.set_value(false);
			auton::drive_distance(25.4);
			auton::turn_to_angle(-90.0);
			auton::drive_distance(64.0);
			auton::turn_to_angle(0.0);
			auton::drive_distance(30.5);
			auton::turn_to_angle(-90.0);
			intakeMotor.move_voltage(-9000);
			auton::drive_distance(30.5);
			intakeMotor.move_voltage(0);
			auton::turn_to_angle(-12.6);
			auton::drive_distance(20.7);
			auton::turn_to_angle(90.0);
			intakeMotor.move_voltage(9000);
			delay(500);
			auton::turn_to_angle(-90);
			leftPiston.set_value(true);
			auton::drive_distance(-88.9);
			intakeMotor.move_voltage(0);
			leftPiston.set_value(false);
			break;
		case -3:
			cata.move_voltage(9000);
			auton::drive_with_voltage(1000, 1000);
			break;
		case 0:
			auton::turn_to_angle(-55 + 180);
			auton::drive_distance(-62.2);
			auton::turn_to_angle(0+180);
			auton::drive_distance(-50.0);
			auton::drive_distance(35);
			auton::turn_to_angle(180+75);
			auton::drive_distance(-28);
			auton::setWingState(rightPiston, leftPiston, true);
			cata.move_voltage(11000);
			delay(45000);
			auton::setWingState(rightPiston, leftPiston, false);
			auton::turn_to_angle(90.0);
			auton::drive_distance(65.4);
			auton::turn_to_angle(180.0);
			auton::drive_distance(45.0);
			auton::turn_to_angle(90);
			auton::drive_distance(185.4);
			auton::turn_to_angle(0.0+180);
			auton::setWingState(rightPiston, leftPiston, true);
			auton::drive_distance(-63.5);
			auton::setWingState(rightPiston, leftPiston, false);
			auton::turn_to_angle(-94.6);
			auton::drive_distance(63.7);
			auton::turn_to_angle(180+37.4);
			auton::setWingState(rightPiston, leftPiston, true);
			auton::drive_distance(-54.4);
			auton::setWingState(rightPiston, leftPiston, false);
			auton::turn_to_angle(-82.4);
			auton::drive_distance(38.4);
			auton::turn_to_angle(-3.2);
			auton::drive_distance(45.8);
			auton::turn_to_angle(180+90.4);
			auton::setWingState(rightPiston, leftPiston, true);
			auton::drive_distance(-88.9);
			auton::setWingState(rightPiston, leftPiston, false);
			auton::turn_to_angle(-90.0);
			auton::drive_distance(78.7);
			auton::turn_to_angle(-8.1);
			auton::drive_distance(71.8);
			auton::turn_to_angle(180+115.9);
			auton::setWingState(rightPiston, leftPiston, true);
			auton::drive_distance(-98.8);
			auton::setWingState(rightPiston, leftPiston, false);
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
		intakeMotor.move_voltage(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ? -10000 : (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ? 10000 : 0));

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
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			cata.move_voltage(11000);
		} else {
			cata.move_voltage(0);
		}
	

		// Lower catapult
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) lowerCata(cata, cataSwitch, 10000);


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