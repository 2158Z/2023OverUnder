#include "main.h"
#include "selector.h"
// #include "api.h"
// #include "okapi/api.hpp"

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
			auton();
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
			// code block
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
	//controller initiation
    pros::Controller master(pros::E_CONTROLLER_MASTER);


    //left side motor group
    pros::Motor left1(8, true);
    pros::Motor left2(9, false);
    pros::Motor left3(10, true);
    pros::Motor_Group left ({left1, left2, left3});


    //right side motor group
    pros::Motor right1(1, false);
    pros::Motor right2(2, true);
    pros::Motor right3(3, false);
    pros::Motor_Group right ({right1, right2, right3});


    left.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    right.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);


    while(true) {

        //naturalize input to a range between -1 and 1
        double leftInput = (double)master.get_analog(ANALOG_LEFT_Y) / 127;
        double rightInput = (double)master.get_analog(ANALOG_RIGHT_X) / 127;


        //joystick curve
        leftInput = pow(leftInput, 3); //cubic function passthrough
        rightInput = pow(rightInput, 3) * (3/3); //amplitude change for turning


        //arcade control + joystick input to voltage conversion
        double leftVoltage = (leftInput + rightInput) * 12000;
        double rightVoltage = (leftInput - rightInput) * 12000;


        //limit max voltage to 12000mV i dont think this is actually needed
        if(leftVoltage > 12000) {
            leftVoltage = 12000;
        }
        if(rightVoltage > 12000) {
            rightVoltage = 12000;
        }


        //move motors
        left.move_voltage(leftVoltage);
        right.move_voltage(rightVoltage);


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
