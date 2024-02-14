#include "main.h"
#include <vector>
using namespace okapi;
namespace auton{
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

    QLength meter(1.0); // SI base unit
    QLength decimeter = meter / 10;
    QLength centimeter = meter / 100;
    QLength millimeter = meter / 1000;
    QLength kilometer = 1000 * meter;
    QLength inch = 2.54 * centimeter;
    QLength foot = 12 * inch;
    QLength yard = 3 * foot;
    QLength mile = 5280 * foot;
    QLength tile = 24 * inch;
    QAngle radian(1.0);
    QAngle degree = static_cast<double>(2_pi / 360.0) * radian;

    // 0-Max Voltage, 1-KP, 2-KI, 3-KD, 4-startI, 5-settle time, 6-settle error, 7-timeout
    std::vector<float> turnConstants = {12000, 0.35, 0.25, 2, 0, 100, 1, 1500};
    std::vector<float> driveConstants = {12000, 1.5125, 0, 0, 0, 100, 0.5, 1500};

    float wheel_diameter = 3;
    float wheel_ratio = 0.75;

    float drive_desired_heading = 0;

    int counter = 0;

    void progSkills(pros::Motor cata){
        cata.set_brake_mode(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
        cata.move_voltage(11000);
    }

    void wings(pros::ADIDigitalOut wings, int time){
        wings.set_value(true);
        pros::delay(time);
        wings.set_value(false);
    }

    void setWingState(pros::ADIDigitalOut Rwings, pros::ADIDigitalOut Lwings, bool state){
        Rwings.set_value(state);
        Lwings.set_value(state);
    }

    void intake(pros::Motor intake, int time){
        intake.move_voltage(8000);
        pros::delay(time);
        intake.move_voltage(0);
    }

    float get_absolute_heading(){ 
        if (counter % 100 == 0){
            printf("%f Inertial\n", inertial.get_heading());
        }
        return inertial.get_heading(); 
    }

    float get_left_position_in(){
        return( (driveLeftFront.get_position()/360) * M_PI * wheel_diameter * wheel_ratio);
    }

    float get_right_position_in(){
        return( (driveRightFront.get_position()/360) * M_PI * wheel_diameter * wheel_ratio);
    }

    void driveVoltage(float leftVoltage, float rightVoltage){
        driveLeft.move_voltage(leftVoltage);
        driveRight.move_voltage(rightVoltage);
    }

    void driveDistance(float distance, std::vector<float> dConstants = driveConstants) {
        PID leftPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], dConstants[7]);
        PID rightPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], dConstants[7]);
        //PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], dConstants[7]);
        driveRightFront.tare_position();
        driveLeftFront.tare_position();

        while(!leftPID.is_settled() && !rightPID.is_settled()) {
            float leftTraveled = driveRightFront.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio; 
            float rightTraveled = driveRightFront.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio;
            
            float leftError = distance - leftTraveled;
            float rightError = distance - rightTraveled;

            float leftOutput = leftPID.compute(leftError) * dConstants[0];
            float rightOutput = rightPID.compute(rightError) * dConstants[0];

            driveVoltage(leftOutput, rightOutput);

            delay(10);
        }

        //float start_average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
        //float average_position = start_average_position;
        // while (!drivePID.is_settled()) {
        //     average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
        //     float drive_error = distance - average_position;
        //     float drive_output = drivePID.compute(drive_error) * 1000;
        //     drive_output = clamp(drive_output, -dConstants[0], dConstants[0]);
        //     driveVoltage(drive_output, drive_output);
        //     printf("Position: %f \n", average_position);
        //     //printf("Left: %f, Right: %f \n", get_left_position_in(), get_right_position_in());
        //     delay(100);
        // }
        //driveLeft.move_voltage(0);
        //driveRight.move_voltage(0);
        driveVoltage(0,0);
    }



    void turnAngle(float angle, std::vector<float> tConstants = turnConstants) {
        drive_desired_heading = angle;
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);
        while (turnPID.is_settled() == false) {
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 1000;
            printf("%f \n", error);
            output = clamp(output, -tConstants[0], tConstants[0]);
            driveVoltage(output, -output);
            delay(10);
        }
        driveLeft.move_voltage(0);
        driveRight.move_voltage(0);
    }

    void driveTurn(float distance, float angle, float turnWeight, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants) {

        driveLeftMiddle.tare_position();
        driveRightMiddle.tare_position();

        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);
        while(!turnPID.is_settled() || !drivePID.is_settled()){
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 1000;
            //printf("%f \n", error);
            output = clamp(output, -tConstants[0], tConstants[0]);
            
            average_position = (get_left_position_in()+get_right_position_in())/2.0;
            float drive_error = distance+start_average_position-average_position;
            float drive_output = drivePID.compute(drive_error) * dConstants[0];
            drive_output = clamp(drive_output, -dConstants[0], dConstants[0]);
            driveVoltage(((2 * turnWeight * output) + (2 * (1 - turnWeight) * drive_output)) / 2.0,
            ((2 * turnWeight * -output) + (2 * (1 - turnWeight) * drive_output)) / 2.0);
            // counter += 1;
            // if (counter % 10 == 0){
            //printf("%f \n", drive_error);
            // }
            delay(10);
        }
        driveVoltage(0,0);
    }

    // void turnAngle(std::shared_ptr<okapi::ChassisController> chassis, double deg){
    //     inertial.reset();
    //     double target = inertial.get() + deg;
    //     deg = static_cast<double>(deg*2);
    //     chassis -> turnAngle(deg*degree);
    //     double curr = inertial.get();
    //     double offset = (abs(curr) - target)*2;
    //     chassis -> turnAngle(-offset*degree);
    // }

    void setDefaultDriveConstants(std::vector<float> constants){
        driveConstants = constants;

    }

}
