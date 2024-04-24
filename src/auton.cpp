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
    //pros::Motor intakeMotor(intakeMotorID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
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

    //                                  0-Max Voltage, 1-KP, 2-KI, 3-KD, 4-startI, 5-settle time, 6-settle error, 7-timeout
    std::vector<float> driveConstants = {12000, 0.17, 0.0005, 1, 2, 75, 0.25, 1000}; //1.25
    std::vector<float> turnConstants = {12000, 0.015, 0.00, 0.103, 2, 75, 0.75, 1000};    //.0075

    float wheel_diameter = 3.25;
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

    void driveDistance(float distance, float timeout, std::vector<float> dConstants = driveConstants) {
        PID leftPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);
        PID rightPID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], timeout);

        driveLeftBack.tare_position();
        driveRightBack.tare_position();
        int counter = 0;
        // float lastRightOutput = 0;
        // float lastLeftOutput = 0;
        // float lastRightAddition = 0;
        // float lastLeftAddition = 0;

        while(!leftPID.is_settled() || !rightPID.is_settled()) {
            float leftTraveled = driveLeftBack.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio; 
            float rightTraveled = driveRightBack.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio;
            
            float rightError = distance - rightTraveled;
            float leftError = distance - leftTraveled;
            

            float leftOutput = leftPID.compute(leftError) * 10000;
            float rightOutput = rightPID.compute(rightError) * 10000;

            leftOutput = clamp(leftOutput, -driveConstants[0], driveConstants[0]);
            rightOutput = clamp(rightOutput, -driveConstants[0], driveConstants[0]);

            // if(rightOutput > (lastRightOutput + lastRightAddition)) {
            //     rightOutput = lastRightOutput + lastRightAddition;
            //     lastRightAddition += 100;
            // }
            // if(leftOutput > lastLeftOutput + lastLeftAddition) {
            //     leftOutput = lastLeftOutput + lastLeftAddition;
            //     lastLeftAddition += 100;
            // }

            driveVoltage(leftOutput, rightOutput); 

            // lastRightOutput = rightOutput;
            // lastLeftOutput = leftOutput;

            printf("%f %f %f %f \n", leftError, rightError, leftOutput, rightOutput);
            counter++;
            delay(10);
        }
        driveVoltage(0,0);
        printf("%s", "settled");
    }



    void turnAngle(float angle, std::vector<float> tConstants = turnConstants) {        // relative
        PID turnPID(angle, tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
        float relativeHeading = 0;
        float absHeading = inertial.get_heading();
        float previousAbsHeading = absHeading;
        while (!turnPID.is_settled()) {
            float deltaAngle = 0;
            absHeading = inertial.get_heading();

            deltaAngle = absHeading - previousAbsHeading;
            if(deltaAngle < -180 || deltaAngle > 180) { //if it crosses from 0 to 360 or vice versa
                deltaAngle = -360 + absHeading + previousAbsHeading;
            }

            relativeHeading += deltaAngle;
            float error = angle - relativeHeading;
            previousAbsHeading = absHeading;

            float output = turnPID.compute(error) * 10000;


            output = clamp(output, -tConstants[0], tConstants[0]);
            driveVoltage(output, -output);
            printf("%f %f\n", error, output);
            delay(10);
        }
        printf("%s", "settled");
    }

    void absTurn(float angle, std::vector<float> tConstants = turnConstants) {       // absolute turning
        PID turnPID(reduce_negative_180_to_180(angle - inertial.get_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
        drive_desired_heading = angle;
        int counter = 0;
        while(turnPID.is_settled() == false)    {
            float error = reduce_negative_180_to_180(angle - inertial.get_heading());
            float output = turnPID.compute(error) * 10000;
            output = clamp(output, -tConstants[0], tConstants[0]);
            printf("%d %f \n", counter, error);
            driveVoltage(output, -output);
            delay(10);
            counter++;
        }
        driveVoltage(0, 0);
    }

    void driveTurn(float distance, float angle, float turnWeight, float settleTime, float timeout, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants) {
        driveLeftFront.tare_position();
        driveRightFront.tare_position();

        float relativeHeading = 0;
        float absHeading = inertial.get_heading();
        float previousAbsHeading = absHeading;

        PID distancePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], settleTime, dConstants[6], timeout);
        PID turnPID(angle, tConstants[1], tConstants[2], tConstants[3], tConstants[4], settleTime, tConstants[6], timeout);

        while(!distancePID.is_settled() || !turnPID.is_settled()){
            //drive pid
            float distanceTraveled = ((driveLeftFront.get_position() + driveRightFront.get_position()) / 2) / 360 * M_PI * wheel_diameter * wheel_ratio; 
            float driveError = distance - distanceTraveled;
            float driveOutput = distancePID.compute(driveError) * 12000;

            //turn pid
            float deltaAngle = 0;
            absHeading = inertial.get_heading();

            deltaAngle = absHeading - previousAbsHeading;

            if(deltaAngle < -180 || deltaAngle > 180) { //if it crosses from 0 to 360 or vice versa
                deltaAngle = -360 + absHeading + previousAbsHeading;
            }

            relativeHeading += deltaAngle;
            float error = angle - relativeHeading;
            previousAbsHeading = absHeading;

            float turnOutput = turnPID.compute(error) * 10000;

            //combine
            turnOutput = clamp(turnOutput, -(12000 * turnWeight), (12000 * turnWeight));
            driveOutput = clamp(driveOutput, -(12000 - (12000 * turnWeight)), (12000 - (12000 * turnWeight)));
            // printf("%f %f \n", error, turnOutput);
            driveVoltage((driveOutput + turnOutput), (driveOutput - turnOutput));
            printf("%f %f \n", (2 * turnWeight * turnOutput), (2 * (1 - turnWeight) * driveOutput));
            delay(10);
        }
        driveVoltage(0,0);
    }

    void absDriveTurn(float distance, float angle, float turnWeight, float settleTime, float timeout, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants) {
        
        driveLeftFront.tare_position();
        driveRightFront.tare_position();

        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], dConstants[7]);
        PID turnPID(reduce_negative_180_to_180(angle - inertial.get_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
        while(!turnPID.is_settled() || !drivePID.is_settled()){
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float turnOutput = turnPID.compute(error) * 1000;

            turnOutput = clamp(turnOutput, -(12000 * turnWeight), (12000 * turnWeight));

            average_position = (get_left_position_in()+get_right_position_in())/2.0;
            float drive_error = distance+start_average_position-average_position;
            float driveOutput = drivePID.compute(drive_error) * dConstants[0];
            driveOutput = clamp(driveOutput, -(12000 - (12000 * turnWeight)), (12000 - (12000 * turnWeight)));
            driveVoltage((driveOutput + turnOutput), (driveOutput - turnOutput));
            printf("%f %f \n", (2 * turnWeight * turnOutput), (2 * (1 - turnWeight) * driveOutput));
            delay(10);
        }
    }

    void setDefaultDriveConstants(std::vector<float> constants){
        driveConstants = constants;

    }

}