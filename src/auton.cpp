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
    std::vector<float> driveConstants = {12000, 0.15, 0.01, 0.9, 1, 150, 0.25, 2000};
    std::vector<float> turnConstants = {12000, 0.015, 0.0021, 0.095, 2, 100, 0.3, 2000};

    float wheel_diameter = 2.75;
    float wheel_ratio = 0.75;

    float desired_heading = 0;

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

        driveRightFront.tare_position();
        driveLeftFront.tare_position();
        int counter = 0;

        while(!leftPID.is_settled() || !rightPID.is_settled()) {
            float leftTraveled = driveLeftFront.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio; 
            float rightTraveled = driveRightFront.get_position() / 360 * M_PI * wheel_diameter * wheel_ratio;
            
            float leftError = distance - leftTraveled;
            float rightError = distance - rightTraveled;

            float leftOutput = leftPID.compute(leftError) * 10000;
            float rightOutput = rightPID.compute(rightError) * 10000;

            leftOutput = clamp(leftOutput, -driveConstants[0], driveConstants[0]);
            rightOutput = clamp(rightOutput, -driveConstants[0], driveConstants[0]);

            driveVoltage(leftOutput, rightOutput);

            //printf("%d %f \n", counter, rightError);
            counter++;
            delay(10);
        }
        driveVoltage(0,0);
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
            printf("%f %f \n", relativeHeading, absHeading);

            output = clamp(output, -tConstants[0], tConstants[0]);
            driveVoltage(output, -output);
            delay(10);
        }
    }

    void absTurn(float angle, std::vector<float> tConstants = turnConstants) {       // absolute turning
        PID turnPID(reduce_negative_180_to_180(angle - inertial.get_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);
        desired_heading = angle;
        while(turnPID.is_settled() == false)    {
            float error = reduce_negative_180_to_180(angle - inertial.get_heading());
            float output = turnPID.compute(error) * 10000;
            output = clamp(output, -tConstants[0], tConstants[0]);
            driveVoltage(output, -output);
            delay(10);
        }
        driveVoltage(0, 0);
    }

    void driveTurn(float distance, float angle, float turnWeight, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants) {

        driveLeftFront.tare_position();
        driveRightFront.tare_position();

        float relativeHeading = 0;
        float absHeading = inertial.get_heading();
        float previousAbsHeading = absHeading;

        PID distancePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6], dConstants[7]);
        PID turnPID(angle, tConstants[1], tConstants[2], tConstants[3], tConstants[4], tConstants[5], tConstants[6], tConstants[7]);

        while(!distancePID.is_settled() || !turnPID.is_settled()){
            //drive pid


            float distanceTraveled = ((driveLeftFront.get_position() + driveRightFront.get_position()) / 2) / 360 * M_PI * wheel_diameter * wheel_ratio; 
            float driveError = distance - distanceTraveled;
            float driveOutput = distancePID.compute(driveError) * 10000;

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
            turnOutput = clamp(turnOutput, -12000, 12000);
            driveOutput = clamp(driveOutput, -12000, 12000);
            // printf("%f %f \n", error, turnOutput);
            driveVoltage(((2 * turnWeight * turnOutput) + (2 * (1 - turnWeight) * driveOutput)) / 2.0, ((2 * turnWeight * -turnOutput) + (2 * (1 - turnWeight) * driveOutput)) / 2.0);
            printf("%f %f \n", (2 * turnWeight * turnOutput), (2 * (1 - turnWeight) * driveOutput));
            delay(10);
        }
        driveVoltage(0,0);
    }

    void setDefaultDriveConstants(std::vector<float> constants){
        driveConstants = constants;
    }

// odom functions
    float get_ForwardTracker_position() {
        if (drive_setup == ZERO_TRACKER_ODOM){
            // For zero tracker odom, the right side of the drive becomes the tracker.
            return(get_right_position_in());
        }
        if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_TWO_ENCODER ||   drive_setup == HOLONOMIC_TWO_ENCODER){
            return(E_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
        } else{
            // This if-else just discriminates based on whether the sensor is an    encoder or rotation sensor.
            return(R_ForwardTracker.position(deg)*ForwardTracker_in_to_deg_ratio);
        }
    }

    float get_SidewaysTracker_position() {
        if (drive_setup == TANK_ONE_ENCODER || drive_setup == TANK_ONE_ROTATION ||  drive_setup == ZERO_TRACKER_ODOM){
            return(0);
            // These setups all pretend that there is a sideways tracker in the center that just never moves.
        } else if (drive_setup == TANK_TWO_ENCODER || drive_setup == HOLONOMIC_TWO_ENCODER){
            return(E_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
        } else{
            return(R_SidewaysTracker.position(deg)*SidewaysTracker_in_to_deg_ratio);
        }
    }

    void position_track() {
        while(1){
            odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
            delay(5);
        }
    }

    void set_heading(float orientation_deg) {
        inertial.set_rotation(orientation_deg);
    }

    void set_coordinates(float X_position, float Y_position, float orientation_deg) {
        odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
        set_heading(orientation_deg);
        odom_task = task(position_track_task);
    }

    float get_X_position(){
        return(odom.X_position);
    }

    float get_Y_position(){
        return(odom.Y_position);
    }

    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage){
        drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    }

    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
        drive_to_point(X_position, Y_position, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
    }

    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
        PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
        PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
        while(drivePID.is_settled() == false){
            float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
            // The drive error is just equal to the distance between the current and desired points.
            float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
            // This uses atan2(x,y) rather than atan2(y,x) because doing so places 0 degrees on the positive Y axis.
            float drive_output = drivePID.compute(drive_error);

            float heading_scale_factor = cos(to_rad(heading_error));
            drive_output*=heading_scale_factor;
            // The scale factor slows the drive down the more it's facing away from the desired point,
            // and that way the heading correction has time to catch up.
            heading_error = reduce_negative_90_to_90(heading_error);
            // Here we reduce -90 to 90 because this allows the robot to travel backwards if it's easier
            // to do so.
            float heading_output = headingPID.compute(heading_error);

            if (drive_error<drive_settle_error) { heading_output = 0; }
            // This if statement prevents the heading correction from acting up after the robot gets close
            // to being settled.

            drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
            heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

            driveVoltage(drive_output+heading_output, drive_output-heading_output);
            delay(10);
        }
        desired_heading = get_absolute_heading();
        driveVoltage(0, 0);
    }

    void turn_to_point(float X_position, float Y_position, float extra_angle_deg){
        turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
    }

    void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
        turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
    }

    void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
        PID turnPID(reduce_negative_180_to_180(to_rad(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
        while(turnPID.is_settled() == false) {
        float error = reduce_negative_180_to_180(to_rad(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
        // Again, using atan2(x,y) puts 0 degrees on the positive Y axis.
        float output = turnPID.compute(error);
        output = clamp(output, -turn_max_voltage, turn_max_voltage);
        driveVoltage(output, -output);
        delay(10);
        }
        desired_heading = get_absolute_heading();
        driveVoltage(0, 0);
    }

}