#include "main.h"
using namespace okapi;
using namespace std;
namespace auton{
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

    pros::IMU inertial(14);
    Odom odom;
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

    float wheel_diameter = 3.25;
    float wheel_ratio = 0.3;
    float gyro_scale = 1;
    float drive_in_to_deg_ratio = wheel_ratio/360.0*M_PI*wheel_diameter;
    float forwardTrackerCenterDistance = 0;
    float forwardTrackerDiameter = 0;
    float forwardTrackerInToDegRatio = M_PI*forwardTrackerDiameter/360.0;
    float SidewaysTracker_center_distance = 0;
    float sidewaysTrackerDiameter = 0;
    float sidewaysTrackerInToDegRatio = M_PI*sidewaysTrackerDiameter/360.0;

    // 0-Max Voltage, 1-KP, 2-KI, 3-KD, 4-startI, 5-settle time, 6-settle error, 7-timeout
    vector<float> turnConstants = {12000, 0.35, 0.25, 2, 0, 100, 1, 1500};
    vector<float> driveConstants = {11000, 1.5, 0, 0.85, 0, 100, 1.5, 1500};

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
        return inertial.get_heading(); 
    }

    float get_left_position_in(){
        if (counter % 100 == 0){
            //printf("%f Left\n", ( (driveLeftMotorMiddle.get_position()/180) * M_PI * wheel_diameter * wheel_ratio))/2;
        }
        return( (driveLeftMotorMiddle.get_position()/180) * M_PI * wheel_diameter * wheel_ratio);
    }

    float get_right_position_in(){
        if (counter % 100 == 0){
            //printf("%f Right\n", -1 * ( (driveRightMotorMiddle.get_position()/180) * M_PI * wheel_diameter * wheel_ratio)/2);
        }
        return -1 * ( (driveRightMotorMiddle.get_position()/180) * M_PI * wheel_diameter * wheel_ratio);
    }

    float get_X_position(){
        return(odom.X_position);
    }

    float get_Y_position(){
    return(odom.Y_position);
    }

    void position_track(){
        while(1){
            odom.update_position(-1 * get_right_position_in(), 0, get_absolute_heading());
            pros::delay(5);
        }
    }

    void drive_with_voltage(float leftVoltage, float rightVoltage){
        leftMotorGroup.move_voltage(leftVoltage);
        rightMotorGroup.move_voltage(-1 * rightVoltage);
    }

    void drive_distance(float distance, vector<float> dConstants = driveConstants) {
        // Convert distance from centimeters to inches
        distance = distance / 2.54;

        // Initialize PID controller for drive
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);

        // Reset the position of the middle motors
        driveLeftMotorMiddle.tare_position();
        driveRightMotorMiddle.tare_position();

        // Calculate the initial average position of the left and right motors
        float start_average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
        float average_position = start_average_position;

        // Continue driving until the drive PID controller is settled
        while (drivePID.is_settled() == false) {
            // Recalculate the average position of the left and right motors.
            average_position = (get_left_position_in() + get_right_position_in()) / 2.0;

            // Calculate the drive error based on the desired distance and the change in average position
            float drive_error = distance + start_average_position - average_position;

            // Compute PID output
            float drive_output = drivePID.compute(drive_error) * 1000;
            drive_output = clamp(drive_output, -dConstants[0], dConstants[0]);

            // Apply the calculated voltages for driving
            drive_with_voltage(drive_output, drive_output);

            // Print the drive error for debugging purposes
            printf("%f \n", drive_error);

            // Introduce a delay to control loop execution speed
            delay(10);
        }

        // Stop the left and right motor groups after the drive is complete
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
}

    void turn_to_angle(float angle, vector<float> tConstants = turnConstants) {
        // Set the desired heading for the turn.
        drive_desired_heading = angle;

        // Initialize PID for turning
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);

        // Continue turning until PID settles
        while (turnPID.is_settled() == false) {
            // Calculate the error between the target angle and the current heading.
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());

            // Compute PID output and scale
            float output = turnPID.compute(error) * 1000;

            // debugging
            printf("%f \n", error);

            // Force max speed
            output = clamp(output, -tConstants[0], tConstants[0]);

            // Apply the calculated voltages to motors for turning
            drive_with_voltage(output, -output);

            // Delay to control loop execution speed
            delay(10);
        }

        // Stop motorgroups on compeletion
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
    }

    void driveTurn(float distance, float angle, float turnWeight, vector<float> tConstants = turnConstants, vector<float> dConstants = driveConstants) {

        driveLeftMotorMiddle.tare_position();
        driveRightMotorMiddle.tare_position();

        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);
        while(turnPID.is_settled() == false || drivePID.is_settled() == false){
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 1000;
            //printf("%f \n", error);
            output = clamp(output, -tConstants[0], tConstants[0]);
            
            average_position = (get_left_position_in()+get_right_position_in())/2.0;
            float drive_error = distance+start_average_position-average_position;
            float drive_output = drivePID.compute(drive_error) * 1000;
            drive_output = clamp(drive_output, -dConstants[0], dConstants[0]);
            drive_with_voltage(((2 * turnWeight * output) + (2 * (1 - turnWeight) * drive_output)) / 2.0,
            ((2 * turnWeight * -output) + (2 * (1 - turnWeight) * drive_output)) / 2.0);
            // counter += 1;
            // if (counter % 10 == 0){
            //printf("%f \n", drive_error);
            // }
            delay(10);
        }

    }

    void driveToPoint(float X_position, float Y_position, vector<float> dConstants = driveConstants, vector<float> tConstants = turnConstants){
        PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);
        PID headingPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4]);
        while(drivePID.is_settled() == false){
            float drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
            float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
            float drive_output = drivePID.compute(drive_error);

            float heading_scale_factor = cos(to_rad(heading_error));
            drive_output*=heading_scale_factor;
            heading_error = reduce_negative_90_to_90(heading_error);
            float heading_output = headingPID.compute(heading_error);
            
            if (drive_error<dConstants[6]) { heading_output = 0; }

            drive_output = clamp(drive_output, -fabs(heading_scale_factor)*dConstants[0], fabs(heading_scale_factor)*dConstants[0]);
            heading_output = clamp(heading_output, -tConstants[0], tConstants[0]);

            drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
            pros::delay(10);
        }
        drive_desired_heading = get_absolute_heading();
        drive_with_voltage(0, 0);
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
}
