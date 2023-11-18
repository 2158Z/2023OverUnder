#include "main.h"
using namespace okapi;
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

    pros::IMU inertial(4);
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

    float drive_turn_max_voltage = 12000;
    float drive_turn_kp = 0.35;
    float drive_turn_ki = 0.25;    
    float drive_turn_kd = 2;
    float drive_turn_starti = 0;

    float drive_turn_settle_error = 1;
    float drive_turn_settle_time = 600;
    float drive_turn_timeout = 1000;

    float drive_drive_max_voltage = 8000;
    float drive_drive_kp = 1.5;
    float drive_drive_ki = 0;
    float drive_drive_kd = .85;
    float drive_drive_starti = 0;

    float drive_drive_settle_error = 1.5;
    float drive_drive_settle_time = 750;
    float drive_drive_timeout = 2000;

    float drive_heading_max_voltage = 12000;
    float drive_heading_kp = 0;
    float drive_heading_ki = 0;
    float drive_heading_kd = 0;
    float drive_heading_starti = 0;

    float drive_swing_max_voltage = 12000;
    float drive_swing_kp = 0;
    float drive_swing_ki = 0;
    float drive_swing_kd = 0;
    float drive_swing_starti = 0;

    float drive_swing_settle_error = 1;
    float drive_swing_settle_time = 300;
    float drive_swing_timeout = 3000;

    float drive_desired_heading = 0;

    int counter = 0;

    void progSkills(pros::Motor cata){
        cata.set_brake_mode(motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
        cata.move_voltage(9000);
        
    }

    void wings(pros::ADIDigitalOut wings, int time){
        wings.set_value(true);
        pros::delay(time);
        wings.set_value(false);
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

    void drive_distance(float distance, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti){
        distance = distance /  2.54; //Conversion from Centimeter to Inch
        PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_time, drive_settle_error, drive_settle_time);
        driveLeftMotorMiddle.tare_position();
        driveRightMotorMiddle.tare_position();
        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        while(drivePID.is_settled() == false){
            average_position = (get_left_position_in()+get_right_position_in())/2.0;
            float drive_error = distance+start_average_position-average_position;
            float drive_output = drivePID.compute(drive_error) * 1000;
            drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
            drive_with_voltage(drive_output, drive_output);
            // counter += 1;
            // if (counter % 10 == 0){
                printf("%f \n", drive_error);
            // }
            delay(10);
        }
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
    }

    void turn_to_angle(float angle, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti){
        drive_desired_heading = angle;
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_time, turn_settle_error, turn_timeout);
        while(turnPID.is_settled() == false){
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 1000;
            printf("%f \n", error);
            output = clamp(output, -turn_max_voltage, turn_max_voltage);
            drive_with_voltage(output, -output);
            delay(10);
        }
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
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
