#include "main.h"
using namespace okapi;
namespace auton{
    okapi::Motor leftMotor1(18, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::Motor leftMotor2(19, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::Motor leftMotor3(20, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::MotorGroup leftMotorGroup ({leftMotor1, leftMotor2, leftMotor3});

    //right side motor group
    okapi::Motor rightMotor1(11, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::Motor rightMotor2(12, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::Motor rightMotor3(13, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
    okapi::MotorGroup rightMotorGroup ({rightMotor1, rightMotor2, rightMotor3});

    okapi::MotorGroup fullMotorGroup({rightMotor1, rightMotor2, rightMotor3, leftMotor1, leftMotor2, leftMotor3});

    okapi::IMU inertial(1);
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

    float wheel_diameter = 3.125;
    float wheel_ratio = 0.6;
    float gyro_scale = 1;
    float drive_in_to_deg_ratio = wheel_ratio/360.0*M_PI*wheel_diameter;
    float ForwardTracker_center_distance(ForwardTracker_center_distance);
    float ForwardTracker_diameter = 0;
    float ForwardTracker_in_to_deg_ratio = M_PI*ForwardTracker_diameter/360.0;
    float SidewaysTracker_center_distance = 0;
    float SidewaysTracker_diameter = 0;
    float SidewaysTracker_in_to_deg_ratio = M_PI*SidewaysTracker_diameter/360.0;

    float drive_turn_max_voltage = 12000;
    float drive_turn_kp = 0;
    float drive_turn_ki = 0;    
    float drive_turn_kd = 0;
    float drive_turn_starti = 0;

    float drive_turn_settle_error = 1;
    float drive_turn_settle_time = 300;
    float drive_turn_timeout = 5000;

    float drive_drive_max_voltage = 12000;
    float drive_drive_kp = 1;
    float drive_drive_ki = 0;
    float drive_drive_kd = 0;
    float drive_drive_starti = 0;

    float drive_drive_settle_error = 1.5;
    float drive_drive_settle_time = 300;
    float drive_drive_timeout = 5000;

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

    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata){
        cata.setBrakeMode(AbstractMotor::brakeMode::hold);
        cata.moveVoltage(9000);
        
    }

    void wings(pros::ADIDigitalOut wings, int time){
        wings.set_value(true);
        pros::delay(time);
        wings.set_value(false);
    }
    void intake(okapi::Motor intake, int time){
        intake.moveVoltage(8000);
        pros::delay(time);
        intake.moveVoltage(0);
    }

    float get_absolute_heading(){ 
        return reduce_0_to_360(inertial.get()); 
    }

    float get_left_position_in(){
        return( leftMotor2.getPosition()*drive_in_to_deg_ratio );
        }

    float get_right_position_in(){
        return( rightMotor2.getPosition()*drive_in_to_deg_ratio );
    }

    float get_X_position(){
        return(odom.X_position);
    }

    float get_Y_position(){
    return(odom.Y_position);
    }

    void position_track(){
        while(1){
            odom.update_position(get_ForwardTracker_position(), 0, get_absolute_heading());
            pros::delay(5);
        }
    }

    void drive_with_voltage(float leftVoltage, float rightVoltage){
        leftMotorGroup.moveVoltage(leftVoltage);
        rightMotorGroup.moveVoltage(rightVoltage);
    }

    void drive_distance(float distance, float heading = drive_desired_heading, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti){
        drive_desired_heading = heading;
        PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
        PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        while(drivePID.is_settled() == false){
            average_position = (get_left_position_in()+get_right_position_in())/2.0;
            float drive_error = distance+start_average_position-average_position;
            float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
            float drive_output = drivePID.compute(drive_error);
            float heading_output = headingPID.compute(heading_error);

            drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
            heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

            drive_with_voltage(drive_output+heading_output, drive_output-heading_output);
            printf("f%\n", (drive_error));
            delay(10);
        }
        leftMotorGroup.moveVoltage(0);
        rightMotorGroup.moveVoltage(0);
    }

    void turn_to_angle(float angle, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti){
    drive_desired_heading = angle;
    PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
    while(turnPID.is_settled() == false){
        float error = reduce_negative_180_to_180(angle - get_absolute_heading());
        float output = turnPID.compute(error);
        output = clamp(output, -turn_max_voltage, turn_max_voltage);
        drive_with_voltage(output, -output);
        delay(10);
    }
    leftMotorGroup.moveVoltage(0);
    rightMotorGroup.moveVoltage(0);
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
