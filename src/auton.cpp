#include "main.h"
#include <vector>
using namespace okapi;
namespace auton{
    pros::Motor driveLeftMotorFront(driveLeftMotorFrontID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor driveLeftMotorMiddle(driveLeftMotorMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor driveLeftMotorBack(driveLeftMotorBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

    // Individual motors for drive right side
    pros::Motor driveRightMotorFront(driveRightMotorFrontID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor driveRightMotorMiddle(driveRightMotorMiddleID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
    pros::Motor driveRightMotorBack(driveRightMotorBackID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
        
    // Motor groups for drive left and rights sides
    pros::Motor_Group leftMotorGroup( {driveLeftMotorBack, driveLeftMotorMiddle, driveLeftMotorFront} );
    pros::Motor_Group rightMotorGroup( {driveRightMotorBack, driveRightMotorMiddle, driveRightMotorFront} );
    pros::Motor_Group fullMotorGroup( {driveRightMotorBack, driveRightMotorMiddle, driveRightMotorFront, driveLeftMotorBack, driveLeftMotorMiddle, driveLeftMotorFront} );



    pros::IMU inertial(inertialID);
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

    // 0-Max Voltage, 1-KP, 2-KI, 3-KD, 4-startI, 5-settle time, 6-settle error, 7-timeout
    std::vector<float> turnConstants = {12000, 0.35, 0.25, 2, 0, 100, 1, 1500};
    std::vector<float> driveConstants = {11000, 1.5, 0, 0.85, 0, 100, 1.5, 1500};

    float wheel_diameter = 2.75;
    float wheel_ratio = 0.75;
    float gyro_scale = 360;

    float drive_turn_max_voltage = 12000;
    float drive_turn_kp = 0.35;
    float drive_turn_ki = 0.25;    
    float drive_turn_kd = 2;
    float drive_turn_starti = 0;

    float drive_turn_settle_error = 1;
    float drive_turn_settle_time = 1000;
    float drive_turn_timeout = 500; //1000

    float drive_drive_max_voltage = 12000;
    float drive_drive_kp = 0.5;
    float drive_drive_ki = 0;
    float drive_drive_kd = 0;
    float drive_drive_starti = 0;

    float drive_drive_with_angle_turn_max_voltage = 0;

    float drive_drive_settle_error = 1;
    float drive_drive_settle_time = 5000;
    float drive_drive_timeout = 10000; //500

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
        if (counter % 100 == 0){
            //printf("%f Left\n", ( (driveLeftMotorMiddle.get_position()/180) * M_PI * wheel_diameter * wheel_ratio))/2;
        }
        return( (driveLeftMotorMiddle.get_position()/360) * M_PI * wheel_diameter * wheel_ratio);
        }

    float get_right_position_in(){
        if (counter % 100 == 0){
            //printf("%f Right\n", -1 * ( (driveRightMotorMiddle.get_position()/360) * M_PI * wheel_diameter * wheel_ratio)/2);
        }
        return( (driveRightMotorMiddle.get_position()/360) * M_PI * wheel_diameter * wheel_ratio);
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

    void drive_distance(float distance, std::vector<float> dConstants = driveConstants) {
        distance = distance / 2.54;
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);
        driveLeftMotorMiddle.tare_position();
        driveRightMotorMiddle.tare_position();
        float start_average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
        float average_position = start_average_position;
        while (drivePID.is_settled() == false) {
            average_position = (get_left_position_in() + get_right_position_in()) / 2.0;
            float drive_error = distance + start_average_position - average_position;
            float drive_output = drivePID.compute(drive_error) * 1000;
            drive_output = clamp(drive_output, -dConstants[0], dConstants[0]);
            drive_with_voltage(drive_output, drive_output);
            printf("%f \n", drive_error);
            delay(10);
        }
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
}

    void turn_to_angle(float angle, std::vector<float> tConstants = turnConstants) {
        drive_desired_heading = angle;
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);
        while (turnPID.is_settled() == false) {
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 10000;
            printf("%f \n", error);
            output = clamp(output, -tConstants[0], tConstants[0]);
            drive_with_voltage(output, -output);
            delay(10);
        }
        leftMotorGroup.move_voltage(0);
        rightMotorGroup.move_voltage(0);
    }

    void driveTurn(float distance, float angle, float turnWeight, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants) {

        driveLeftMotorMiddle.tare_position();
        driveRightMotorMiddle.tare_position();

        float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
        float average_position = start_average_position;
        PID drivePID(distance, dConstants[1], dConstants[2], dConstants[3], dConstants[4], dConstants[5], dConstants[6],
                    dConstants[7]);
        PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), tConstants[1], tConstants[2], tConstants[3], tConstants[4],
                    tConstants[5], tConstants[6], tConstants[7]);
        while((turnPID.is_settled() == false && turnWeight != 0) || drivePID.is_settled() == false){
            float error = reduce_negative_180_to_180(angle - get_absolute_heading());
            float output = turnPID.compute(error) * 10000;
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
