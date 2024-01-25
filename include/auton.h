#pragma once

#include "main.h"
#include <vector>

namespace auton {
    extern float wheel_diameter;
    extern float wheel_ratio;
    extern float gyro_scale;
    extern float drive_in_to_deg_ratio;
    extern float forwardTrackerCenterDistance;
    extern float forwardTrackerDiameter;
    extern float forwardTrackerInToDegRatio;
    extern float SidewaysTracker_center_distance;
    extern float sidewaysTrackerDiameter;
    extern float sidewaysTrackerInToDegRatio;

    extern std::vector<float> driveConstants;
    extern std::vector<float> turnConstants;

    void position_track();
    void progSkills(pros::Motor cata);
    void wings(pros::ADIDigitalOut wings, int time);
    void setWingState(pros::ADIDigitalOut Rwings, pros::ADIDigitalOut Lwings, bool state);
    void intake(pros::Motor intake, int time);
    float get_absolute_heading();
    float get_left_position_in();
    float get_right_position_in();
    float get_X_position();
    float get_Y_position();
    void drive_with_voltage(float leftVoltage, float rightVoltage);
    
    /**
    * @brief Drive the robot a specified distance with a heading using PID controllers.
    *
    * This function uses PID controllers to drive the robot a specified distance with a specified heading.
    *
    * @param distance Desired distance to drive in centimeters.
    * @param dConstants The drive constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
    */
    void drive_distance(float distance, std::vector<float> dConstants = driveConstants);
    
    /**
    * @brief Turn the robot to a specified angle using turn PID controller.
    *
    * Adjusts the robot's heading to the specified angle using a PID controller
    *
    * @param angle The target angle to turn towards
    * @param tConstants The turning constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
    */
   
    void turn_to_angle(float angle, std::vector<float> tConstants = turnConstants);

    /**
     * @brief Drive to a point using odometry and Pid
     * 
     * @param X The desired point's X axis
     * @param Y The desired point's Y axis
     * @param dConstants The driving constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
     * @param tConstants The turning constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
     */
    void driveToPoint(float X, float Y, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants);
    void driveTurn(float distance, float angle, float turnWeight, float drive_timeout = drive_drive_timeout, float turn_timeout = drive_turn_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float drive_settle_time = drive_drive_settle_time, float drive_settle_error = drive_drive_settle_error, float drive_max_voltage = drive_drive_max_voltage, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_max_voltage = drive_turn_max_voltage);
} // namespace auton