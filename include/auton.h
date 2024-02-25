#pragma once

#include "main.h"
#include <vector>

namespace auton {
    extern float wheel_diameter;
    extern float wheel_ratio;

    extern std::vector<float> driveConstants;
    extern std::vector<float> turnConstants;

    void driveVoltage(float leftVoltage, float rightVoltage);
    
    /**
    * @brief Drive the robot a specified distance with a heading using PID controllers.
    *
    * This function uses PID controllers to drive the robot a specified distance with a specified heading.
    *
    * @param distance Desired distance to drive in centimeters.
    * @param dConstants The drive constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
    */
    void driveDistance(float distance, std::vector<float> dConstants = driveConstants);
    
    /**
    * @brief Turn the robot to a specified angle using turn PID controller.
    *
    * Adjusts the robot's heading to the specified angle using a PID controller
    *
    * @param angle The target angle to turn towards
    * @param tConstants The turning constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
    */
   
    void turnAngle(float angle, std::vector<float> tConstants = turnConstants);
    void absTurn(float angle, std::vector<float> tConstants = turnConstants);

    /**
     * @brief Drive to a point using odometry and Pid
     * 
     * @param X The desired point's X axis
     * @param Y The desired point's Y axis
     * @param dConstants The driving constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
     * @param tConstants The turning constants {Max Voltage, KP, KI, KD, starti, settle time, settle error, timeout}
     */
    void driveToPoint(float X, float Y, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants);

    void driveTurn(float distance, float angle, float turnWeight, std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants);
    void setDefaultDriveConstants(std::vector<float> constants);

// odom functions
    float get_ForwardTracker_position();
    float get_SidewaysTracker_position();
    void position_track();

    void set_heading(float orientation_deg);
    void set_coordinates(float X_position, float Y_position, float orientation_deg);
    void get_X_position();
    void get_Y_position();

    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage);
    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout);
    void drive_to_point(float X_position, float Y_position, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti);
    
    void turn_to_point(float X_position, float Y_position, float extra_angle_deg);
    void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout);
    void turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti);
}