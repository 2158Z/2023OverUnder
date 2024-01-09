#pragma once

#include "main.h"

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

    extern float drive_turn_max_voltage;
    extern float drive_turn_kp;
    extern float drive_turn_ki;
    extern float drive_turn_kd;
    extern float drive_turn_starti;

    extern float drive_turn_settle_error;
    extern float drive_turn_settle_time;
    extern float drive_turn_timeout;

    extern float drive_drive_max_voltage;
    extern float drive_drive_kp;
    extern float drive_drive_ki;
    extern float drive_drive_kd;
    extern float drive_drive_starti;

    extern float drive_drive_settle_error;
    extern float drive_drive_settle_time;
    extern float drive_drive_timeout;

    extern float drive_heading_max_voltage;
    extern float drive_heading_kp;
    extern float drive_heading_ki;
    extern float drive_heading_kd;
    extern float drive_heading_starti;

    extern float drive_swing_max_voltage;
    extern float drive_swing_kp;
    extern float drive_swing_ki;
    extern float drive_swing_kd;
    extern float drive_swing_starti;

    extern float drive_swing_settle_error;
    extern float drive_swing_settle_time;
    extern float drive_swing_timeout;

    extern float drive_desired_heading;

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
    * @param drive_max_voltage Maximum voltage allowed for driving (default: drive_drive_max_voltage).
    * @param heading_max_voltage Maximum voltage allowed for heading adjustment (default: drive_heading_max_voltage).
    * @param drive_settle_error Error threshold for con1 sidering the drive settled (default: drive_drive_settle_error).
    * @param drive_settle_time Time required for the drive to be considered settled (default: drive_drive_settle_time).
    * @param drive_timeout Timeout duration for the drive operation (default: drive_drive_timeout).
    * @param drive_kp Proportional gain for the drive PID controller (default: drive_drive_kp).
    * @param drive_ki Integral gain for the drive PID controller (default: drive_drive_ki).
    * @param drive_kd Derivative gain for the drive PID controller (default: drive_drive_kd).
    * @param drive_starti Initial integral value for the drive PID controller (default: drive_drive_starti).
    * @param heading_kp Proportional gain for the heading PID controller (default: drive_heading_kp).
    * @param heading_ki Integral gain for the heading PID controller (default: drive_heading_ki).
    * @param heading_kd Derivative gain for the heading PID controller (default: drive_heading_kd).
    * @param heading_starti Initial integral value for the heading PID controller (default: drive_heading_starti).
    */
    void drive_distance(float distance, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti);
    
    /**
    * @brief Turn the robot to a specified angle using turn PID controller.
    *
    * Adjusts the robot's heading to the specified angle using a PID controller
    *
    * @param angle The target angle to turn towards
    * @param turn_bias Flag indicating whether to apply a turn bias (default: false)
    * @param turn_max_voltage Maximum voltage allowed for turning (default: drive_turn_max_voltage)
    * @param turn_settle_error Error threshold for considering the turn settled (default: drive_turn_settle_error)
    * @param turn_settle_time Time required for the turn to be considered settled (default: drive_turn_settle_time)
    * @param turn_timeout Timeout duration for the turn operation (default: drive_turn_timeout)
    * @param turn_kp Proportional gain for the PID controller (default: drive_turn_kp)
    * @param turn_ki Integral gain for the PID controller (default: drive_turn_ki)
    * @param turn_kd Derivative gain for the PID controller (default: drive_turn_kd)
    * @param turn_starti Initial integral value for the PID controller (default: drive_turn_starti)
    */
   
    void turn_to_angle(float angle, bool turn_bias = false, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti);
    
    /**
    * @brief Drive the robot a specific distance with a turn using PID controllers.
    *
    * Drive the robot a specific distance with a specified angle adjustment using PID controllers.
    *
    * @param distance Desired distance to drive in inches
    * @param angle Target angle to maintain during the drive
    * @param turnWeight Weight assigned to the turning component in the drive output
    * @param drive_timeout Timeout duration for the drive operation (default: drive_drive_timeout)
    * @param turn_timeout Timeout duration for the turn operation (default: drive_turn_timeout)
    * @param drive_kp Proportional gain for the drive PID controller (default: drive_drive_kp)
    * @param drive_ki Integral gain for the drive PID controller (default: drive_drive_ki)
    * @param drive_kd Derivative gain for the drive PID controller (default: drive_drive_kd)
    * @param drive_starti Initial integral value for the drive PID controller (default: drive_drive_starti)
    * @param drive_settle_time Time required for the drive to be considered settled (default: drive_drive_settle_time)
    * @param drive_settle_error Error threshold for considering the drive settled (default: drive_drive_settle_error)
    * @param drive_max_voltage Maximum voltage allowed for driving (default: drive_drive_max_voltage)
    * @param turn_kp Proportional gain for the turn PID controller (default: drive_turn_kp)
    * @param turn_ki Integral gain for the turn PID controller (default: drive_turn_ki)
    * @param turn_kd Derivative gain for the turn PID controller (default: drive_turn_kd)
    * @param turn_starti Initial integral value for the turn PID controller (default: drive_turn_starti)
    * @param turn_settle_time Time required for the turn to be considered settled (default: drive_turn_settle_time)
    * @param turn_settle_error Error threshold for considering the turn settled (default: drive_turn_settle_error)
    * @param turn_max_voltage Maximum voltage allowed for turning (default: drive_turn_max_voltage)
    */
    void driveTurn(float distance, float angle, float turnWeight, float drive_timeout = drive_drive_timeout, float turn_timeout = drive_turn_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float drive_settle_time = drive_drive_settle_time, float drive_settle_error = drive_drive_settle_error, float drive_max_voltage = drive_drive_max_voltage, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_max_voltage = drive_turn_max_voltage);
} // namespace auton