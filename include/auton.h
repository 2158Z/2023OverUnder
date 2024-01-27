#pragma once

#include "main.h"

namespace auton
{
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
    void drive_distance(float distance, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti);
    void turn_to_angle(float angle, bool turn_bias = false, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti);
    void driveTurn(float distance, float angle, float turnWeight, float drive_timeout = drive_drive_timeout, float turn_timeout = drive_turn_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float drive_settle_time = drive_drive_settle_time, float drive_settle_error = drive_drive_settle_error, float drive_max_voltage = drive_drive_max_voltage, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_max_voltage = drive_turn_max_voltage);
} // namespace auton
