#pragma once

#include "main.h"

namespace auton {
    extern float wheel_diameter;
    extern float wheel_ratio;
    extern float gyro_scale;
    extern float drive_in_to_deg_ratio;
    extern float ForwardTracker_center_distance;
    extern float ForwardTracker_diameter;
    extern float ForwardTracker_in_to_deg_ratio;
    extern float SidewaysTracker_center_distance;
    extern float SidewaysTracker_diameter;
    extern float SidewaysTracker_in_to_deg_ratio;

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
    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata);
    void wings(pros::ADIDigitalOut wings, int time);
    void intake(okapi::Motor intake, int time);
    float get_absolute_heading();
    float get_left_position_in();
    float get_right_position_in();
    float get_X_position();
    float get_Y_position();
    void drive_with_voltage(float leftVoltage, float rightVoltage);
    void drive_distance(float distance, float heading = drive_desired_heading, float drive_max_voltage = drive_drive_max_voltage, float heading_max_voltage = drive_heading_max_voltage, float drive_settle_error = drive_drive_settle_error, float drive_settle_time = drive_drive_settle_time, float drive_timeout = drive_drive_timeout, float drive_kp = drive_drive_kp, float drive_ki = drive_drive_ki, float drive_kd = drive_drive_kd, float drive_starti = drive_drive_starti, float heading_kp = drive_heading_kp, float heading_ki = drive_heading_ki, float heading_kd = drive_heading_kd, float heading_starti = drive_heading_starti);
    void turn_to_angle(float angle, float turn_max_voltage = drive_turn_max_voltage, float turn_settle_error = drive_turn_settle_error, float turn_settle_time = drive_turn_settle_time, float turn_timeout = drive_turn_timeout, float turn_kp = drive_turn_kp, float turn_ki = drive_turn_ki, float turn_kd = drive_turn_kd, float turn_starti = drive_turn_starti);

} // namespace auton