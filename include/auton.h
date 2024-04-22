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
    void driveDistance(float distance, float timeout = driveConstants[7], std::vector<float> dConstants = driveConstants);
    
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

    void driveTurn(float distance, float angle, float turnWeight, float settleTime = driveConstants[5], float timeout = driveConstants[7], std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants);
    void absDriveTurn(float distance, float angle, float turnWeight, float settleTime = driveConstants[5], float timeout = driveConstants[7], std::vector<float> dConstants = driveConstants, std::vector<float> tConstants = turnConstants);

    void setDefaultDriveConstants(std::vector<float> constants);
}