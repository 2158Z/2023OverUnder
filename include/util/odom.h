#pragma once
#include "main.h"

class odom
{
    public:

    pros::Motor leftIME;
    pros::Motor rightIME;

    // pros::Rotation leftEncoder;
    // pros::Rotation rightEncoder;

    float wheelDiameter= 0;
    float wheelRatio = 0;
    float track = 0;

    float deltaLeft = 0;
    float deltaRight = 0;

    float previousLeft = 0;
    float previousRight = 0;

    float deltaHeading = 0;
    float previousHeading = 0;

    odom(pros::Motor leftIME, pros::Motor rightIME, float wheelDiameter, float wheelRatio, float track); 
    // odom(pros::Rotation leftEncoder, pros::Rotation rightEncoder, float wheelDiameter, float track); 
    float compute();

};