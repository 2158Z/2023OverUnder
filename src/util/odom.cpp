#include "main.h"

odom::odom(pros::Motor ileftIME, pros::Motor irightIME, float iwheelDiameter, float iwheelRatio, float itrack) :
    leftIME(ileftIME),
    rightIME(irightIME),
    wheelDiameter(iwheelDiameter),
    wheelRatio(iwheelRatio),
    track(itrack)
{};

float odom::compute() {
    //deltaLeft = previousLeft - leftIME.get_position();
    //deltaRight = previousRight - rightIME.get_position();
    //printf("%f %f", deltaLeft, deltaRight);
    printf("%f", 1);
    //pros::delay(10);
}

// odom::odom(pros::Rotation ileftEncoder, pros::Rotation irightEncoder, float iwheelDiameter, float itrack) :
//     leftEncoder(ileftEncoder),
//     rightEncoder(irightEncoder),
//     wheelDiameter(iwheelDiameter),
//     track(itrack)
// {};
    