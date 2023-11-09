#pragma once
#include <string>

namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata, pros::ADIDigitalIn switchLim);
    void logError();
    void wings(pros::ADIDigitalOut wings, int time);
    void intake(okapi::Motor intake, int time)
}
