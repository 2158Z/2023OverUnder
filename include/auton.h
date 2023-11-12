#pragma once
#include <string>

namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata, pros::ADIDigitalIn switchLim);
    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata);
    void logError();
    void wings(pros::ADIDigitalOut wings, int time);
    void intake(okapi::Motor intake, int time);
    void autonTest(std::shared_ptr<okapi::ChassisController> chassis);
    void moveDistance(std::shared_ptr<okapi::ChassisController> chassis, int inches);
    void turnAngle(std::shared_ptr<okapi::ChassisController> chassis, int deg);
}
