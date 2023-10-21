#pragma once
#include <string>

namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata, pros::ADIDigitalIn switchLim);
    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, okapi::Motor cata, pros::ADIDigitalOut rightWing, pros::ADIDigitalOut leftWing);
    void logError();
    void autonTest(std::shared_ptr<okapi::ChassisController> chassis);
}
