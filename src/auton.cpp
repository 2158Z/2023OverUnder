#include "main.h"
#include "auton.h"
using namespace okapi;
namespace auton{

    void test(std::shared_ptr<okapi::ChassisController> chassis, Motor cata, pros::ADIDigitalIn switchLim){
        cata.setBrakeMode(AbstractMotor::brakeMode::hold);
        chassis->moveDistance(6_in);
        chassis->turnAngle(-45_deg);
        chassis->moveDistance(6_in);
    }

    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, Motor cata, pros::ADIDigitalOut rightWing, pros::ADIDigitalOut leftWing){
        cata.setBrakeMode(AbstractMotor::brakeMode::hold);
        rightWing.set_value(1);
        cata.moveVoltage(8000);
    }

    void autonTest(std::shared_ptr<okapi::ChassisController> chassis){
        chassis->moveDistance(-28_in);
    }
    void wings(pros::ADIDigitalOut wings, int time){
        wings.set_value(true);
        pros::delay(time);
        wings.set_value(false);
    }
    void intake(okapi::Motor intake, int time){
        intake.moveVoltage(8000);
        pros::delay(time);
        intake.moveVoltage(0);
    }
}
