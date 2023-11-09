#include "main.h"
#include "auton.h"
using namespace okapi;
namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis, Motor cata, pros::ADIDigitalIn switchLim){
        chassis->moveDistance(6_in);
        chassis->turnAngle(-45_deg);
        chassis->moveDistance(6_in);
    }
    void neumatics(){
    }
    void wings(pros::ADIDigitalOut wings, int time){
        piston.set_value(true);
        pros::delay(time);
        piston.set_value(false);
    }
    void intake(okapi::Motor intake, int time){
        intake.moveVoltage(8000)
        pros::delay(time);
        intake.moveVoltage(0)
    }
}
