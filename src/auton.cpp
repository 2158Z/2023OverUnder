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
}
