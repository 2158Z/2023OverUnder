#include "main.h"
#include "auton.h"
using namespace okapi;
namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis, Motor cata, pros::ADIDigitalIn switch){
        if (switch.get_value()){
            cata.moveVoltage(2000);
        }
    }
    void logError(){
    }
}
