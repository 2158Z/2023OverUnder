#include "main.h"
#include "auton.h"
using namespace okapi;
namespace auton{
    void test(std::shared_ptr<okapi::ChassisController> chassis){
        chassis->moveDistance(24_in);
    }
    void logError(){
    }
}
