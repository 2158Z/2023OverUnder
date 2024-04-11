#include "main.h"
#include "display/lv_objx/lv_img.h"
#include "selector.h"
LV_IMG_DECLARE(zerotwo);
LV_IMG_DECLARE(canman_left);
namespace selector {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    int auton = 0;
    void init(int defaultAuton, const char **redAutons, const char **blueAutons){
        master.clear();
        const char** currentAuton = redAutons;
        int currentAutonIndex = 0;
        int index = defaultAuton;
        // while(pros::competition::is_disabled()){
        while(1){
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) || master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                if (currentAutonIndex == 1){
                    currentAutonIndex == -1;
                    currentAuton = redAutons;
                } else {
                    currentAutonIndex == 1;
                    currentAuton = blueAutons;
                }
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && index < sizeof(currentAuton) - 1){
                index ++;
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && index > 0){
                index --;
            }
            master.set_text(0, 0, "                                         ");
            pros::delay(60);
            master.set_text(0, 0, currentAuton[index]);
            pros::delay(60);
            master.set_text(1, 0, (currentAutonIndex == 1) ? "blueAutons" : "redAutons"); //if currentAutonIndex is true, return string blueAuton, else redAutons
            auton = currentAutonIndex * index;
            printf("%f", auton);
        }
    }
}
