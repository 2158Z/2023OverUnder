#include "main.h"
#include "display/lv_objx/lv_img.h"
#include "selector.h"
LV_IMG_DECLARE(zerotwo);
LV_IMG_DECLARE(canman_left);
namespace selector {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    int auton = 0;
    int side = 0;
    int index;
    void init(int defaultAuton, const char **closeAutons, const char **farAutons){
        master.clear();
        const char** currentAuton = closeAutons;
        index = defaultAuton;
        // while(pros::competition::is_disabled()){
        while(1){
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && side != 1){
                side += 1;
                pros::delay(1000/8);
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && side > -1){
                side -= 1;
                pros::delay(1000/8);
            }

            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && index < sizeof(currentAuton) - 1){
                index += 1;
                pros::delay(1000/8);
            }
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && index > 0){
                index -= 1;
                pros::delay(1000/8);
            }
            switch(selector::side){
                case -1:
                    master.set_text(0, 0, closeAutons[index]);
                    pros::delay(60);
                    master.set_text(1, 0, "closeSide     ");
                    break;
                case 1:
                    master.set_text(0, 0, farAutons[index]);
                    pros::delay(60);
                    master.set_text(1, 0, "farSide       ");
                    break;
                case 0:
                    master.set_text(0, 0, "Skills        ");
                    pros::delay(60);
                    master.set_text(1, 0, "uwu           ");
                    break;
                default:
                    master.set_text(0, 0, closeAutons[index]);
                    pros::delay(60);
                    master.set_text(1, 0, "closeSide     ");
            }
            pros::delay(60);
            auton = side * index;
            printf("Running");
            if (selector::side == 0){ auton = 0;}
        }
    }
}
