#include "main.h"

int auton;
#include "selector.h"
namespace selector{
    int auton;
    int autonCount;
    const char *btnMap[] = {"","","","","","","","","","",""};

    lv_obj_t *tabView;
    lv_obj_t *redBtn;
    lv_obj_t *blueBtn;
    double batteryLevel = pros::battery::get_current()
    double batteryTemp = pros::battery::get_temperature()

    lv_res_t redBtnAction(lv_obj_t *btnm, const char *txt){
        for(int i = 0; i < autonCount; i++){
            if(strcmp(txt, btnMap[i]) == 0){
                auton = i+1;
            }
        }

        return LV_RES_OK;
    }

    lv_res_t blueBtnAction(lv_obj_t *btnm, const char *txt){
        for(int i = 0; i < autonCount; i++){
            if(strcmp(txt, btnMap[i]) == 0){
                auton = -(i+1);
            }
        }

        return LV_RES_OK;
    }

    lv_res_t skillsBtnAction(lv_obj_t *btn){
        auton = 0;
        return LV_RES_OK;
    }

    int tabWatcher(){
        int activeTab = lv_tabview_get_tab_act(tabView);
        lv_obj_t * batteryLabel = lv_label_create(miscTab, NULL);
        lv_label_set_text(batteryLabel, batteryLevel)
        lv_obj_set_size(batteryLabel, 450, 50);
        lv_obj_set_pos(batteryLabel, 0, 100);
        lv_obj_align(batteryLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
        
        while(1){
            int currentTab = lv_tabview_get_tab_act(tabView);

            if(currentTab != activeTab){
                activeTab = currentTab;
                switch(activeTab){
                    case 0:
                        if(auton == 0) auton = 1;
                        auton = abs(auton);
                        lv_btnm_set_toggle(redBtn, true, abs(auton)-1);
                        break;
                    case 1:
                        if(auton == 0) auton = -1;
                        auton = -abs(auton);
                        lv_btnm_set_toggle(blueBtn, true, abs(auton)-1);
                        break;
                    case 2:
                        auton = 0;
                        break;
                    case 3:
                        if(batteryLevel > 80){
                            lv_label_set_text(batteryLabel, SYMBOL_BATTERY_FULL);
                        }else if (batteryLevel > 60 || batteryLevel < 80){
                            lv_label_set_text(batteryLabel, SYMBOL_BATTERY_3);
                        }else if (batteryLevel > 40 || batteryLevel < 60){
                            lv_label_set_text(batteryLabel, SYMBOL_BATTERY_2);
                        }else if (batteryLevel > 20 || batteryLevel < 40){
                            lv_label_set_text(batteryLabel, SYMBOL_BATTERY_1);
                        }else if (batteryLevel < 20){
                            lv_label_set_text(batteryLabel, SYMBOL_BATTERY_EMPTY);
                        }
                        break;
                }
            }
            pros::delay(20);
        }
    }

    void init(int defaultAuton, const char **autons){
        int i = 0;
        do{
            memcpy(&btnMap[i], &autons[i], sizeof(&autons));
            i++;
        }while(strcmp(autons[i], "") != 0);

        autonCount = i;
        auton = defaultAuton;

        lv_theme_t *th = lv_theme_alien_init(60, NULL);
        lv_theme_set_current(th);

        tabView = lv_tabview_create(lv_scr_act(), NULL);

        lv_obj_t *redTab = lv_tabview_add_tab(tabView, "Red");
        lv_obj_t *blueTab = lv_tabview_add_tab(tabView, "Blue");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Skills");
        lv_obj_t *miscTab = lv_tabview_add_tab(tabView, "Misc")

        if(auton < 0){
            lv_tabview_set_tab_act(tabView, 1, LV_ANIM_NONE);
        }else if(auton == 0){
            lv_tabview_set_tab_act(tabView, 2, LV_ANIM_NONE);
        }

        redBtn = lv_btnm_create(redTab, NULL);
        lv_btnm_set_map(redBtn, btnMap);
        lv_btnm_set_action(redBtn, *redBtnAction);
        lv_btnm_set_toggle(redBtn, true, abs(auton)-1);
        lv_obj_set_size(redBtn, 450, 50);
        lv_obj_set_pos(redBtn, 0, 100);
        lv_obj_align(redBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        blueBtn = lv_btnm_create(blueTab, NULL);
        lv_btnm_set_map(blueBtn, btnMap);
        lv_btnm_set_action(blueBtn, *blueBtnAction);
        lv_btnm_set_toggle(blueBtn, true, abs(auton)-1);
        lv_obj_set_size(blueBtn, 450, 50);
        lv_obj_set_pos(blueBtn, 0, 100);
        lv_obj_align(blueBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        lv_obj_t *skillsBtn = lv_btn_create(skillsTab, NULL);
        lv_obj_t *label = lv_label_create(skillsBtn, NULL);
        lv_label_set_text(label, "Skills");
        lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, *skillsBtnAction);
        lv_obj_set_size(skillsBtn, 450, 50);
        lv_obj_set_pos(skillsBtn, 0, 100);
        lv_obj_align(skillsBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        pros::Task tabWatcher_task(tabWatcher);

    }
}