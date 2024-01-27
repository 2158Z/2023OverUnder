#include "main.h"
#include "display/lv_objx/lv_img.h"
#include "selector.h"
LV_IMG_DECLARE(zerotwo);
LV_IMG_DECLARE(canman_left);
namespace selector {
    int auton;
    int autonCount;
    const char *btnMapRed[] = {"","","","","","","","","","",""};
    const char *btnMapBlue[] = {"","","","","","","","","","",""};

    lv_obj_t *tabView;
    lv_obj_t *miscTab;
    lv_obj_t *redBtn;
    lv_obj_t *blueBtn;
    lv_obj_t *canman;
    
    lv_obj_t *redBtnLabel;
    lv_obj_t *blueBtnLabel;

    lv_obj_t *cordLabel;
    lv_obj_t *tempLabel;
    lv_obj_t *autonLabel;
    lv_obj_t *phaseLabel;

    lv_style_t relButtonStyle; //released style
    lv_style_t prButtonStyle; //pressed style

    lv_obj_t *Pspinbox;

    lv_res_t redBtnAction(lv_obj_t *btn){
        if (btnMapRed[abs(auton)] == ""){auton = 0;}
        auton = auton + 1;
        lv_label_set_text(redBtnLabel, btnMapRed[abs(auton) - 1]);
        lv_btn_set_state(redBtn, LV_BTN_STATE_REL);  // Set the button state to released
        return LV_RES_OK;
    }

    lv_res_t blueBtnAction(lv_obj_t *btn){
        if (btnMapBlue[abs(auton)] == ""){auton = 0;}
        auton = auton - 1;
        lv_label_set_text(blueBtnLabel, btnMapBlue[abs(auton) - 1]);
        lv_btn_set_state(blueBtn, LV_BTN_STATE_REL);  // Set the button state to released
        printf("%f \n", auton);
        return LV_RES_OK;
    }

    lv_res_t skillsBtnAction(lv_obj_t *btn){
        auton = 0;
        return LV_RES_OK;
    }

    int tabWatcher(){
        int activeTab = lv_tabview_get_tab_act(tabView);
        while(1){
            int currentTab = lv_tabview_get_tab_act(tabView);
            if(currentTab != activeTab){
                activeTab = currentTab;
                switch(activeTab){
                    case 0:
                        if(auton == 0) auton = 1;
                        auton = abs(auton);
                        lv_label_set_text(redBtnLabel, btnMapRed[abs(auton) - 1]);
                        lv_btnm_set_toggle(redBtn, true, abs(auton)-1);
                        break;
                    case 1:
                        if(auton == 0) auton = -1;
                        auton = -abs(auton);
                        lv_label_set_text(blueBtnLabel, btnMapBlue[abs(auton) - 1]);
                        lv_btnm_set_toggle(blueBtn, true, abs(auton)-1);
                        break;
                    case 2:
                        auton = 0;
                        break;
                    case 3:
                        auton = 0;
                        break;
                    default:
                        break;
                }
            }
            printf("Auton: %d\n", auton);
            pros::delay(20);
        }
    }
    void init(int defaultAuton, const char **redAutons, const char **blueAutons){
        int i = 0;
        do{
            memcpy(&btnMapRed[i], &redAutons[i], sizeof(&redAutons));
            i++;
        } while(strcmp(redAutons[i], "") != 0);
        i = 0;
        do{
            memcpy(&btnMapBlue[i], &blueAutons[i], sizeof(&blueAutons));
            i++;
        } while(strcmp(blueAutons[i], "") != 0);

        autonCount = i;
        auton = defaultAuton;

        lv_theme_t *th = lv_theme_alien_init(60, NULL);
        lv_theme_set_current(th);

        tabView = lv_tabview_create(lv_scr_act(), NULL);

        lv_obj_t *redTab = lv_tabview_add_tab(tabView, "Close");
        lv_obj_t *blueTab = lv_tabview_add_tab(tabView, "Far");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Skills");
        miscTab = lv_tabview_add_tab(tabView, "Misc");

        if(auton < 0){
            lv_tabview_set_tab_act(tabView, 1, LV_ANIM_NONE);
        } else if(auton == 0){
            lv_tabview_set_tab_act(tabView, 2, LV_ANIM_NONE);
        }

        redBtn = lv_btn_create(redTab, NULL);
        redBtnLabel = lv_label_create(redBtn, NULL);
        lv_label_set_text(redBtnLabel, btnMapRed[abs(auton) - 1]);
        lv_obj_set_size(redBtn, 450, 50);
        lv_obj_set_pos(redBtn, 0, 100);
        lv_obj_align(redBtn, NULL, LV_ALIGN_CENTER, 0, 0);
        lv_btn_set_action(redBtn, LV_BTN_ACTION_CLICK, redBtnAction);

        blueBtn = lv_btn_create(blueTab, NULL);
        blueBtnLabel = lv_label_create(blueBtn, NULL);
        lv_label_set_text(blueBtnLabel, btnMapBlue[abs(auton) - 1]);
        lv_obj_set_size(blueBtn, 450, 50);
        lv_obj_set_pos(blueBtn, 0, 100);
        lv_obj_align(blueBtn, NULL, LV_ALIGN_CENTER, 0, 0);
        lv_btn_set_action(blueBtn, LV_BTN_ACTION_CLICK, blueBtnAction);

        lv_obj_t *skillsBtn = lv_btn_create(skillsTab, NULL);
        lv_obj_t *skillLabel = lv_label_create(skillsBtn, NULL);
        lv_label_set_text(skillLabel, "Skills");
        lv_btn_set_action(skillsBtn, LV_BTN_ACTION_CLICK, *skillsBtnAction);
        lv_obj_set_size(skillsBtn, 450, 50);
        lv_obj_set_pos(skillsBtn, 0, 100);
        lv_obj_align(skillsBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        canman = lv_img_create(lv_scr_act(), NULL);
        lv_img_set_src(canman, &canman_left);
        lv_img_set_auto_size(canman, true);
        lv_obj_set_x(canman, 167);
        lv_obj_set_y(canman, 65);
        lv_obj_align(canman, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 0, 0);

        cordLabel = lv_label_create(miscTab, NULL);
        lv_obj_align(cordLabel, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);

        tempLabel = lv_label_create(miscTab, NULL);
        lv_obj_align(tempLabel, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

        autonLabel = lv_obj_create(miscTab, NULL);
        lv_obj_align(autonLabel, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

        phaseLabel = lv_obj_create(miscTab, NULL);
        lv_obj_align(phaseLabel, NULL, LV_ALIGN_CENTER, 0, 0);

        pros::Task tabWatcher_task(tabWatcher);
    }
}
