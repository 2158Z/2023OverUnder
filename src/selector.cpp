#pragma once
#include "main.h"

lv_obj_t *tabView;
lv_obj_t *redBtn;
lv_obj_t *blueBtn;
lv_obj_t *skillBtn;
lv_obj_t *testBtn;

namespace selector{
    void init(){
        lv_obj_t *redTab = lv_tabview_add_tab(tabView, "Red");
        lv_obj_t *blueTab = lv_tabview_add_tab(tabView, "Blue");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Skills");
        lv_obj_t *testTab = lv_tabview_add_tab(tabView, "Tests");

        redBtn = lv_btnm_create(redTab, NULL);
        lv_obj_set_size(redBtn, 0, 0); //TODO Change values of gui
        lv_obj_set_pos(redBtn, 0, 0);
        lv_obj_align(redBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        blueBtn = lv_btnm_create(blueTab, NULL);
        lv_obj_set_size(blueBtn, 0, 0);
        lv_obj_set_pos(blueBtn, 0, 0);
        lv_obj_align(blueBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        skillBtn = lv_btnm_create(skillsTab, NULL);
        lv_obj_set_size(skillBtn, 0, 0);
        lv_obj_set_pos(skillBtn, 0, 0);
        lv_obj_align(skillBtn, NULL, LV_ALIGN_CENTER, 0, 0);

        testBtn = lv_btnm_create(testTab, NULL);
        lv_obj_set_size(testBtn, 0, 0);
        lv_obj_set_pos(testBtn, 0, 0);
        lv_obj_align(testBtn, NULL, LV_ALIGN_CENTER, 0, 0);
    }
}