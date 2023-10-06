#pragma once
#include "main.h"

lv_obj_t *tabView;
lv_obj_t *redBtnm;
lv_obj_t *blueBtnm;
lv_obj_t *skillBtnm;
lv_obj_t *testBtnm;

namespace selector{
    void init(){
        lv_obj_t *redTab = lv_tabview_add_tab(tabView, "Red");
        lv_obj_t *blueTab = lv_tabview_add_tab(tabView, "Blue");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Skills");
        lv_obj_t *skillsTab = lv_tabview_add_tab(tabView, "Tests");

        redBtnm = lv_btnm_create(redTab, NULL);
        blueBtnm = lv_btnm_create(redTab, NULL);
        skillBtnm = lv_btnm_create(redTab, NULL);
        testBtnm = lv_btnm_create(redTab, NULL);
    }
}