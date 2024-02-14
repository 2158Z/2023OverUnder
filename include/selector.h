#pragma once

#include <string>
#include "main.h"

#define DEFAULT 1
#define REDAUTONS "Close Side Elim", "2", "3"
#define BLUEAUTONS "Far Side 5Ball", "Farside Safe", "-3"

namespace selector{
    extern lv_obj_t *tabView;
    extern int auton;
    extern lv_obj_t *miscTab;
    extern lv_obj_t *cordLabel;
    extern lv_obj_t *tempLabel;
    extern lv_obj_t *autonLabel;
    extern lv_obj_t *phaseLabel;
    static const char *r[] = {REDAUTONS, ""};
    static const char *b[] = {BLUEAUTONS, ""};
    void init(int defaultAuton = DEFAULT, const char **redAutons = r, const char **blueAutons = b);
}
