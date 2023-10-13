#pragma once

#include <string>

#define DEFAULT 1
#define AUTONS "Monkey", "Monkey2", "Monkey3"

namespace selector{
    extern lv_obj_t *tabView;
    extern int auton;
    static const char *b[] = {AUTONS, ""};
    void init(int defaultAuton=DEFAULT, const char **autons = b);
}
