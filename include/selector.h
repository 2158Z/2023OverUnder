#pragma once

#include <string>
#include "main.h"

#define DEFAULT 1
#define REDAUTONS "Winpoint         ", "Elim                   ", "3                    "
#define BLUEAUTONS "Far Side Safe            ", "2             ", "-3           "

namespace selector{
    extern int auton;
    static const char *close[] = {REDAUTONS, ""};
    static const char *far[] = {BLUEAUTONS, ""};
    void init(int defaultAuton = DEFAULT, const char **closeAutons = close, const char **farAutons = far);
}
