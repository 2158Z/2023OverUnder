#pragma once

#include "pros/rtos.hpp"
#include "util/util.h"
#include "util/pose.hpp"
#include "auton.h"

namespace odom {
float getDistanceTraveled();

Pose getPose(bool radians);

void setPose(Pose pose, bool radians);

Pose getSpeed(bool radians);

Pose getLocalSpeed(bool radians);

Pose estimatePose(float time, bool radians);

void update();

void init();
}