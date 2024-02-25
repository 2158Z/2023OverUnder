#pragma once
#include "main.h"
using namespace pros;

enum drive_setup {ZERO_TRACKER_NO_ODOM, ZERO_TRACKER_ODOM, TANK_ONE_ENCODER, TANK_ONE_ROTATION, TANK_TWO_ENCODER, TANK_TWO_ROTATION, HOLONOMIC_TWO_ENCODER, HOLONOMIC_TWO_ROTATION};

class Odom
{
private:
    float ForwardTracker_center_distance;
    float SidewaysTracker_center_distance;
    float ForwardTracker_position;
    float SideWaysTracker_position;

public:
    Odom odom;
    drive_setup drive_setup = ZERO_TRACKER_NO_ODOM;

    rotation R_ForwardTracker;
    rotation R_SidewaysTracker;
    encoder E_ForwardTracker;
    encoder E_SidewaysTracker;

    float X_position;
    float Y_position;
    float orientation_deg;
    void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
    void update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg);
    void set_physical_distances(float ForwardTracker_center_distance, float   SidewaysTracker_center_distance);
};