#pragma once

class xOdom
{
private:
  float ForwardTracker_center_distance;
  float SidewaysTracker_center_distance;
  float ForwardTracker_position;
  float SideWaysTracker_position;
public:
  float X_position;
  float Y_position;
  float orientation_deg;
  void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
  void update_position(float ForwardTracker_position, float orientation_deg);
  float get_ForwardTracker_position(float deg);
  float get_SidewaysTracker_position(float deg);
  float get_Xposition();
  float get_Yposition();
  void set_physical_distances(float forwardTrackerCenterDistance, float SidewaysTracker_center_distance);
};

