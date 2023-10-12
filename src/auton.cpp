#include "main.h"
#include "auton.h"
namespace auton{
  void default_constants(){
    chassis.set_drive_constants(12, 1.5, 0, 10, 0);
    chassis.set_heading_constants(6, .4, 0, 1, 0);
    chassis.set_turn_constants(12, .41, .03, 3.2, 15);
    chassis.set_swing_constants(12, .3, .001, 2, 15);
    chassis.set_drive_exit_conditions(1.5, 300, 5000);
    chassis.set_turn_exit_conditions(1, 300, 3000);
    chassis.set_swing_exit_conditions(1, 300, 3000);
  }

  void drive(int desiredDist, double voltage, double timeout){
    chassis.set_drive_constants(voltage, 1.5, 0, 10, 0);
    chassis.drive_distance(desiredDist);
  }
  void turn(int desiredAngle){
    chassis.turn_to_angle(desiredAngle);
  }
}
