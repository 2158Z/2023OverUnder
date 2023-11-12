#include "main.h"
#include "auton.h"
using namespace okapi;
namespace auton{
    IMU inertial(1);
    QLength meter(1.0); // SI base unit
    QLength decimeter = meter / 10;
    QLength centimeter = meter / 100;
    QLength millimeter = meter / 1000;
    QLength kilometer = 1000 * meter;
    QLength inch = 2.54 * centimeter;
    QLength foot = 12 * inch;
    QLength yard = 3 * foot;
    QLength mile = 5280 * foot;
    QLength tile = 24 * inch;

    QAngle radian(1.0);
    QAngle degree = static_cast<double>(2_pi / 360.0) * radian;

    void test(std::shared_ptr<okapi::ChassisController> chassis, Motor cata, pros::ADIDigitalIn switchLim){
        cata.setBrakeMode(AbstractMotor::brakeMode::hold);
        chassis->moveDistance(6_in);
        chassis->turnAngle(-45_deg);
        chassis->moveDistance(6_in);
    }

    void progSkills(std::shared_ptr<okapi::ChassisController> chassis, Motor cata){
        cata.setBrakeMode(AbstractMotor::brakeMode::hold);
        cata.moveVoltage(9000);
    }

    void autonTest(std::shared_ptr<okapi::ChassisController> chassis){
        chassis->moveDistance(-28_in);
    }
    void wings(pros::ADIDigitalOut wings, int time){
        wings.set_value(true);
        pros::delay(time);
        wings.set_value(false);
    }
    void intake(okapi::Motor intake, int time){
        intake.moveVoltage(8000);
        pros::delay(time);
        intake.moveVoltage(0);
    }

    void moveDistance(std::shared_ptr<okapi::ChassisController> chassis, int inches){
        inches = (static_cast<double>(inches)*3)/2;
        chassis -> moveDistance(inches*inch);
    }

    void turnAngle(std::shared_ptr<okapi::ChassisController> chassis, int deg){
        inertial.reset();
        double target = inertial.get() + deg;
        deg = static_cast<double>(deg*2);
        chassis -> turnAngle(deg*degree);
        double curr = inertial.get();
        double offset = (abs(curr) - target)*2;
        chassis -> turnAngle(-offset*degree);
    }

}
