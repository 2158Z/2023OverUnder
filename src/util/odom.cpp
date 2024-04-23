#include <math.h>
#include "util/odom.hpp"

pros::Task* trackingTask = nullptr;
pros::IMU inertial(inertialID);

pros::Motor driveLeftFront(driveLeftFrontID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftMiddle(driveLeftMiddleID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveLeftBack(driveLeftBackID, pros::E_MOTOR_GEAR_BLUE, 1, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor driveRightFront(driveRightFrontID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightMiddle(driveRightMiddleID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor driveRightBack(driveRightBackID, pros::E_MOTOR_GEAR_BLUE, 0, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group driveLeft( {driveLeftFront, driveLeftMiddle, driveLeftBack} );
pros::Motor_Group driveRight( {driveRightFront, driveRightMiddle, driveRightBack} );
pros::Motor_Group fullMotorGroup( {driveRightBack, driveRightMiddle, driveRightFront, driveLeftBack, driveLeftMiddle, driveLeftFront} );

double diameter = 0;
double rpm = 0;
double driveOffset = 0;

Pose odomPose(0, 0, 0);
Pose odomSpeed(0, 0, 0);
Pose odomLocalSpeed(0, 0, 0);

float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;

float getDistanceTraveled() {
    std::vector<pros::motor_gearset_e_t> gearsets = fullMotorGroup.get_gearing();
    std::vector<double> positions = fullMotorGroup.get_positions();
    std::vector<float> distances;
    for (int i = 0; i < fullMotorGroup.size(); i++) {
        float in;
        switch (gearsets[i]) {
            case pros::E_MOTOR_GEARSET_36: in = 100; break;
            case pros::E_MOTOR_GEARSET_18: in = 200; break;
            case pros::E_MOTOR_GEARSET_06: in = 600; break;
            default: in = 200; break;
        }
        distances.push_back(positions[i] * (diameter * M_PI) * (rpm / in));
    }
    return avg(distances);
}

Pose getPose(bool radians) {
    if (radians) return odomPose;
    else return Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void setPose(Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = Pose(pose.x, pose.y, degToRad(pose.theta));
}

Pose getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

Pose getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

Pose estimatePose(float time, bool radians) {
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    Pose deltaLocalPose = localSpeed * time;

    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void update() {
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    vertical1Raw = getDistanceTraveled();
    vertical2Raw = getDistanceTraveled();
    imuRaw = degToRad(inertial.get_heading());

    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    float heading = odomPose.theta;
    heading -= (deltaVertical1 - deltaVertical2);
    heading += deltaImu;
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    float rawVertical = 0;
    float rawHorizontal = 0;
    rawVertical = getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    verticalOffset = driveOffset;

    float deltaX = 0;
    float deltaY = 0;
    deltaY = rawVertical - prevVertical;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) {
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    Pose prevPose = odomPose;

    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;

    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);
}

void init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}