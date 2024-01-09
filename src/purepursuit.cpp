#include "main.h"
#include "auton.h"
#include "util/odom.h"
#include "purepursuit.h"

// Function to calculate distance between two points
double distance(Odom odom, Point b) {
    return sqrt(pow(b.x - odom.get_Xposition(), 2) + pow(b.y - odom.get_Yposition(), 2));
}

// Function to calculate curvature of the path at a given point
double calculateCurvature(Odom odom, Point pathPoint) {
    double distanceToPath = distance(odom, pathPoint);
    return 2 * (pathPoint.x - odom.get_Xposition()) / (distanceToPath * distanceToPath);
}

// Function to calculate Pure Pursuit control
double calculatePurePursuit(Odom odom, Point path[], int pathLength) {
    int targetIndex = -1;

    // Find the index of the point
    for (int i = 0; i < pathLength; i++) {
        if (distance(odom, path[i]) < 2) {
            targetIndex = i;
        }
    }

    // If point is found, calculate curvature
    if (targetIndex != -1) {
        double curvature = calculateCurvature(odom, path[targetIndex]);
        // Use curvature to calculate heading
        return atan2(1, curvature);
    }
    return 0;
}

// Function to drive the robot with Pure Pursuit
void drivePurePursuit(Odom odom, Point path[], int pathLength) {
    double headingError = calculatePurePursuit(odom, path, pathLength) - auton::get_absolute_heading();
    double distanceError = distance(odom, path[pathLength - 1]); // Use the last point in the path as the target
    double turningCorrection = 0;
    double drivingCorrection = 0;
    auton::drive_with_voltage(12000 - turningCorrection - drivingCorrection, 12000 + turningCorrection - drivingCorrection);
}

// int main() {
//     // Example usage
//     Point robotPosition = {0, 0};
//     Point path[] = {{1, 1}, {2, 3}, {4, 2}, {6, 4}};
//     int pathLength = sizeof(path) / sizeof(path[0]);

//     while (true) {
//         drivePurePursuit(robotPosition, path, pathLength);
//         delay(20);
//     }

//     return 0;
// }
