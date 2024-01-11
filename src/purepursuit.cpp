#include "main.h"
#include <cmath>

double k = 0.1;
double look_distance = 2.0;
double drive_kp = 1.0;
double heading_kp = 2.0;
double tick = 0.1;
double wheelbase = 2.9;
double target_voltage = 12000.0;

double left_voltage = 0;
double right_voltage = 0;

class TargetCourse {
public:
    std::vector<double> cx, cy;
    int current_index;

    TargetCourse(std::vector<double> cx, std::vector<double> cy) : cx(cx), cy(cy), current_index(0) {}

    // Find the index of the next point.
    std::pair<int, double> search_target_index(Odom odom) {
        double min_distance = std::numeric_limits<double>::max();
        int nearest_point_index = current_index;

        for (size_t i = current_index; i < cx.size(); i++) {
            // Displacement till point
            double dx = odom.X_position - cx[i];
            double dy = odom.Y_position - cy[i];
            // Distance to point
            double d = sqrt(dx * dx + dy * dy);
            if (d < min_distance) {
                min_distance = d;
                nearest_point_index = i;
            }
        }

        return std::make_pair(nearest_point_index, min_distance);
    }
};
std::pair<double, int> pure_pursuit_steer_control(Odom odom, TargetCourse trajectory) {
    int ind, _;
    std::tie(ind, _) = trajectory.search_target_index(odom);

    double tx, ty;
    if (ind < trajectory.cx.size()) {
        tx = trajectory.cx[ind];
        ty = trajectory.cy[ind];
    } else {
        tx = trajectory.cx.back();
        ty = trajectory.cy.back();
        ind = trajectory.cx.size() - 1;
    }
    // Heading error
    double alpha = atan2(ty - odom.Y_position, tx - odom.X_position) - odom.orientation_deg;
    // Required heading correction
    double delta = atan2(heading_kp * wheelbase * sin(alpha) / look_distance, 1.0);

    return std::make_pair(delta, ind);
}

// int main() {
//     Odom odom;
//     std::vector<double> cx = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
//     std::vector<double> cy = {0.0, 10.0, -5.0, 15.0, -10.0, 5.0};
//     TargetCourse target_course(cx, cy);

//     while (1) {
//         double di;
//         int nearest_index, _;
//         std::tie(di, nearest_index) = pure_pursuit_steer_control(odom, target_course);

//         if (nearest_index == target_course.cx.size() - 1) {
//             drive_with_voltage(0, 0);
//             break;
//         }

//         left_voltage = drive_kp * (target_voltage - left_voltage) - wheelbase * di;
//         right_voltage = drive_kp * (target_voltage - right_voltage) + wheelbase * di;

//         drive_with_voltage(left_voltage, right_voltage);
//         pros::delay(int(tick * 1000));
//     }

//     return 0;
// }
