#include "util/purepursuit.h"

long fileSize;
float distTravelled = 0;
float maxAccel = 0;
double trackWidth = 0;
float Kv = 0;
float Ka = 0;
float Kp = 0;

namespace purepur{
  float rateLimit(float target, float current, float maxChange) {
      float change = target - current;
      if (maxChange == 0) return target;
      if (change > maxChange) change = maxChange;
      else if (change < -maxChange) change = -maxChange;
      return current + change;
  }

  template <class T> inline int sgn(T v) {
    return (v > T(0)) - (v < T(0));
  }

  std::vector<double> linearEnterp(std::vector<double> pos, std::vector<double> other, float t) { // support more points?
    std::vector<double> r;
    r = {pos[0] + (other[0] - pos[0]) * t, pos[1] + (other[1] - pos[1]) * t, pos[2]};
    return r;
  }

  // vector operations
  std::vector<double> subtract(std::vector<double> v1, std::vector<double> v2){
    std::vector<double> r;
    r = {v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]};
    return r;
  }

  float multiply(std::vector<double> v1, std::vector<double> v2){
    float r;
    r = v1[0] * v2[0] + v1[1] * v2[1];
    return r;
  }

  //read file from pros file system
  char* readFileToString(const char* filename) {
      FILE *file = fopen(filename, "r");
      if (file == NULL) {
          fprintf(stderr, "Error opening file: %s\n", filename);
          return NULL;
      }
      // Find the file size
      fseek(file, 0, SEEK_END);
      fileSize = ftell(file);
      fseek(file, 0, SEEK_SET);
      // Allocate memory to store the file content
      char* content = (char*)malloc(fileSize + 1);
      if (content == NULL) {
          fprintf(stderr, "Memory allocation error\n");
          fclose(file);
          return NULL;
      }
      // Read file content into the allocated memory
      size_t bytesRead = fread(content, 1, fileSize, file);
      content[bytesRead] = '\0'; // Null-terminate the string
      fclose(file);

      return content;
  }
  //basically python split()
  std::vector<std::string> readElement(const std::string& input, const std::string& delimiter) {
      std::string token;
      std::string s = input;
      std::vector<std::string> output;
      size_t pos = 0;
      while ((pos = s.find(delimiter)) != std::string::npos) { // while there are still delimiters in the string
          token = s.substr(0, pos); // processed substring
          output.push_back(token);
          s.erase(0, pos + delimiter.length()); // remove the read prev substring
      }

      output.push_back(s); // add the last element to the returned string

      return output;
  }
 
  //Interpreting the strings from array as vectors
  std::vector<std::vector<double>> getData(const char* filename) {
      std::vector<std::vector<double>> robotPath;

      std::string fileContent = readFileToString(filename);
      if (fileContent.empty()) {
          std::cerr << "Failed to read file: " << filename << std::endl;
          return robotPath;
      }

      const std::vector<std::string> dataLines = readElement(fileContent, "\n");

      for (const std::string& line : dataLines) {
          if (line == "endData" || line == "endData\r") break;
          const std::vector<std::string> pointInput = readElement(line, ", ");
          if (pointInput.size() != 3) {
              std::cerr << "Invalid data format in line: " << line << std::endl;
              continue;
          }
          std::vector<double> pathPoint = {0, 0, 0};
          pathPoint[0] = std::stof(pointInput.at(0)); // x position
          pathPoint[1] = std::stof(pointInput.at(1)); // y position
          pathPoint[2] = std::stof(pointInput.at(2)); // velocity
          robotPath.push_back(pathPoint);
      }

      return robotPath;
  }

  // distance formula
  int findClosest(std::vector<double> pose, std::vector<std::vector<double>> path) {
      int closestPoint;
      float closestDist = 10000000000000;

      // loop through all path points
      for (int i = 0; i < path.size(); i++) {
          const float dist = sqrt(pow((pose[0] - path.at(i)[0]),2) + pow((pose[1] - path.at(i)[1]),2));;
          if (dist < closestDist) { // new closest point
              closestDist = dist;
              closestPoint = i;
          }
      }

      return closestPoint;
  }

  float circleIntersect(std::vector<double> p1, std::vector<double> p2, std::vector<double> pose, float lookaheadDist) {

      std::vector<double> d = subtract(p2, p1); // Direction vector of ray, from start to end
      std::vector<double> f = subtract(p1, pose); // Vector from center sphere to ray start
      /* Based on the equation t^2 * (d · d) + 2t*( f · d ) + ( f · f - r2 ) = 0
       * And solving for t
       */
      float a = multiply(d, d);
      float b = 2 * multiply(f, d);
      float c = multiply(f, f) - lookaheadDist * lookaheadDist;
      float discriminant = b * b - 4 * a * c;

      // if a possible intersection was found
      if (discriminant >= 0) {
          discriminant = sqrt(discriminant);
          float t1 = (-b - discriminant) / (2 * a);
          float t2 = (-b + discriminant) / (2 * a);

          // prioritize further down the path
          if (t2 >= 0 && t2 <= 1) return t2;
          else if (t1 >= 0 && t1 <= 1) return t1;
      }

      // no intersection found
      return -1;
  }

  /**
   * @brief returns the lookahead point
   *
   * @param lastLookahead - the last lookahead point
   * @param pose - the current position of the robot
   * @param path - the path to follow
   * @param closest - the index of the point closest to the robot
   * @param lookaheadDist - the lookahead distance of the algorithm
   */
  std::vector<double> lookaheadPoint(std::vector<double> lastLookahead, std::vector<double> pose, std::vector<std::vector<double>> path, int closest,
                              float lookaheadDist) {
      // optimizations applied:
      // only consider intersections that have an index greater than or equal to the point closest
      // to the robot
      // and intersections that have an index greater than or equal to the index of the last
      // lookahead point
      const int start = std::max(closest, int(lastLookahead[2]));
      for (int i = start; i < path.size() - 1; i++) {
          std::vector<double> lastPathPose = path.at(i);
          std::vector<double> currentPathPose = path.at(i + 1);

          float t = circleIntersect(lastPathPose, currentPathPose, pose, lookaheadDist);

          if (t != -1) {
              std::vector<double> lookahead = linearEnterp(lastPathPose, currentPathPose, t);
              lookahead[2] = i;
              return lookahead;
          }
      }

      // robot deviated from path, use last lookahead point
      return lastLookahead;
  }

  float findLookaheadCurvature(std::vector<double> pose, float heading, std::vector<double> lookahead) {
      // calculate whether the robot is on the left or right side of the circle
      float side = sgn(std::sin(heading) * (lookahead[0] - pose[0]) - std::cos(heading) * (lookahead[1] - pose[1]));
      // calculate center point and radius
      float a = -std::tan(heading);
      float c = std::tan(heading) * pose[0] - pose[1];
      float x = std::fabs(a * lookahead[0] + lookahead[1] + c) / std::sqrt((a * a) + 1);
      float d = std::hypot(lookahead[0] - pose[0], lookahead[1] - pose[1]);

      // return curvature
      return side * ((2 * x) / (d * d));
  }

  void follow(const char* path, float lookahead, int timeout, bool forwards, bool async) {
      if (async) {
          pros::Task task([&]() { follow(path, lookahead, timeout, forwards, false); });
          pros::delay(10); // delay to give the task time to start
          return;
      }

      std::vector<std::vector<double>> pathPoints = getData(path); // get list of path points
      if (pathPoints.size() == 0) {
          distTravelled = -1;
          return;
      }
      std::vector<double> pose = auton::getPose();
      std::vector<double> lastPose = pose;
      std::vector<double> lookaheadPose{0, 0, 0};
      std::vector<double> lastLookahead = pathPoints.at(0);
      lastLookahead[2] = 0;
      float curvature;
      float targetVel;
      float prevLeftVel = 0;
      float prevRightVel = 0;
      int closestPoint;
      float leftInput = 0;
      float rightInput = 0;
      float prevVel = 0;
      int compState = pros::competition::get_status();
      distTravelled = 0;
      float leftFF;
      float leftFB;
      float rightFF;
      float rightFB;

      // loop until the robot is within the end tolerance
      for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState; i++) {
          // get the current position of the robot
          pose = auton::getPose();
          if (!forwards) pose[2] -= M_PI;

          // update vars
          distTravelled += sqrt(pow((pose[0] - lastPose[0]),2) + pow((pose[1] - lastPose[1]),2));
          lastPose = pose;

          // find the closest point
          closestPoint = findClosest(pose, pathPoints);
          // stop at end of path
          if (pathPoints.at(closestPoint)[2] == 0) break;

          // find the lookahead
          lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, closestPoint, lookahead);
          lastLookahead = lookaheadPose; // update last look position

          // get curvature of the arc between robot and lookahead point
          float curvatureHeading = M_PI / 2 - pose[2];
          curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

          // get target velocity of robot
          targetVel = pathPoints.at(closestPoint)[2];
          targetVel = rateLimit(targetVel, prevVel, maxAccel);
          prevVel = targetVel;

          // calculate target left and right velocities
          float targetLeftVel = targetVel * (2 + curvature * trackWidth) / 2;
          float targetRightVel = targetVel * (2 - curvature * trackWidth) / 2;

          prevLeftVel = targetLeftVel;
          prevRightVel = targetRightVel;

          // feed forward and back math
          leftFF = Kv * targetLeftVel;
          leftFB = Kp * (targetLeftVel - leftMotorGroup.get_actual_velocities()[1]);
          rightFF = Kv * targetRightVel;
          rightFB = Kp * (targetRightVel - rightMotorGroup.get_actual_velocities()[1]);

          // move
          if (forwards) {
              leftMotorGroup.move_velocity(leftFF + leftFB);
              rightMotorGroup.move_velocity(leftFF + leftFB);
          } else {
              leftMotorGroup.move_velocity(-(leftFF + leftFB));
              rightMotorGroup.move_velocity(-(leftFF + leftFB));
          }

          pros::delay(10);
      }
      leftMotorGroup.move_velocity(0);
      rightMotorGroup.move_velocity(0);
      distTravelled = -1;
  }
}