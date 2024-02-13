#pragma once

#include <stdint.h>
#include <cstddef>
#include <vector>
#include <string>
#include <valarray>
#include "main.h"
#include "util/odom.h"
#include "auton.h"

namespace purepur{
  float rateLimit(float target, float current, float maxChange);
  template <class T> inline int sgn(T v);
  std::vector<double> linearEnterp(std::vector<double> pos, std::vector<double> other, float t);
  std::vector<double> subtract(std::vector<double> v1, std::vector<double> v2);
  float multiply(std::vector<double> v1, std::vector<double> v2);
  std::vector<std::string> readElement(const std::string& input, const std::string& delimiter);
  std::vector<std::vector<double>> getData(const char* filename);
  int findClosest(std::vector<double> pose, std::vector<std::vector<double>> path);
  float circleIntersect(std::vector<double> p1, std::vector<double> p2, std::vector<double> pose, float lookaheadDist);
  std::vector<double> lookaheadPoint(std::vector<double> lastLookahead, std::vector<double> pose, std::vector<std::vector<double>> path, int closest, float lookaheadDist);
  float findLookaheadCurvature(std::vector<double> pose, float heading, std::vector<double> lookahead);
  void follow(const char* filename, float lookahead, int timeout, bool forwards = true, bool async = true);
}