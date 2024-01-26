#pragma once

#include "main.h"
#include "global-variables.h"
#include <vector>
#include <math.h>

using namespace std;

class pureID{
  double pkP;
  double pkD;
  double pTarget;
  double pTargetX;
  double pTargetY;
  double pMaxSpeed;
  double pPreferredAngle;

  public:
    pureID(double kP, double kD, double target);

    pureID(double kP, double kD, double targetX, double targetY, double maxSpeed, double preferredAngle);

    void turnPID();
    void StraightPID();

};

class APPC{

  std::vector<std::array<double,3>> pPathVector;
  int pResolution;
  double pLookaheadDistance;
  bool pMoreIntake;

  public:
  APPC(std::vector<std::array<double,3>> pathVector, int resolution, double lookaheadDistance);
  void PurePursuit();

};