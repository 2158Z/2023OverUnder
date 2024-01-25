#include "main.h"

class APPC{

  std::vector<std::array<double,3>> pPathVector;
  int pResolution;
  double pLookaheadDistance;
  bool pMoreIntake;

  public:
  APPC(std::vector<std::array<double,3>> pathVector, int resolution, double lookaheadDistance);
  void PurePursuit(Odom odom);

};