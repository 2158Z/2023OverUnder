#include "main.h"
#include "util/purepursuit.h"
#include "auton.h"

double ppDisplacementX;
double ppDisplacementY;
double ppDistanceError;
double ppTurnError;
double ppHeadingGoal;
double ppHeadingCont;
double ppTurnPreviousError;
double ppTurnKP = 1000;
double ppTurnKD = 0;
double ppHeadingCart;
bool ppFinalPoint = false;

double normalize(double value, double start, double end){
  double width = end - start;   //
  double offsetValue = value - start;   // value relative to 0

  return (offsetValue-(floor(offsetValue/width)*width))+start ;
  // + start to reset back to start of original range
}

// deg to radian from motors
double imuTheta(){
  double x = (cos(auton::get_left_deg()*M_PI/180 + M_PI) + cos(auton::get_right_deg()*M_PI/180 + M_PI)) / 2;
  double y = (sin(auton::get_left_deg()*M_PI/180 + M_PI) + sin(auton::get_right_deg()*M_PI/180 + M_PI)) / 2;

  return std::abs(atan2f(y, x) + M_PI);
}

//PID CONSTRUCTORS
pureID::pureID(double kP, double kD, double target){
  pkP = kP;
  pkD = kD;
  pTarget = target;
}

pureID::pureID(double kP, double kD, double targetX, double targetY, double maxSpeed, double preferredAngle){
  pkP = kP;
  pkD = kD;
  pTargetX = targetX;
  pTargetY = targetY;
  pMaxSpeed = maxSpeed;
  pPreferredAngle = preferredAngle;
}

//PID variables
bool toggle = true;
bool toggle2 = true;
double displacementX;
double displacementY;
double headingGoal;
double headingStart;
double headingCont;
double headingCart;
double turnPreviousError;
double turnCount;
double errorCount;
double turnError;
double forwardPreviousError;
double distanceCount;
double distanceError;
double turnIntegral;
double turnKP = 500; //turn correction proportional
double turnKI = 0; //turn correction integral
double turnKD = 50; //turn correction derivative
double distancePreviousError;
double xTarget;
double yTarget;

void pureID::StraightPID(){
  toggle = true;
  toggle2 = true;
  while(1){
  pros::lcd::print(5,"X: ""%f", auton::get_X_position());
	pros::lcd::print(6,"Y: ""%f", auton::get_Y_position());
	pros::lcd::print(7,"THETA: " "%f", theta*180/M_PI);

  double xStart;
  double yStart;
  double calcDistance;
  xTarget = pTargetX;
  yTarget = pTargetY;
  if(toggle == true){
    xStart = auton::get_X_position();
    yStart = auton::get_Y_position();
    calcDistance = sqrt(pow((xStart - xTarget),2) + pow((yStart - yTarget),2));
    headingStart = imuTheta()*180/M_PI;
    std::cout << "xStart " << xStart << std::endl;
    std::cout << "xTarget " << xTarget << std::endl;
    std::cout << "yStart " << yStart << std::endl;
    std::cout << "yTarget " << yTarget << std::endl;
    toggle = false;
  }
//For personal use and entering values, it makes more sense to use the clock method, but translations for pure pursuit is easier
//with cartesian plane
//headingCart is heading cartesion
  headingCart = normalize(-imuTheta()*180/M_PI+450, 0, 360);

  displacementX = xTarget - auton::get_X_position();
  displacementY = yTarget - auton::get_Y_position();

  double displacementX_local = displacementY*cos(headingCart/180*3.1415926535) - displacementX*sin(headingCart/180*3.1415926535);
  double displacementY_local = displacementY*sin(headingCart/180*3.1415926535) + displacementX*cos(headingCart/180*3.1415926535);

  double movementXratio = displacementX_local/(fabs(displacementX_local)+fabs(displacementY_local));
  double movementYratio = displacementY_local/(fabs(displacementX_local)+fabs(displacementY_local));

  //basic pythagorean thing calculating hypotenuse
  distanceError = sqrt(pow((auton::get_X_position()-xTarget),2) + pow((auton::get_Y_position()-yTarget),2));
  std::cout << "xRatio " << movementXratio << std::endl;
  std::cout << "yRatio " << movementYratio << std::endl;

  //if there isnt a preferredAngle
  if(pPreferredAngle == -1){
    //if robot is within 3 inches of the target point
    if(distanceError < 3  &&  toggle2 == true){
      headingCont = imuTheta()*180/M_PI; //headingCont is heading of robot in degrees
      toggle2 = false;
    }
    if(distanceError < 3){
      headingGoal = headingCont;
    }else{
    //get heading from opposite/hypotenuse then convert to degrees
    headingGoal = (normalize((atan2(yTarget - auton::get_Y_position(),xTarget - auton::get_X_position())*180/3.1415926) - 90, 0, 360) - 360)* -1;
    }
  }else{//the goal of the robot is the preferred angle
    headingGoal = pPreferredAngle;
  }

  //headingGoal - imuTheta()*180/M_PI how much the robot needs to turn to face its headingGoal
  //these if statements optimize it as the robot only needs to turn 180 degrees maximum to reach any heading
  //but if the goal is to face 5 degrees and the robot is facing 355, the error is 355 degrees.
  if(headingGoal - imuTheta()*180/M_PI > 180){
    turnError = headingGoal - imuTheta()*180/M_PI-360;
  }
  //simplest calculation is possible the error is just headingGoal-robotHeading
  if(-180 < headingGoal - imuTheta()*180/M_PI  &&  headingGoal - imuTheta()*180/M_PI <= 180){

    turnError = headingGoal-imuTheta()*180/M_PI;
  }
  //
  if(headingGoal - imuTheta()*180/M_PI <= -180){
    turnError = headingGoal-imuTheta()*180/M_PI+360;
  }

  double turnDerivative = turnError - turnPreviousError;
  double distanceDerivative = distanceError - distancePreviousError;

  if(distanceError < 10 && distanceError > -10){
    distanceCount += 1;
  }
  else { //If It isn't within +/- 10
    distanceCount = 0;
  }

  if (distanceError < 10 && distanceError > -10){
    errorCount += 1;
  }

  else { //If It isn't within +/- 10
    errorCount = 0;
  }
  //end PID loop, sometimes the PID will get stuck in a loop forever.
  if(distanceCount>=30 || errorCount>=80){
    distanceCount=0;
    errorCount=0;
    toggle = true;
    auton::drive_with_voltage(0, 0);
    break;
  }

  if(turnError < 1 && turnError > -1){
    turnIntegral = 0; //I don't need integral if the turn error is within +/- 1
  }else{//If It isn't within +/- 1
    turnIntegral += turnError;
  }
  //calculating the speeds for the motors. turn error has been calculated and drive error has been calculated
  //in the most basic PID speed = error. There is a multiplier added for tuning (kP, kI, kD) hence PID.
  double mVdistance = distanceError * pkP + distanceDerivative * pkD;
  //std::cout << "mVdistance " << mVdistance << std::endl;
  double mVforward = movementYratio*mVdistance;
  double mVturn = turnError * turnKP + turnDerivative * turnKD + turnIntegral * turnKI;
  double maxCtrl =
  fmax(fmax(fabs(-mVforward + mVturn), fabs(mVforward + mVturn)),
  fmax(fabs(mVforward - mVturn), fabs(-mVforward - mVturn)));
  //maxCtrl is finding the max speed of the speed values calculated.
  //each motor has its respective speed, and if its speed goes above 12000 then the motor has maxed out
  //if its greater, then it caps the speed

  if(maxCtrl>12000){
    auton::drive_with_voltage((-mVforward + mVturn) / (maxCtrl / 12000)*pMaxSpeed, (mVforward - mVturn) / (maxCtrl / 12000)*pMaxSpeed);

  } else {//just default to what the code wants
    auton::drive_with_voltage((-mVforward + mVturn)*pMaxSpeed, (mVforward - mVturn)*pMaxSpeed);
  }

  turnPreviousError = turnError;
  distancePreviousError = distanceError;
  }
}
APPC::APPC(std::vector<std::array<double,3>> pathVector, int resolution, double lookaheadDistance){
  pPathVector = pathVector; //when calling the class this is the vector of points (x,y,theta) because of my limited drive, theta is-1
  pResolution = resolution; //Resolution of the points generated. distance between each point
  pLookaheadDistance = lookaheadDistance; //how far the robot looks ahead to the path
}

void APPC::PurePursuit(){

  std::array<double,3> targetPoint = {0,0,0};
  std::vector<std::array<double,3>> subpoints;
  //std::cout << "Subpoint Calculations" << std::endl;
  //Subpoint Calculations
  for(int i = 0; i < pPathVector.size() - 1; i++){
    //gets the starting coordinates and ending coordinates from user input in main.cpp
    std::array<double,3> coordStart = {std::get<0>(pPathVector[i]),std::get<1>(pPathVector[i]),std::get<2>(pPathVector[i])};
    std::array<double,3> coordEnd = {std::get<0>(pPathVector[i+1]),std::get<1>(pPathVector[i+1]),std::get<2>(pPathVector[i+1])};
    double xDistance = coordEnd[0]-coordStart[0];
    double yDistance = coordEnd[1]-coordStart[1];

    subpoints.push_back(coordStart);

    //std::cout << coordStart[0] << ", " << coordStart[1] << std::endl;
    //point generation
    for(int i2=1; i2<pResolution; i2++){

      std::array<double,3> newPoint =
      {coordStart[0]+(xDistance/pResolution*i2),coordStart[1]+(yDistance/pResolution*i2),coordStart[2]};

      subpoints.push_back(newPoint);

      //std::cout << newPoint[0] << ", " << newPoint[1] << std::endl;
    }
    //last coordStart
    if(i == pPathVector.size()-2){
      subpoints.push_back(coordEnd);
      //std::cout << coordEnd[0] << ", " << coordEnd[1] << std::endl;
    }
  }
  int i3;

  // std::cout << "Amount of Points: " << subpoints.size() << std::endl;
  // std::cout << std::endl;
  // std::cout << "Target Points: " << std::endl;

  std::array<double,3> previousPoint = {std::get<0>(pPathVector[0]),std::get<0>(pPathVector[1]),std::get<0>(pPathVector[2])};
  double targetDistance = pLookaheadDistance+10;

  while(1){
    double previousSubpointDistance=1000;
    int targetPosition;
    int previousTargetPosition = 10;

    i3+=1;

    double finalDistance = sqrt(pow((auton::get_X_position()-std::get<0>(pPathVector[pPathVector.size()-1])),2)+pow((auton::get_Y_position()-std::get<1>(pPathVector[pPathVector.size()-1])),2));

    //Find the closest point
    //loop through all the subpoints and find the closest one.
    if(finalDistance > pLookaheadDistance+1){
      for(int i=0; i<subpoints.size(); i++){
        double subpointX = std::get<0>(subpoints[i]);
        double subpointY = std::get<1>(subpoints[i]);
        double subpointHeading = std::get<2>(subpoints[i]);
        double subpointDistance = sqrt(pow((auton::get_X_position()-subpointX),2)+pow((auton::get_Y_position()-subpointY),2));
        double subpointDistanceDifference = subpointDistance-pLookaheadDistance;
        double previousSubpointDifference = previousSubpointDistance-pLookaheadDistance;
        if(subpointDistanceDifference < previousSubpointDifference && subpointDistanceDifference > 0){
          previousSubpointDistance = subpointDistance;
          targetPoint = {subpointX,subpointY,subpointHeading};
          targetPosition = i;
        }
      }

      for(int i = 0; i < targetPosition; i++){
        if(i!=subpoints.size()-1){
          std::get<0>(subpoints[i]) = -1000;
          std::get<1>(subpoints[i]) = -1000;
          std::get<2>(subpoints[i]) = 0;
        }
      }

      if(targetPosition >= previousTargetPosition+4 && targetDistance > pLookaheadDistance){
        targetPoint = previousPoint;
      }
      targetDistance = sqrt(pow((auton::get_X_position()-targetPoint[0]),2)+pow((auton::get_Y_position()-targetPoint[1]),2));

      previousPoint = targetPoint;
      previousTargetPosition = targetPosition;
      }else {
      ppFinalPoint = true;
      targetPoint = {std::get<0>(subpoints[subpoints.size()-1]),std::get<1>(subpoints[subpoints.size()-1]),std::get<2>(subpoints[subpoints.size()-1])};
    }

    //Movement Section head to the nearest point
    double pp_turnDerivative = ppTurnError - ppTurnPreviousError;
    //Vectors
    //heading of the robot in cartesian
    ppHeadingCart = normalize(-auton::get_absolute_heading()+450, 0, 360);

    //deltaX and deltaY
    ppDisplacementX = targetPoint[0]-auton::get_X_position();
    ppDisplacementY = targetPoint[1]-auton::get_Y_position();

    //find the displacement relative to the robot because it may not be facing the points
    //there are 2 vectors, how much to go forwards, how much to strafe
    double ppDisplacementXLocal =
    ppDisplacementY*cos(ppHeadingCart/180*3.1415926535) - ppDisplacementX*sin(ppHeadingCart/180*3.1415926535);
    //-50
    double ppDisplacementYLocal =
    ppDisplacementY*sin(ppHeadingCart/180*3.1415926535) + ppDisplacementX*cos(ppHeadingCart/180*3.1415926535);

    //quantifying the vecotr, numbers between 0 and 1
    double pp_movementXratio = ppDisplacementXLocal/(fabs(ppDisplacementXLocal)+fabs(ppDisplacementYLocal));
    double pp_movementYratio = ppDisplacementYLocal/(fabs(ppDisplacementXLocal)+fabs(ppDisplacementYLocal));

    //simple pythag
    ppDistanceError = sqrt(pow((auton::get_X_position()-targetPoint[0]),2)+pow((auton::get_Y_position()-targetPoint[1]),2));

    //Output every 50ms i was using it for testing.
    // if(i3>=5){
    //   // std::cout << "Target: " <<targetPoint[0] << ", " << targetPoint[1] << ", " << targetPoint[2] << std::endl;
    //   // std::cout << "Distance: " << targetDistance << std::endl;
    //   i3=0;
    // }

    //Heading is undefined theres no desired robot heading so it can calculate what it wants.
    if(targetPoint[2] == -1){
      ppHeadingGoal = (normalize((atan2(targetPoint[1]-auton::get_Y_position(),targetPoint[0]-auton::get_X_position())*180/3.1415926)-90, 0, 360)-360)*-1;
    } else {
      ppHeadingGoal = targetPoint[2];
    }

    //optimize the direction to turn like before
    if(ppHeadingGoal-theta > 180){
      ppTurnError = ppHeadingGoal-theta-360;
    }
    if(-180 < ppHeadingGoal - theta && ppHeadingGoal - theta <= 180){
      ppTurnError = ppHeadingGoal-theta;
    }
    if(ppHeadingGoal - theta <= -180){
      ppTurnError = ppHeadingGoal-theta+360;
    }

    if(ppFinalPoint == true){//it has almost reached its final point
      pureID PathFollowing(5000, 500, targetPoint[0], targetPoint[1], 1, targetPoint[2]);
      PathFollowing.StraightPID();

      ppFinalPoint = false;
      subpoints.clear();
      pPathVector.clear();
      break;
    }

    //Motor Control
    double turnPower = ppTurnError * ppTurnKP + pp_turnDerivative * ppTurnKD;
    double pp_maxCtrl =fmax(
    fmax(fabs(-pp_movementYratio + turnPower/12000 + pp_movementXratio), fabs(pp_movementYratio + turnPower/12000 + pp_movementXratio)),
    fmax(fabs(pp_movementYratio - turnPower/12000 + pp_movementXratio), fabs(-pp_movementYratio - turnPower/12000 + pp_movementXratio))
    );

    //terminal output for debugging
    printf("L: %f\n, R: %f\n", -((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl), (pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl);

    if(pp_maxCtrl > 1){
      auton::drive_with_voltage(-((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl), (pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000/pp_maxCtrl);

    }else {
      auton::drive_with_voltage(-((-pp_movementYratio + turnPower/12000 + pp_movementXratio)*12000), (pp_movementYratio - turnPower/12000 + pp_movementXratio)*12000);
    }

    //Variables to get derivatives for next loop. change in unit/10ms
    previousPoint = targetPoint;
    previousTargetPosition = targetPosition;
    ppTurnPreviousError = ppTurnError;
    pros::delay(10);
  }
}