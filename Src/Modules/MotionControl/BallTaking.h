/**
* @file BallTaking.h
* @author Thomas Muender
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/BallTakingOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"

#include "Tools/RingBufferWithSum.h"

MODULE(BallTaking)
  REQUIRES(MotionSelection)
  REQUIRES(FilteredJointData)
  REQUIRES(FilteredSensorData)
  REQUIRES(RobotDimensions)
  REQUIRES(MassCalibration)
  REQUIRES(RobotModel)
  REQUIRES(FrameInfo)
  REQUIRES(BallModel)
  PROVIDES_WITH_MODIFY(BallTakingOutput)
  DEFINES_PARAMETER(int, phaseDuration, 600)

  DEFINES_PARAMETER(float, xFactor, 50.f)
  DEFINES_PARAMETER(float, rotXFactor, 0.5f)
  DEFINES_PARAMETER(float, zStandFactor, 5.f)
  DEFINES_PARAMETER(float, y2Factor, 0.4f)
  DEFINES_PARAMETER(float, rot, 1.396f)
END_MODULE

class BallTaking : public BallTakingBase
{
private:

  //general stuff
  float side;
  int phase;
  unsigned phaseStart;
  bool phaseLeavingPossible;

  //balldata
  bool valid;
  Vector2<> crossing;
  RingBufferWithSum<float, 40> moved;
  RingBufferWithSum<char, 20> takable;
  
  //phase 0
  int c;

  //phase 1
  float y;
  float zStand;
  float rotZ;
  float rotX;
  float y2;

  //foot pose
  Pose3D standLeftFoot, standRightFoot;
  Pose3D targetLeftFootPositon, targetRightFootPositon;
  Pose3D leftEndPhaseOne, rightEndPhaseOne;
  Pose3D leftEndPhaseThree, rightEndPhaseThree;
  Pose3D leftEndPhaseFour, rightEndPhaseFour;
  Pose3D leftEndPhaseFive, rightEndPhaseFive;

public:

  BallTaking() : side(0), phase(0), phaseStart(0), phaseLeavingPossible(false), c(0)
  {
    standLeftFoot = Pose3D(-11.4841f, 49.8687f, -181.846f);
    standRightFoot = Pose3D(-11.4841f, -50.1313f, -181.846f);
  }

  void ballData(BallTakingOutput& output);

  void update(BallTakingOutput& output);

  void phaseZero(BallTakingOutput& output);

  void phaseOne(BallTakingOutput& output);

  void phaseTwo(BallTakingOutput& output);

  void phaseThree(BallTakingOutput& output);
  
  void phaseFour(BallTakingOutput& output);
  
  void phaseFive(BallTakingOutput& output);

  float sinUp(const float& x)
  {
    if(x < 0) return 0.f;
    else if(x > 1) return 1.f;
    else return (std::sin((x * pi) - pi_2) + 1) * 0.5f;
  }
};
