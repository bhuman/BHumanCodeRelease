/**
 * @file Walk2014GeneratorLearner.h
 * This file declares a module that learns joint offsets for the walking.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/Walk2014Modifier.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/WalkLearner.h"
#include <vector>
#include "Tools/Debugging/DebugDrawings.h"

MODULE(Walk2014GeneratorLearner,
{,
  REQUIRES(InertialData),
  REQUIRES(Walk2014Modifier),
  PROVIDES(WalkLearner),
});

class Walk2014GeneratorLearner : public Walk2014GeneratorLearnerBase
{
private:
  float gyroForwardBalanceFactor; //The current balancing value for balancing forward
  float gyroBackwardBalanceFactor; //The current balancing value for balancing backward
  float oldForwardGyro; // The old balancing value for balancing forward
  float oldAverageMaxGyro; // average positive max gyro.y value over the last x frames
  float oldAverageMaxGyroPhase1; // average positive max gyro.y value over the last x frames of Phase 2
  float oldBackwardGyro; // The old balancing value for balancing backward
  float oldAverageMinGyro; // average negativ max gyro.y value over the last x frames
  float oldAverageMinGyroPhase1; // average negative max gyro.y value over the last x frames of Phase 2
  float speed; // speed of the current walk in x-coordinate

  int phaseLearn; //learn phase
  int learnIteration; //learnIteration. If > theWalk2014Modifier.numOfGyroPeaks, then change balance factor
  bool isForwardPhase; //are we searching for the current positive gyro max value?
  bool wasPositiv; //gyro changed sign

  std::vector<float> gyroForwardMax, //save the last theWalk2014Modifier.numOfGyroPeaks gyro peaks
      gyroBackwardMin;

  void update(WalkLearner& walkLearner) override;

  //learn method
  void learnGyroBalanceFactor(WalkLearner& walkLearner);

  //This method is needed so the Walk2014GeneratorLearner knows the starting balancing values and the current walking speed.
  void setBaseWalkParams(float gyroForward, float gyroBackward, float speedTransX);

public:
  Walk2014GeneratorLearner();
};
