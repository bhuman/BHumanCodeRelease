/**
 * @file WalkLearnerProvider.h
 * This file declares a module that learns joint offsets for the walking.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/WalkModifier.h"
#include "Representations/Sensing/JointPlay.h"
#include "Framework/Module.h"
#include <vector>
#include "Debugging/DebugDrawings.h"

MODULE(WalkLearnerProvider,
{,
  REQUIRES(InertialData),
  REQUIRES(JointPlay),
  REQUIRES(WalkModifier),
  PROVIDES(WalkLearner),
  DEFINES_PARAMETERS(
  {,
    (Rangef)({30.f, 1.f}) adjustmentRange,
    (float)(20.f) adjustmentSteps,
    (float)(15.f) minForwardStep,
    (float)(30.f) maxSideStep,
    (Angle)(15_deg) maxTurnStep,
    (float)(1.2f) clipStepDurationRatio,
    (int)(2) maxStepsForJointPlayInitializing,
    (float)(240.f) jointPlayInitializing,
    (Rangef)({1.f, 1.07f}) bestStepDuration,
  }),
});

class WalkLearnerProvider : public WalkLearnerProviderBase
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
  int learnIteration; //learnIteration. If > theWalkModifier.numOfGyroPeaks, then change balance factor
  bool isForwardPhase; //are we searching for the current positive gyro max value?
  bool wasPositiv; //gyro changed sign

  std::vector<float> gyroForwardMax, //save the last theWalkModifier.numOfGyroPeaks gyro peaks
      gyroBackwardMin;

  RingBufferWithSum<float, 10> stepDurationBuffer;
  int adjustmentCounter = 0;

  void update(WalkLearner& walkLearner) override;

  //learn method
  void learnGyroBalanceFactor(WalkLearner& walkLearner);

  //This method is needed so the WalkLearnerProvider knows the starting balancing values and the current walking speed.
  void setBaseWalkParams(float gyroForward, float gyroBackward, float speedTransX);

  void learnStepHeightDuration(const Pose2f& lastStep, const float lastDuration, const float refStepDuration, float& currentAdjustment);

public:
  WalkLearnerProvider();
};
