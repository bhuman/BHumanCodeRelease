/**
 * @file WalkLearnerProvider.h
 * This file declares a module that learns joint offsets for the walking.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/Sensing/JointPlay.h"
#include "Framework/Module.h"

MODULE(WalkLearnerProvider,
{,
  REQUIRES(JointPlay),
  PROVIDES(WalkLearner),
  LOADS_PARAMETERS(
  {,
    (Rangef) adjustmentRange,
    (float) adjustmentSteps,
    (float) minForwardStep,
    (float) maxSideStep,
    (Angle) maxTurnStep,
    (float) clipStepDurationRatio,
    (int) maxStepsForJointPlayInitializing,
    (float) jointPlayInitializing,
    (Rangef) bestStepDuration,
  }),
});

class WalkLearnerProvider : public WalkLearnerProviderBase
{
private:
  RingBufferWithSum<float, 10> stepDurationBuffer;
  int adjustmentCounter = 0;

  void update(WalkLearner& walkLearner) override;

  void learnStepHeightDuration(const Pose2f& lastStep, const float lastDuration, const float refStepDuration, float& currentAdjustment);
};
