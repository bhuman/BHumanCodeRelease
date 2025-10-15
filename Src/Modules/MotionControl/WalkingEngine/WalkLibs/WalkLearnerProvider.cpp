/**
 * @file WalkLearnerProvider.cpp
 *
 * @author Philip Reichenberg
 */

#include "WalkLearnerProvider.h"

MAKE_MODULE(WalkLearnerProvider);

void WalkLearnerProvider::update(WalkLearner& walkLearner)
{
  walkLearner.updateStepDuration = [this, &walkLearner](const Pose2f& lastStep, const float lastDuration, const float refStepDuration)
  {
    learnStepHeightDuration(lastStep, lastDuration, refStepDuration, walkLearner.stepHeightDurationOffset);
  };
}

void WalkLearnerProvider::learnStepHeightDuration(const Pose2f& lastStep, const float lastDuration, const float refStepDuration, float& currentAdjustment)
{
  if(std::abs(lastStep.rotation) > maxTurnStep || std::abs(lastStep.translation.y()) > maxSideStep || std::abs(lastStep.translation.x()) < minForwardStep)
    return;

  stepDurationBuffer.push_front(std::min(lastDuration, refStepDuration * clipStepDurationRatio));
  if(stepDurationBuffer.full() && theJointPlay.isCalibrated)
  {
    if(adjustmentCounter < maxStepsForJointPlayInitializing)
      currentAdjustment = (1.f - theJointPlay.qualityOfRobotHardware) * jointPlayInitializing;
    else
    {
      float adjustment = mapToRange(Rangef::ZeroOneRange().limit(adjustmentCounter / adjustmentSteps), 0.f, 1.f, adjustmentRange.min, adjustmentRange.max);
      const float useRefStepDuration = refStepDuration * mapToRange((1.f - theJointPlay.qualityOfRobotHardware), 0.f, 1.f, bestStepDuration.min, bestStepDuration.max);
      if(stepDurationBuffer.average() > useRefStepDuration)
        adjustment *= -1.f;
      currentAdjustment += adjustment;
    }

    currentAdjustment = std::max(0.f, currentAdjustment);

    adjustmentCounter++;
    stepDurationBuffer.clear();
  }
}
