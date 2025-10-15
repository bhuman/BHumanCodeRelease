/**
 * @file KeyframePhaseJointCompensationImpl.cpp
 * This file declares functions for the KeyframePhase
 * Content: Implementation of all methods, that are needed for the joint compensation feature
 * @author Philip Reichenberg
 */

#include "KeyframePhaseBase.h"
#include "Modules/MotionControl/KeyframeMotionEngine/KeyframeMotionEngine.h"
#include "Debugging/Plot.h"

void KeyframePhaseBase::calculateJointDifference()
{
  JointAngles lastRequest;
  FOREACH_ENUM(Joints::Joint, joint)
    lastRequest.angles[joint] = jointRequestOutput.angles[joint] != JointAngles::ignore && jointRequestOutput.angles[joint] != JointAngles::off && targetJoints.stiffnessData.stiffnesses[joint] != 0
                                ? jointRequestOutput.angles[joint]
                                : engine.theJointAngles.angles[joint];

  pastJointAnglesAngles.push_front(lastRequest);
  // Every Joint, that has a joint compensation, shall be checked if the jointDiff shall be predicted 3 frames into the future.
  std::array<bool, Joints::numOfJoints> predict;
  predict.fill(false);

  // Set flags whether the joint position shall be predicted
  auto setPredictionFlag = [&](const std::vector<JointCompensationParams>& jointCompensation)
  {
    for(JointCompensationParams list : jointCompensation)
    {
      if(!list.hipPitchDifferenceCompensation)
      {
        predict[isMirror ? Joints::mirror(list.jointDelta) : list.jointDelta] |= list.predictJointDiff;
        if(list.jointDelta == Joints::lHipYawPitch && isMirror)
          predict[list.jointDelta] |= list.predictJointDiff; // make sure the lHipYawPitch is always set. Needed for other parts in the balancer
      }
      else
      {
        predict[Joints::lHipPitch] = true;
        predict[Joints::rHipPitch] = true;
      }
    }
  };
  // Current keyframe
  setPredictionFlag(currentKeyframe.jointCompensation);
  // Same done for previous keyframe
  setPredictionFlag(lastKeyframe.jointCompensation);

  // Calculated current joint difference of "set" angles and "reached" angles.
  for(std::size_t i = 0; i < pastJointAnglesAngles.back().angles.size(); i++)
  {
    Angle diff = pastJointAnglesAngles.back().angles[i] - engine.theJointAngles.angles[i];
    Angle prevDiff = filteredJointDiffAngles[i].currentValue;
    filteredJointDiffAngles[i].update(pastJointAnglesAngles.back().angles[i] != JointAngles::off ? diff : 0_deg);

    jointDiffPredictedAngles.angles[i] = filteredJointDiffAngles[i].currentValue;

    if(predict[i])
    {
      // predict 3 frames into the future
      jointDiffPredictedAngles.angles[i] += 3.f * (filteredJointDiffAngles[i].currentValue - prevDiff);
      // don't overpredict, use a value that is as near as possible to 0_deg
      const Rangea clipPredictRange(std::min(Angle(filteredJointDiffAngles[i].currentValue), jointDiffPredictedAngles.angles[i]), std::max(Angle(filteredJointDiffAngles[i].currentValue), jointDiffPredictedAngles.angles[i]));
      jointDiffPredictedAngles.angles[i] = clipPredictRange.limit(0);
    }
  }

  PLOT("module:KeyframeMotionEngine:jointDiff1:lAnklePitch", jointDiffPredictedAngles.angles[Joints::lAnklePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rAnklePitch", jointDiffPredictedAngles.angles[Joints::rAnklePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lKneePitch", jointDiffPredictedAngles.angles[Joints::lKneePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rKneePitch", jointDiffPredictedAngles.angles[Joints::rKneePitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipPitch", jointDiffPredictedAngles.angles[Joints::lHipPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rHipPitch", jointDiffPredictedAngles.angles[Joints::rHipPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipYawPitch", jointDiffPredictedAngles.angles[Joints::lHipYawPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderPitch", jointDiffPredictedAngles.angles[Joints::lShoulderPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderPitch", jointDiffPredictedAngles.angles[Joints::rShoulderPitch].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lAnkleRoll", jointDiffPredictedAngles.angles[Joints::lAnkleRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rAnkleRoll", jointDiffPredictedAngles.angles[Joints::rAnkleRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lShoulderRoll", jointDiffPredictedAngles.angles[Joints::lShoulderRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rShoulderRoll", jointDiffPredictedAngles.angles[Joints::rShoulderRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowRoll", jointDiffPredictedAngles.angles[Joints::lElbowRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowRoll", jointDiffPredictedAngles.angles[Joints::rElbowRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lElbowYaw", jointDiffPredictedAngles.angles[Joints::lElbowYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rElbowYaw", jointDiffPredictedAngles.angles[Joints::rElbowYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lWristYaw", jointDiffPredictedAngles.angles[Joints::lWristYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rWristYaw", jointDiffPredictedAngles.angles[Joints::rWristYaw].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:lHipRoll", jointDiffPredictedAngles.angles[Joints::lHipRoll].toDegrees());
  PLOT("module:KeyframeMotionEngine:jointDiff1:rHipRoll", jointDiffPredictedAngles.angles[Joints::rHipRoll].toDegrees());
}

void KeyframePhaseBase::removePreviousKeyframeJointCompensation()
{
  if(!lastKeyframe.jointCompensation.empty())
  {
    JointRequest dummy;
    dummy.angles = lastKeyframeLineJointRequest.angles;
    const std::vector<std::pair<JointCompensationParams, JointAngles>> updatedLastKeyframeCompensation = applyJointCompensation(lastKeyframe.jointCompensation, dummy);
    ASSERT(updatedLastKeyframeCompensation.size() == lastKeyframeCompensation.size());

    for(std::size_t index = 0; index < updatedLastKeyframeCompensation.size(); index ++)
    {
      JointCompensationParams& params = lastKeyframeCompensation[index].first;
      JointAngles& oldCompensation = lastKeyframeCompensation[index].second;

      const JointAngles& newCompensation = updatedLastKeyframeCompensation[index].second;
      ASSERT(updatedLastKeyframeCompensation[index].first.jointDelta == params.jointDelta);

      // we compensate the asymmetry in the current motion. no need to handle overcompensation
      if(params.hipPitchDifferenceCompensation)
        continue;

      // Update the compensation and remove them from the startJoints.
      for(JointPair jointPair : params.jointPairs)
      {
        const Joints::Joint joint = isMirror ? Joints::mirror(jointPair.joint) : jointPair.joint;
        // We do not need to check the predictJointDiff flag, because jointDiffPredicted values already do this job
        if(lastKeyframeLineJointRequest.angles[joint] != JointAngles::off && lastKeyframeLineJointRequest.angles[joint] != JointAngles::ignore && targetJoints.stiffnessData.stiffnesses[joint] != 0 // special case
           && lastOutputLineJointRequestAngles.angles[joint] != JointAngles::off && lastOutputLineJointRequestAngles.angles[joint] != JointAngles::ignore) // special case
        {
          startJoints.angles[joint] += newCompensation.angles[joint] - oldCompensation.angles[joint];
          oldCompensation.angles[joint] = newCompensation.angles[joint];
        }
      }
    }
  }
}

static float constexpr scaleWithRange(const Rangea& range, const Angle& val)
{
  float scaling = 0.f;
  if(range.min * range.max < 0) // scale from 0 to min and 0 to max, and use higher resulting value
  {
    ASSERT(range.max >= range.min);
    const float minScale = range.min != 0 ? val / range.min : 0.f;
    const float maxScale = range.max != 0 ? val / range.max : 0.f;
    scaling = std::max(minScale, maxScale);
  }
  // Normal case. scale from min to max
  else
    scaling = (val - range.min) / (range.max - range.min);
  return std::max(scaling, 0.f);
  // TODO clip scaling to 1. Otherwise with a range for example of Rangea(4deg,8deg) does not scale linear for ratio > 1,
  // but exponentially! This could result in overcompensation!
}

std::vector<std::pair<JointCompensationParams, JointAngles>> KeyframePhaseBase::applyJointCompensation(const std::vector<JointCompensationParams>& jointCompensation, JointRequest& request)
{
  // Add jointCompensation for the current keyframe
  std::vector<std::pair<JointCompensationParams, JointAngles>> appliedCompensation;
  if(engine.stepKeyframes)
    return appliedCompensation;
  for(const JointCompensationParams& list : jointCompensation)
  {
    JointAngles compensation;
    const Joints::Joint jointDelta = isMirror ? Joints::mirror(list.jointDelta) : list.jointDelta;
    for(const JointPair& jointPair : list.jointPairs)
    {
      // when hipPitch difference compensation is active, then we can not use the predicted joint angles
      const Angle& refAngle = !list.hipPitchDifferenceCompensation ? jointDiffPredictedAngles.angles[jointDelta] : Angle(engine.theJointAngles.angles[Joints::lHipPitch] - engine.theJointAngles.angles[Joints::rHipPitch]);

      Rangea useRange = list.range;
      if(isMirror && (jointDelta == Joints::lAnkleRoll || jointDelta == Joints::rAnkleRoll || jointDelta == Joints::lHipRoll || jointDelta == Joints::rHipRoll))
      {
        useRange.min = -list.range.min;
        useRange.max = -list.range.max;
      }
      float percent = scaleWithRange(useRange, refAngle);

      Joints::Joint joint = jointPair.joint;
      if(!list.hipPitchDifferenceCompensation && isMirror)  // HipPitch difference compensation needs no mirror, because the sign of the difference handles it
      {
        joint = Joints::mirror(joint);
        if(joint == Joints::lAnkleRoll || joint == Joints::rAnkleRoll || joint == Joints::lHipRoll || joint == Joints::rHipRoll)
          percent *= -1.f;
      }
      if(list.hipPitchDifferenceCompensation)
        percent = std::min(percent, 1.f);
      request.angles[joint] += percent * jointPair.scaling * useRange.max;
      compensation.angles[joint] = percent * jointPair.scaling * useRange.max;
    }
    appliedCompensation.push_back({ list, compensation });
  }
  return appliedCompensation;
}
