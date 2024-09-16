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
  if(currentKeyframe.jointCompensation)
    for(JointCompensationParams list : currentKeyframe.jointCompensation->jointCompensationParams)
      predict[isMirror ? Joints::mirror(list.jointDelta) : list.jointDelta] |= list.predictJointDiff;

  // Calculated current joint difference of "set" angles and "reached" angles.
  for(std::size_t i = 0; i < pastJointAnglesAngles.back().angles.size(); i++)
  {
    Angle diff = pastJointAnglesAngles.back().angles[i] - engine.theJointAngles.angles[i];
    Angle prevDiff = jointDiff1Angles.angles[i];
    jointDiff1Angles.angles[i] = pastJointAnglesAngles.back().angles[i] != JointAngles::off
                                 ? 0.6f * prevDiff + 0.4f * diff // Low-Pass filter difference
                                 : 0.f;
    jointDiffPredictedAngles.angles[i] = jointDiff1Angles.angles[i];

    if(predict[i])
    {
      // predict 3 frames into the future
      jointDiffPredictedAngles.angles[i] += 3.f * (jointDiff1Angles.angles[i] - prevDiff);
      // don't overpredict, use a value that is as near as possible to 0_deg
      const Rangea clipPredictRange(std::min(jointDiff1Angles.angles[i], jointDiffPredictedAngles.angles[i]), std::max(jointDiff1Angles.angles[i], jointDiffPredictedAngles.angles[i]));
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
  JointRequest startDiff;
  for(unsigned int i = 0; i < Joints::numOfJoints; i++)
    startDiff.angles[i] = lastKeyframeLineJointRequest.angles[i] - lastOutputLineJointRequestAngles.angles[i]; // Diff of joint targets of last Keyframe with and without balancing and compensation

  // save last compensation reduce factors
  const auto jointCompensationReducerCopy = jointCompensationReducer;
  if(lastKeyframe.jointCompensation)
    // jointCompensation index must be 0, because the frameWork assumes, that only(!) the first entry is used.
    for(JointCompensationParams list : lastKeyframe.jointCompensation->jointCompensationParams)
    {
      // we compensate the asymmetry in the current motion. no need to handle overcompensation
      if(list.hipPitchDifferenceCompensation)
        continue;

      const Joints::Joint jointDelta = isMirror ? Joints::mirror(list.jointDelta) : list.jointDelta;
      // If joint started to move again then increase compensationReducer
      if(jointDiffPredictedAngles.angles[jointDelta] < jointDiff1Angles.angles[jointDelta]
         && jointDiff1Angles.angles[jointDelta] - jointDiffPredictedAngles.angles[jointDelta] > engine.minJointCompensationReduceAngleDiff)
        jointCompensationReducer[jointDelta] += lastKeyframe.jointCompensation->reduceFactorJointCompensation;

      // Update the compensation and remove them from the startJoints.
      for(JointPair jointPair : list.jointPairs)
      {
        const Joints::Joint joint = isMirror ? Joints::mirror(jointPair.joint) : jointPair.joint;
        // We do not need to check the predictJointDiff flag, because jointDiffPredicted values already do this job
        if(jointDiffPredictedAngles.angles[jointDelta] < jointDiff1Angles.angles[jointDelta] // stuck joint moves fast
           && jointDiff1Angles.angles[jointDelta] - jointDiffPredictedAngles.angles[jointDelta] > engine.minJointCompensationReduceAngleDiff // stuck joint moves fast enough (diff > threshold)
           && lastKeyframeLineJointRequest.angles[joint] != JointAngles::off && lastKeyframeLineJointRequest.angles[joint] != JointAngles::ignore && targetJoints.stiffnessData.stiffnesses[joint] != 0 // special case
           && lastOutputLineJointRequestAngles.angles[joint] != JointAngles::off && lastOutputLineJointRequestAngles.angles[joint] != JointAngles::ignore // special case
           && jointCompensationReducerCopy[jointDelta] < 1.f && jointCompensationReducerCopy[jointDelta] < jointCompensationReducer[jointDelta]) // make sure we don't erase more than 100% of the last jointCompensation and only reduce if the joint keeps moving.
        {
          // previous reduction value was < 1.f, now above 1.f. Apply difference to reach 100% reduction
          if(jointCompensationReducer[jointDelta] > 1.f)
            startJoints.angles[joint] += (1.f - jointCompensationReducerCopy[jointDelta]) * startDiff.angles[joint];
          else
            startJoints.angles[joint] += currentKeyframe.jointCompensation->reduceFactorJointCompensation * startDiff.angles[joint];
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

void KeyframePhaseBase::applyJointCompensation()
{
  // Add jointCompensation for the current keyframe
  lineJointRequest.angles = lineJointRequest2Angles.angles;
  if(currentKeyframe.jointCompensation)
    for(JointCompensationParams list : currentKeyframe.jointCompensation->jointCompensationParams)
    {
      const Joints::Joint jointDelta = isMirror ? Joints::mirror(list.jointDelta) : list.jointDelta;
      for(JointPair jointPair : list.jointPairs)
      {
        // when hipPitch difference compensation is active, then we can not use the predicted joint angles
        const Angle& refAngle = !list.hipPitchDifferenceCompensation ? jointDiffPredictedAngles.angles[jointDelta] : Angle(engine.theJointAngles.angles[Joints::lHipPitch] - engine.theJointAngles.angles[Joints::rHipPitch]);
        float percent = scaleWithRange(list.range, refAngle);

        Joints::Joint joint = jointPair.joint;
        if(!list.hipPitchDifferenceCompensation && isMirror) // HipPitch difference compensation needs no mirror, because the sign of the difference handles it
        {
          joint = Joints::mirror(joint);
          if(joint == Joints::lAnkleRoll || joint == Joints::rAnkleRoll)
            percent *= -1.f;
        }
        if(list.hipPitchDifferenceCompensation)
          percent = std::min(percent, 1.f);
        lineJointRequest.angles[joint] += percent * jointPair.scaling * list.range.max;
      }
    }
}
