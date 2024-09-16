/**
 * @file KeyframePhaseConditionsImpl.cpp
 * This file declares functions for the KeyframePhase
 * Content: Implementation of all methods, that are needed to handle conditional keyframes
 * @author Philip Reichenberg
 */

#include "KeyframePhaseBase.h"
#include "Modules/MotionControl/KeyframeMotionEngine/KeyframeMotionEngine.h"

template<class C>
bool KeyframePhaseBase::checkConditions(const std::vector<C>& conditions, bool useAnd)
{
  static_assert(std::is_assignable<Condition, C>::value, "Incompatible types!");

  std::size_t conditionSize = conditions.size();
  if(conditionSize == 0)
    return true;
  // Write the values for the conditions
  float variableValues[numOfConditionVars];
  variableValues[ConditionVar::InertialDataAngleY] = engine.theInertialData.angle.y().toDegrees();
  variableValues[ConditionVar::InertialDataAngleAbsoluteX] = std::abs(engine.theInertialData.angle.x().toDegrees());
  variableValues[ConditionVar::FluctuationY] = fluctuation.y().toDegrees() * 1.f / Constants::motionCycleTime;
  variableValues[ConditionVar::ShoulderPitchLeft] = engine.theJointAngles.angles[Joints::lShoulderPitch].toDegrees();
  variableValues[ConditionVar::ShoulderPitchRight] = engine.theJointAngles.angles[Joints::rShoulderPitch].toDegrees();
  variableValues[ConditionVar::ShoulderRollLeft] = engine.theJointAngles.angles[Joints::lShoulderRoll].toDegrees();
  variableValues[ConditionVar::ShoulderRollRight] = engine.theJointAngles.angles[Joints::rShoulderRoll].toDegrees();
  variableValues[ConditionVar::WristTranslationYLeft] = engine.theRobotModel.limbs[Limbs::wristLeft].translation.y();
  variableValues[ConditionVar::WristTranslationYRight] = engine.theRobotModel.limbs[Limbs::wristRight].translation.y();
  variableValues[ConditionVar::WaitTime] = static_cast<float>(engine.theFrameInfo.getTimeSince(waitTimestamp));
  variableValues[ConditionVar::FrontHack] = frontHackTriggered ? 1.f : 0.f;
  variableValues[ConditionVar::IsSitting] = std::min(engine.theJointAngles.angles[Joints::lKneePitch], engine.theJointAngles.angles[Joints::rKneePitch]).toDegrees();
  variableValues[ConditionVar::HYPDifference] = std::abs(engine.theJointAngles.angles[Joints::lHipYawPitch].toDegrees() - jointRequestOutput.angles[Joints::lHipYawPitch].toDegrees());
  variableValues[ConditionVar::FootSupportVal] = (!isMirror ? 1.f : -1.f) * engine.theFootSupport.support;
  variableValues[ConditionVar::BreakUp] = (!currentTorsoAngleBreakUp.pitchDirection.isInside(engine.theInertialData.angle.y()) || !currentTorsoAngleBreakUp.rollDirection.isInside(engine.theInertialData.angle.x())) ? 1.f : 0.f;
  variableValues[ConditionVar::LowCom] = (engine.theTorsoMatrix * engine.theRobotModel.centerOfMass).z();
  variableValues[ConditionVar::ComOutOfSupportPolygonX] = comDistanceToEdge.x();
  variableValues[ConditionVar::LyingOnArmsFront] = ((engine.theRobotModel.limbs[Limbs::wristLeft].translation.y() < 90.f && engine.theJointAngles.angles[Joints::lShoulderPitch] > 0_deg) // check both arms
                                                    || (engine.theRobotModel.limbs[Limbs::wristRight].translation.y() > -90.f && engine.theJointAngles.angles[Joints::rShoulderPitch] > 0_deg)) ? 1.f : 0.f;
  variableValues[ConditionVar::TryCounter] = static_cast<float>(tryCounter);

  // If a waitCondition is checked, this will help to abort the wait time IF it is impossible to fulfill the condition.
  // If a normal condition is checked, this code just wastes processor time and does stuff that has no consequences.
  if(!currentKeyframe.forbidWaitBreak)
  {
    bool increaseFailedWaitCounter = false;
    for(std::size_t i = 0; i < conditionSize; i++)
    {
      // TODO check if simplified version is the same (but with less bug)
      const float diff = std::abs(variableValues[conditions[i].variable] - conditions[i].range.limit(variableValues[conditions[i].variable]));
      if(!increaseFailedWaitCounter && diff < variableValuesCompare[conditions[i].variable])
      {
        failedWaitCounter += 100.f * Constants::motionCycleTime; // currently +1.2 for NAO V6
        increaseFailedWaitCounter = true;
      }
      variableValuesCompare[conditions[i].variable] = diff;
    }
  }

  auto mirrorCondition = [](const ConditionVar& var, const bool isMirror)
  {
    if(!isMirror)
      return var;
    switch(var)
    {
      case ConditionVar::ShoulderRollLeft:
        return ConditionVar::ShoulderRollRight;
      case ConditionVar::ShoulderRollRight:
        return ConditionVar::ShoulderRollLeft;
      case ConditionVar::ShoulderPitchLeft:
        return ConditionVar::ShoulderPitchRight;
      case ConditionVar::ShoulderPitchRight:
        return ConditionVar::ShoulderPitchLeft;
      case ConditionVar::WristTranslationYLeft:
        return ConditionVar::WristTranslationYRight;
      case ConditionVar::WristTranslationYRight:
        return ConditionVar::WristTranslationYLeft;
      default:
        return var;
    }
  };

  auto mirrorRange = [](const ConditionVar& var, Rangef range, const bool isMirror)
  {
    if(!isMirror)
      return range;
    switch(var)
    {
      case ConditionVar::ShoulderRollLeft:
      case ConditionVar::ShoulderRollRight:
      case ConditionVar::WristTranslationYLeft:
      case ConditionVar::WristTranslationYRight:
        return range.mirror();
      default:
        return range;
    }
  };

  for(std::size_t i = 0; i < conditionSize; ++i)
  {
    ConditionVar var = mirrorCondition(conditions[i].variable, isMirror);

    if(conditions[i].variable == ConditionVar::FluctuationY)
      failedWaitCounter = std::max(failedWaitCounter - 0.7f, 0.f);

    bool check = mirrorRange(var, conditions[i].range, isMirror).isInside(variableValues[var]);
    if(conditions[i].isNot)
      check = !check;
    if(!check && useAnd)
      return false;
    else if(check && !useAnd)
      return true;
  }

  return useAnd;
}

template bool KeyframePhaseBase::checkConditions(const std::vector<Condition>& conditions, bool useAnd);
template bool KeyframePhaseBase::checkConditions(const std::vector<WaitCondition>& conditions, bool useAnd);

bool KeyframePhaseBase::checkOptionalLine()
{
  for(const auto& branch : currentKeyframe.keyframeBranches)
  {
    if(!branch.preCondition.conditions.empty() && checkConditions(branch.preCondition.conditions, branch.preCondition.isAnd))
    {
      // make method ouf of it
      if(branch.motionID && (branch.maxNumberOfLoopsMotionID < 0 || branch.maxNumberOfLoopsMotionID > motionListIDLoops[branch.motionID.value()]))
      {
        currentMotion = engine.motions[branch.motionID.value()];
        motionListIDLoops[branch.motionID.value()]++;
        currentMotionBlock.clear();
      }
      else if(!branch.motionID.has_value() && (branch.maxNumberOfLoopsBlockID < 0 || branch.maxNumberOfLoopsBlockID > blockIDLoops[branch.blockID]))
      {
        bool setBaseStiffness = false;
        std::vector<KeyframeBlockBranched> oldKeyframes;
        if(branch.initStiffness && !currentMotionBlock.empty())
        {
          currentMotionBlock[0].stiffness = jointRequestOutput.stiffnessData.stiffnesses; // TODO use information!
          setBaseStiffness = true;
        }
        if(!branch.removeCurrentBlock)
          oldKeyframes = currentMotionBlock;
        currentMotionBlock.clear();
        currentMotionBlock.push_back(engine.keyframeBlock[branch.blockID]);

        blockIDLoops[branch.blockID]++;

        if(!oldKeyframes.empty())
          currentMotionBlock.insert(currentMotionBlock.end(), oldKeyframes.begin(), oldKeyframes.end());
        if(setBaseStiffness)
          setJointStiffnessBase();
      }
      return true;
    }
  }
  return false;
}

bool KeyframePhaseBase::checkEarlyBranch()
{
  for(const auto& branch : currentKeyframe.keyframeBranches)
  {
    if(branch.useEarlyEntrance && checkConditions(branch.earlyEntranceCondition.conditions, branch.earlyEntranceCondition.isAnd))
    {
      bool setBaseStiffness = false;
      // make method ouf of it
      if(branch.motionID && (branch.maxNumberOfLoopsMotionID < 0 || branch.maxNumberOfLoopsMotionID > motionListIDLoops[branch.motionID.value()]))
      {
        currentMotion = engine.motions[branch.motionID.value()];
        motionListIDLoops[branch.motionID.value()]++;
        currentMotionBlock.clear();
        setBaseStiffness = true;
        return true;
      }
      else if(!branch.motionID.has_value() && (branch.maxNumberOfLoopsBlockID < 0 || branch.maxNumberOfLoopsBlockID > blockIDLoops[branch.blockID]))
      {
        std::vector<KeyframeBlockBranched> oldKeyframes;
        if(branch.initStiffness && !currentMotionBlock.empty())
        {
          currentMotionBlock[0].stiffness = jointRequestOutput.stiffnessData.stiffnesses; // TODO use information!
          setBaseStiffness = true;
        }
        if(!branch.removeCurrentBlock)
          oldKeyframes = currentMotionBlock;
        currentMotionBlock.clear();
        currentMotionBlock.push_back(engine.keyframeBlock[branch.blockID]);

        blockIDLoops[branch.blockID]++;

        if(!oldKeyframes.empty())
          currentMotionBlock.insert(currentMotionBlock.end(), oldKeyframes.begin(), oldKeyframes.end());
        if(setBaseStiffness)
          setJointStiffnessBase();
        return true;
      }
    }
  }
  return false;
}
