#include "ArmKeyFrameEngine.h"

MAKE_MODULE(ArmKeyFrameEngine, motionControl);

ArmKeyFrameEngine::ArmKeyFrameEngine() : ArmKeyFrameEngineBase()
{
  arms[Arms::left] = Arm(Arms::left, Joints::lShoulderPitch);
  arms[Arms::right] = Arm(Arms::right, Joints::rShoulderPitch);

  ASSERT(allMotions[ArmKeyFrameRequest::useDefault].states.size() == 1);
  defaultPos = allMotions[ArmKeyFrameRequest::useDefault].states[0];
}

void ArmKeyFrameEngine::update(ArmKeyFrameGenerator& armKeyFrameGenerator)
{
  armKeyFrameGenerator.calcJoints = [this](Arms::Arm arm, const ArmMotionRequest& armMotionRequest, JointRequest& jointRequest, ArmMotionInfo& armMotionInfo)
  {
    // Hack to allow the walking engine to take over
    // TODO reverse should calculate the interpolation speed.
    if(armMotionRequest.armKeyFrameRequest.arms[arm].motion == ArmKeyFrameRequest::reverse && armMotionInfo.armKeyFrameRequest.arms[arm].motion == ArmKeyFrameRequest::raiseArm)
    {
      armMotionInfo.isFree[arm] = true;
      return;
    }
    if(armMotionInfo.armMotion[arm] != ArmMotionInfo::keyFrame)
      arms[arm].wasActive = false;
    updateArm(arms[arm], armMotionRequest, jointRequest);
    armMotionInfo.armMotion[arm] = ArmMotionInfo::keyFrame;
    armMotionInfo.armKeyFrameRequest.arms[arm].motion = arms[arm].currentMotion.id;
    armMotionInfo.armKeyFrameRequest.arms[arm].fast = arms[arm].fast;
    armMotionInfo.isFree[arm] = arms[arm].isLastMotionFinished &&
                                (arms[arm].currentMotion.id == ArmKeyFrameRequest::useDefault || arms[arm].currentMotion.id == ArmKeyFrameRequest::reverse);
  };
}

void ArmKeyFrameEngine::updateArm(Arm& arm, const ArmMotionRequest& armMotionRequest, JointRequest& jointRequest)
{
  if(!arm.wasActive || (arm.isLastMotionFinished && arm.currentMotion.id != armMotionRequest.armKeyFrameRequest.arms[arm.id].motion))
  {
    const ArmKeyFrameMotion& nextMotion = armMotionRequest.armKeyFrameRequest.arms[arm.id].motion == ArmKeyFrameRequest::reverse ?
                                                          (arm.currentMotion.id != ArmKeyFrameRequest::reverse ? arm.currentMotion.reverse(defaultPos) : allMotions[ArmKeyFrameRequest::useDefault]) :
                                                          allMotions[armMotionRequest.armKeyFrameRequest.arms[arm.id].motion];

    arm.startMotion(nextMotion, armMotionRequest.armKeyFrameRequest.arms[arm.id].fast, theJointAngles);
  }
  arm.wasActive = true;
  // test whether a motion is active now
  if(!arm.isLastMotionFinished)
  {
    // a motion is active, so decide what to do
    if(arm.stateIndex == arm.currentMotion.states.size())
    {
      // stay in target position
      arm.isLastMotionFinished = true;
      updateOutput(arm, jointRequest, arm.currentMotion.getTargetState());
    }
    else
    {
      // target not yet reached, so interpolate between the states
      ArmKeyFrameMotion::ArmAngles nextState = arm.currentMotion.states[arm.stateIndex];
      if(!arm.fast)
      {
        ArmKeyFrameMotion::ArmAngles result;
        createOutput(arm, nextState, arm.interpolationTime, result);
        updateOutput(arm, jointRequest, result);

        if(arm.interpolationTime >= nextState.steps)
        {
          ++arm.stateIndex;
          arm.interpolationTime = 1;
          arm.interpolationStart = nextState;
        }
      }
      else
      {
        // no interpolation
        updateOutput(arm, jointRequest, nextState);
        ++arm.stateIndex;
        arm.interpolationTime = 1;
        arm.interpolationStart = nextState;
      }
    }
  }
  else
    updateOutput(arm, jointRequest, arm.currentMotion.getTargetState());
}

void ArmKeyFrameEngine::createOutput(Arm& arm, ArmKeyFrameMotion::ArmAngles target, float& time, ArmKeyFrameMotion::ArmAngles& result)
{
  // interpolate angles and set stiffness
  const ArmKeyFrameMotion::ArmAngles from = arm.interpolationStart;
  for(unsigned i = 0; i < from.angles.size(); ++i)
  {
    const float offset = target.angles[i] - from.angles[i];
    const float speed = static_cast<float>(time) / static_cast<float>(target.steps);
    result.angles[i] = from.angles[i] + offset * speed;
    result.stiffness[i] = target.stiffness[i] == StiffnessData::useDefault
                          ? theStiffnessSettings.stiffnesses[arm.firstJoint + i]
                          : target.stiffness[i];
  }
  time += Constants::motionCycleTime * 1000.f;
}

void ArmKeyFrameEngine::updateOutput(const Arm& arm, JointRequest& jointRequest, const ArmKeyFrameMotion::ArmAngles& values)
{
  const Joints::Joint startJoint = arm.firstJoint;
  for(unsigned i = 0; i < values.angles.size(); ++i)
  {
    if(arm.id == Arms::right &&
       (i == Joints::shoulderRoll ||
        i == Joints::elbowYaw ||
        i == Joints::elbowRoll ||
        i == Joints::wristYaw))
      jointRequest.angles[startJoint + i] = -values.angles[i];
    else
      jointRequest.angles[startJoint + i] = values.angles[i];
    jointRequest.stiffnessData.stiffnesses[startJoint + i] = values.stiffness[i];
  }
}
