
#include "ArmKeyFrameEngine.h"

MAKE_MODULE(ArmKeyFrameEngine, motionControl)

ArmKeyFrameEngine::ArmKeyFrameEngine() : ArmKeyFrameEngineBase()
{
  arms[Arms::left] = Arm(Arms::left, Joints::lShoulderPitch);
  arms[Arms::right] = Arm(Arms::right, Joints::rShoulderPitch);

  ASSERT(allMotions[ArmKeyFrameRequest::useDefault].states.size() == 1);
  defaultPos = allMotions[ArmKeyFrameRequest::useDefault].states[0];
}

void ArmKeyFrameEngine::update(ArmKeyFrameEngineOutput& armKeyFrameEngineOutput)
{
  updateArm(arms[Arms::left], armKeyFrameEngineOutput);
  updateArm(arms[Arms::right], armKeyFrameEngineOutput);
}

void ArmKeyFrameEngine::updateArm(Arm& arm, ArmKeyFrameEngineOutput& armKeyFrameEngineOutput)
{
  if(theArmMotionSelection.targetArmMotion[arm.id] != ArmMotionSelection::keyFrameS)
  {
    arm.wasActive = false;
    return;
  }

  if(!arm.wasActive || (arm.isLastMotionFinished && arm.currentMotion.id != theArmKeyFrameRequest.arms[arm.id].motion))
  {
    const ArmKeyFrameMotion& nextMotion = theArmKeyFrameRequest.arms[arm.id].motion == ArmKeyFrameRequest::reverse ?
                                          (arm.currentMotion.id != ArmKeyFrameRequest::reverse ? arm.currentMotion.reverse(defaultPos) : allMotions[ArmKeyFrameRequest::useDefault]) :
                                          allMotions[theArmKeyFrameRequest.arms[arm.id].motion];

    arm.startMotion(nextMotion, theArmKeyFrameRequest.arms[arm.id].fast, theJointAngles);
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
      updateOutput(arm, armKeyFrameEngineOutput, arm.currentMotion.getTargetState());
    }
    else
    {
      // target not yet reached, so interpolate between the states
      ArmKeyFrameMotion::ArmAngles nextState = arm.currentMotion.states[arm.stateIndex];
      if(!arm.fast)
      {
        ArmKeyFrameMotion::ArmAngles result;
        createOutput(arm, nextState, arm.interpolationTime, result);
        updateOutput(arm, armKeyFrameEngineOutput, result);

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
        updateOutput(arm, armKeyFrameEngineOutput, nextState);
        ++arm.stateIndex;
        arm.interpolationTime = 1;
        arm.interpolationStart = nextState;
      }
    }
  }
}

void ArmKeyFrameEngine::createOutput(Arm& arm, ArmKeyFrameMotion::ArmAngles target, int& time, ArmKeyFrameMotion::ArmAngles& result)
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
  ++time;
}

void ArmKeyFrameEngine::updateOutput(Arm& arm, ArmKeyFrameEngineOutput& armKeyFrameEngineOutput, ArmKeyFrameMotion::ArmAngles& values)
{
  const Joints::Joint startJoint = !arm.id ? Joints::lShoulderPitch : Joints::rShoulderPitch;
  for(unsigned i = 0; i < values.angles.size(); ++i)
  {
    if(arm.id == Arms::right &&
       (i == Joints::shoulderRoll ||
        i == Joints::elbowYaw ||
        i == Joints::elbowRoll ||
        i == Joints::wristYaw))
      armKeyFrameEngineOutput.angles[startJoint + i] = -values.angles[i];
    else
      armKeyFrameEngineOutput.angles[startJoint + i] = values.angles[i];
    armKeyFrameEngineOutput.stiffnessData.stiffnesses[startJoint + i] = values.stiffness[i];
  }
  armKeyFrameEngineOutput.arms[arm.id].motion = arm.currentMotion.id;
  armKeyFrameEngineOutput.arms[arm.id].isFree = arm.isLastMotionFinished &&
                                                (arm.currentMotion.id == ArmKeyFrameRequest::useDefault || arm.currentMotion.id == ArmKeyFrameRequest::reverse);
}
