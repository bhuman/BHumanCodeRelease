#include "ArmKeyFrameEngine.h"
#include "Debugging/Plot.h"

MAKE_MODULE(ArmKeyFrameEngine);

ArmKeyFrameEngine::ArmKeyFrameEngine() :
  arms{Arm(Arms::left), Arm(Arms::right)}
{
  ASSERT(allMotions[ArmKeyFrameRequest::useDefault].states.size() == 1);
  defaultPos = allMotions[ArmKeyFrameRequest::useDefault].states[0];
}

void ArmKeyFrameEngine::update(ArmKeyFrameGenerator& armKeyFrameGenerator)
{
  DECLARE_PLOT("module:ArmKeyFrameEngine:left:interpolation");
  DECLARE_PLOT("module:ArmKeyFrameEngine:right:interpolation");

  armKeyFrameGenerator.calcJoints = [this](Arms::Arm arm, const ArmMotionRequest& armMotionRequest, JointRequest& jointRequest, ArmMotionInfo& armMotionInfo)
  {
    if(armMotionInfo.armMotion[arm] != ArmMotionInfo::keyFrame)
      arms[arm].wasActive = false;
    updateArm(arms[arm], armMotionRequest, jointRequest);
    armMotionInfo.armMotion[arm] = ArmMotionInfo::keyFrame;
    armMotionInfo.armKeyFrameRequest.arms[arm].motion = arms[arm].currentMotion.id;
    armMotionInfo.armKeyFrameRequest.arms[arm].fast = arms[arm].fast;
    armMotionInfo.isFree[arm] = arms[arm].isLastMotionFinished &&
                                (arms[arm].currentMotion.id == ArmKeyFrameRequest::useDefault || arms[arm].currentMotion.id == ArmKeyFrameRequest::reverse);
    armMotionInfo.isFinished[arm] = arms[arm].isLastMotionFinished;
  };
}

void ArmKeyFrameEngine::updateArm(Arm& arm, const ArmMotionRequest& armMotionRequest, JointRequest& jointRequest)
{
  if(!arm.wasActive || (arm.isLastMotionFinished && arm.currentMotion.id != armMotionRequest.armKeyFrameRequest.arms[arm.id].motion))
  {
    const ArmKeyFrameMotion& nextMotion = armMotionRequest.armKeyFrameRequest.arms[arm.id].motion == ArmKeyFrameRequest::reverse ?
                                          (arm.currentMotion.id != ArmKeyFrameRequest::reverse ?
                                           arm.currentMotion.reverse(defaultPos) :
                                           allMotions[ArmKeyFrameRequest::useDefault]) :
                                          allMotions[armMotionRequest.armKeyFrameRequest.arms[arm.id].motion];

    arm.startMotion(nextMotion, armMotionRequest.armKeyFrameRequest.arms[arm.id].fast, theJointAngles);
  }

  if(!arm.isLastMotionFinished && arm.stateIndex == arm.currentMotion.states.size())
    arm.isLastMotionFinished = true;

  if(!arm.isLastMotionFinished)
  {
    // target not yet reached, so interpolate between the states
    const ArmKeyFrameMotion::ArmKeyFrameState& nextState = arm.currentMotion.states[arm.stateIndex];
    if(!arm.fast)
    {
      ArmKeyFrameMotion::ArmAngles result;
      createOutput(arm, nextState, result);
      updateOutput(arm, jointRequest, result);

      if(arm.time >= nextState.duration)
      {
        ++arm.stateIndex;
        arm.time -= nextState.duration;
        arm.interpolationStart = nextState;
      }
    }
    else
    {
      // no interpolation
      updateOutput(arm, jointRequest, nextState);
      ++arm.stateIndex;
      arm.time = 0.f;
      arm.interpolationStart = nextState;
    }
  }
  else
    updateOutput(arm, jointRequest, arm.currentMotion.getTargetState());
}

void ArmKeyFrameEngine::createOutput(Arm& arm, const ArmKeyFrameMotion::ArmKeyFrameState& target, ArmKeyFrameMotion::ArmAngles& result) const
{
  // interpolate angles and set stiffness
  const ArmKeyFrameMotion::ArmAngles from = arm.interpolationStart;

  // calculate interpolation factor
  float phase = Rangef::ZeroOneRange().limit(arm.time / target.duration);
  switch(target.interpolation)
  {
    case ArmKeyFrameMotion::maxToZero:
    {
      phase = std::sin(Constants::pi_2 * phase);
      break;
    }
    case ArmKeyFrameMotion::zeroToMax:
    {
      phase = std::sin(-Constants::pi_2 + Constants::pi_2 * phase) + 1.f;
      break;
    }
    case ArmKeyFrameMotion::zeroToMaxToZero:
    {
      phase = (std::sin(-Constants::pi_2 + Constants::pi * phase) + 1.f) * 0.5f;
      break;
    }
    default:
      break;
  }
  if(arm.id == Arms::left)
    PLOT("module:ArmKeyFrameEngine:left:interpolation", phase);
  else
    PLOT("module:ArmKeyFrameEngine:right:interpolation", phase);

  // apply interpolation
  for(unsigned i = 0; i < from.angles.size(); ++i)
  {
    const float offset = target.angles[i] - from.angles[i];

    result.angles[i] = from.angles[i] + offset * phase;
    result.stiffness[i] = target.stiffness[i] == StiffnessData::useDefault
                          ? theStiffnessSettings.stiffnesses[arm.firstJoint + i]
                          : target.stiffness[i];
  }

  arm.time += Constants::motionCycleTime * 1000.f;
}

void ArmKeyFrameEngine::updateOutput(const Arm& arm, JointRequest& jointRequest, const ArmKeyFrameMotion::ArmAngles& values) const
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
