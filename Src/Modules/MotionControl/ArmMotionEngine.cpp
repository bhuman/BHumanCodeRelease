#include "ArmMotionEngine.h"

MAKE_MODULE(ArmMotionEngine, Motion Control)

ArmMotionEngine::ArmMotionEngine() : ArmMotionEngineBase()
{
  arms[ArmMotionRequest::left] = Arm(ArmMotionRequest::left, JointData::LShoulderPitch);
  arms[ArmMotionRequest::right] = Arm(ArmMotionRequest::right, JointData::RShoulderPitch);

  ASSERT(allMotions[ArmMotionRequest::useDefault].states.size() == 1);
  defaultPos = allMotions[ArmMotionRequest::useDefault].states[0];
}

void ArmMotionEngine::update(ArmMotionEngineOutput& armMotionEngineOutput)
{
  updateArm(arms[ArmMotionRequest::left], armMotionEngineOutput);
  updateArm(arms[ArmMotionRequest::right], armMotionEngineOutput);
}

bool ArmMotionEngine::newMotionPossible(Arm& arm)
{
  return !arm.isMotionActive &&
    (theGameInfo.state == STATE_PLAYING || theGameInfo.state == STATE_READY) &&
    (theMotionInfo.motion == MotionInfo::walk || theMotionInfo.motion == MotionInfo::stand) &&
    theGroundContactState.contact;
}

void ArmMotionEngine::updateArm(Arm& arm, ArmMotionEngineOutput& armMotionEngineOutput)
{
  // assume arm should not be moved.
  armMotionEngineOutput.arms[arm.id].move = false;

  // make sure to not interfere with other motion engines
  if(theFallDownState.state == FallDownState::onGround ||
    theMotionInfo.motion == MotionInfo::getUp)
  {
    arm.isMotionActive = false;
    return;
  }

  if(newMotionPossible(arm))
  {
    // no motion is active, so decide what to do.
    // behavior overrides contact triggered motions
    if(theArmMotionRequest.motion[arm.id] != ArmMotionRequest::useDefault)
    {
      ArmMotion nextMotion = allMotions[theArmMotionRequest.motion[arm.id]];
      arm.contactTriggered = false;
      arm.startMotion(nextMotion,
        theArmMotionRequest.fast[arm.id],
        theArmMotionRequest.autoReverse[arm.id],
        theArmMotionRequest.autoReverseTime[arm.id],
        theFilteredJointData);

    } else if(theFrameInfo.getTimeSince(arm.lastContactAction) > actionDelay &&
      (arm.id == ArmMotionRequest::left ? theArmContactModel.contactLeft : theArmContactModel.contactRight))
    {
#ifdef TARGET_ROBOT
      // check for armcontact
      ArmContactModel::PushDirection dir = (arm.id == ArmMotionRequest::left)
        ? theArmContactModel.pushDirectionLeft
        : theArmContactModel.pushDirectionRight;

      switch(dir)
      {
      case ArmContactModel::S:
      case ArmContactModel::SW:
      case ArmContactModel::SE:
        arm.lastContactAction = theFrameInfo.time;
        arm.contactTriggered = true;
        arm.startMotion(allMotions[ArmMotionRequest::back], false, true, targetTime, theFilteredJointData);
        break;
      default:
        break;
      }
#endif
    }
  }

  // when falling, try to set emergency mode fast
  if(theFallDownState.state == FallDownState::falling && arm.isMotionActive && arm.currentMotion.id != ArmMotionRequest::falling)
  {
    arm.startMotion(allMotions[ArmMotionRequest::falling], true, false, 0, theFilteredJointData);
  }
  else if((theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::onGround || !theGroundContactState.contact) &&
    !arm.isMotionActive)
  {
    // fallen
    return;
  }
  else if(theMotionInfo.motion == MotionInfo::bike && arm.isMotionActive && arm.currentMotion.id != ArmMotionRequest::useDefault)
  {
    arm.startMotion(allMotions[ArmMotionRequest::useDefault], true, false, 0, theFilteredJointData);
  }

  // test whether a motion is active now
  if(arm.isMotionActive)
  {
    // a motion is active, so decide what to do

    if(arm.stateIndex == arm.currentMotion.states.size())
    {
      // arm reached its motion target
      ++arm.targetTime;

      if(arm.currentMotion.id == ArmMotionRequest::useDefault)
      {
        // arm is in default position, so ARME won't output any angles
        armMotionEngineOutput.arms[arm.id].move = false;
        arm.isMotionActive = false;
      }
      else if((arm.autoReverse && arm.targetTime == arm.autoReverseTime) || // target time ran out
        (!theGroundContactState.contact) ||  // ground contact lost
        (theArmMotionRequest.motion[arm.id] != arm.currentMotion.id && !arm.contactTriggered))  // different motion requested
      {
        // start a reversed motion to reach the default position
        arm.startMotion(arm.currentMotion.reverse(defaultPos),
          theArmMotionRequest.fast[arm.id], false, 0, theFilteredJointData);
      }
      else
      {
        // stay in target position
        updateOutput(arm, armMotionEngineOutput, arm.currentMotion.getTargetState());
      }
    }
    else
    {
      // target not yet reached, so interpolate between the states
      ArmMotion::ArmAngles nextState = arm.currentMotion.states[arm.stateIndex];
      if(!arm.fast)
      {
        ArmMotion::ArmAngles result;
        createOutput(arm, nextState, arm.interpolationTime, result);
        updateOutput(arm, armMotionEngineOutput, result);

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
        updateOutput(arm, armMotionEngineOutput, nextState);
        ++arm.stateIndex;
        arm.interpolationTime = 1;
        arm.interpolationStart = nextState;
      }
    }
  }
}

void ArmMotionEngine::createOutput(Arm& arm, ArmMotion::ArmAngles target, int& time, ArmMotion::ArmAngles& result)
{
  // interpolate angles and set hardness
  const ArmMotion::ArmAngles from = arm.interpolationStart;
  for(unsigned i = 0; i < from.angles.size(); ++i)
  {
      const float offset = target.angles[i] - from.angles[i];
      const float speed = (float) time / (float) target.steps;
      result.angles[i] = (from.angles[i] + offset * speed);
      result.hardness[i] = target.hardness[i] == HardnessData::useDefault
        ? theHardnessSettings.hardness[arm.firstJoint + i]
        : target.hardness[i];
  }
  ++time;
}

void ArmMotionEngine::updateOutput(Arm& arm, ArmMotionEngineOutput& armMotionEngineOutput, ArmMotion::ArmAngles& values)
{
  for(unsigned i = 0; i < values.angles.size(); ++i)
  {
    armMotionEngineOutput.arms[arm.id].angles[i] = values.angles[i];
    armMotionEngineOutput.arms[arm.id].hardness[i] = values.hardness[i];
  }
  armMotionEngineOutput.arms[arm.id].motion = arm.currentMotion.id;
  armMotionEngineOutput.arms[arm.id].move = true;
  armMotionEngineOutput.arms[arm.id].lastMovement = theFrameInfo.time;
}