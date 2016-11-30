/**
 * @file Modules/MotionControl/MotionCombinator.cpp
 * This file implements a module that combines the motions created by the different modules.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "MotionCombinator.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/RotationMatrix.h"

MAKE_MODULE(MotionCombinator, motionControl)

void MotionCombinator::update(JointRequest& jointRequest)
{
  specialActionOdometry += theSpecialActionsOutput.odometryOffset;

  copy(theLegJointRequest, jointRequest, theStiffnessSettings, Joints::headYaw, Joints::headPitch);

  copy(theArmJointRequest, jointRequest, theStiffnessSettings, Joints::firstArmJoint, Joints::rHand);
  copy(theLegJointRequest, jointRequest, theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);

  ASSERT(jointRequest.isValid());

  // Find fully active motion and set MotionInfo
  if(theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] == 1.f)
  {
    // default values
    motionInfo.motion = theLegMotionSelection.targetMotion;
    motionInfo.isMotionStable = true;
    motionInfo.upcomingOdometryOffset = Pose2f();

    Pose2f odometryOffset;
    switch(theLegMotionSelection.targetMotion)
    {
      case MotionRequest::walk:
        odometryOffset = theWalkingEngineOutput.odometryOffset;
        motionInfo.walkRequest = theWalkingEngineOutput.executedWalk;
        motionInfo.upcomingOdometryOffset = theWalkingEngineOutput.upcomingOdometryOffset;
        break;
      case MotionRequest::kick:
        odometryOffset = theKickEngineOutput.odometryOffset;
        motionInfo.kickRequest = theKickEngineOutput.executedKickRequest;
        motionInfo.isMotionStable = theKickEngineOutput.isStable;
        break;
      case MotionRequest::specialAction:
        odometryOffset = specialActionOdometry;
        specialActionOdometry = Pose2f();
        motionInfo.specialActionRequest = theSpecialActionsOutput.executedSpecialAction;
        motionInfo.isMotionStable = theSpecialActionsOutput.isMotionStable;
        break;
      case MotionRequest::getUp:
        motionInfo.isMotionStable = false;
        odometryOffset = theGetUpEngineOutput.odometryOffset;
        break;
      case MotionRequest::stand:
      default:
        break;
    }

    if(theRobotInfo.hasFeature(RobotInfo::zGyro) && (theFallDownState.state == FallDownState::upright || theLegMotionSelection.targetMotion == MotionRequest::getUp))
    {
      Vector3f rotatedGyros = theInertialData.orientation * theInertialData.gyro.cast<float>();
      odometryOffset.rotation = rotatedGyros.z() * theFrameInfo.cycleTime;
    }

    odometryData += odometryOffset;
  }

  if(emergencyOffEnabled && motionInfo.motion != MotionRequest::getUp)
  {
    if(theFallDownState.state == FallDownState::falling && motionInfo.motion != MotionRequest::specialAction)

    {
      safeFall(jointRequest);
      centerHead(jointRequest);
      currentRecoveryTime = 0;

      ASSERT(jointRequest.isValid());
    }
    else if(theFallDownState.state == FallDownState::onGround && motionInfo.motion != MotionRequest::specialAction)
    {
      centerHead(jointRequest);
    }
    else
    {
      if(theFallDownState.state == FallDownState::upright)
      {
        headYawInSafePosition = false;
        headPitchInSafePosition = false;
      }

      if(currentRecoveryTime < recoveryTime)
      {
        currentRecoveryTime += 1;
        float ratio = (1.f / float(recoveryTime)) * currentRecoveryTime;
        for(int i = 0; i < Joints::numOfJoints; i++)
        {
          jointRequest.stiffnessData.stiffnesses[i] = 30 + int(ratio * float(jointRequest.stiffnessData.stiffnesses[i] - 30));
        }
      }
    }
  }

  if(debugArms)
    debugReleaseArms(jointRequest);
  eraseStiffness(jointRequest);

  float sum = 0;
  int count = 0;
  for(int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if(jointRequest.angles[i] != JointAngles::off && jointRequest.angles[i] != JointAngles::ignore &&
       lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += std::abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:legsOnly", sum / count);
  for(int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if(jointRequest.angles[i] != JointAngles::off && jointRequest.angles[i] != JointAngles::ignore &&
       lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += std::abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:all", sum / count);

  sum = 0;
  count = 0;
  for(int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
  {
    if(lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += std::abs(lastJointRequest.angles[i] - theJointAngles.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:differenceToJointData:legsOnly", sum / count);

  for(int i = 0; i < Joints::lHipYawPitch; i++)
  {
    if(lastJointRequest.angles[i] != JointAngles::off && lastJointRequest.angles[i] != JointAngles::ignore)
    {
      sum += std::abs(lastJointRequest.angles[i] - theJointAngles.angles[i]);
      count++;
    }
  }
  lastJointRequest = jointRequest;
  PLOT("module:MotionCombinator:differenceToJointData:all", sum / count);

#ifndef NDEBUG
  if(!jointRequest.isValid())
  {
    {
      std::string logDir = "";
#ifdef TARGET_ROBOT
      logDir = "../logs/";
#endif
      OutMapFile stream(logDir + "jointRequest.log");
      stream << jointRequest;
      OutMapFile stream2(logDir + "motionSelection.log");
      stream2 << theLegMotionSelection;
    }
    ASSERT(false);
  }
#endif
}

void MotionCombinator::update(OdometryData& odometryData)
{
  if(!theRobotInfo.hasFeature(RobotInfo::zGyro) || (theFallDownState.state != FallDownState::upright && theLegMotionSelection.targetMotion != MotionRequest::getUp))
    this->odometryData.rotation += theFallDownState.odometryRotationOffset;
  this->odometryData.rotation.normalize();

  odometryData = this->odometryData;

  Pose2f odometryOffset = odometryData;
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionCombinator:odometryOffsetX", odometryOffset.translation.x());
  PLOT("module:MotionCombinator:odometryOffsetY", odometryOffset.translation.y());
  PLOT("module:MotionCombinator:odometryOffsetRotation", odometryOffset.rotation.toDegrees());
  lastOdometryData = odometryData;
}

void MotionCombinator::safeFall(JointRequest& jointRequest)
{
  for(int i = 0; i < Joints::numOfJoints; i++)
    jointRequest.stiffnessData.stiffnesses[i] = 30;
}

void MotionCombinator::centerHead(JointRequest& jointRequest)
{
  jointRequest.angles[Joints::headYaw] = 0;
  jointRequest.angles[Joints::headPitch] = 0;
  if(theFallDownState.direction == FallDownState::front)
    jointRequest.angles[Joints::headPitch] = -0.4f;
  else if(theFallDownState.direction == FallDownState::back)
    jointRequest.angles[Joints::headPitch] = 0.3f;
  if(std::abs(theJointAngles.angles[Joints::headYaw]) > 0.1f && !headYawInSafePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 100;
  else
  {
    headYawInSafePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headYaw] = 25;
  }
  if(std::abs(theJointAngles.angles[Joints::headPitch] - jointRequest.angles[Joints::headPitch]) > 0.1f && !headPitchInSafePosition)
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 100;
  else
  {
    headPitchInSafePosition = true;
    jointRequest.stiffnessData.stiffnesses[Joints::headPitch] = 25;
  }
}

void MotionCombinator::copy(const JointRequest& source, JointRequest& target,
                            const StiffnessSettings& theStiffnessSettings,
                            const Joints::Joint startJoint, const Joints::Joint endJoint)
{
  for(int i = startJoint; i <= endJoint; ++i)
  {
    if(source.angles[i] != JointAngles::ignore)
      target.angles[i] = source.angles[i];
    target.stiffnessData.stiffnesses[i] = target.angles[i] != JointAngles::off ? source.stiffnessData.stiffnesses[i] : 0;
    if(target.stiffnessData.stiffnesses[i] == StiffnessData::useDefault)
      target.stiffnessData.stiffnesses[i] = theStiffnessSettings.stiffnesses[i];
  }
}

void MotionCombinator::interpolate(const JointRequest& from, const JointRequest& to,
                                   float fromRatio, JointRequest& target, bool interpolateStiffness,
                                   const StiffnessSettings& theStiffnessSettings, const JointAngles& lastJointAngles,
                                   const Joints::Joint startJoint, const Joints::Joint endJoint)
{
  for(int i = startJoint; i <= endJoint; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == JointAngles::ignore && f == JointAngles::ignore)
      continue;

    if(t == JointAngles::ignore)
      t = target.angles[i];
    if(f == JointAngles::ignore)
      f = target.angles[i];

    int fStiffness = f != JointAngles::off ? from.stiffnessData.stiffnesses[i] : 0;
    int tStiffness = t != JointAngles::off ? to.stiffnessData.stiffnesses[i] : 0;
    if(fStiffness == StiffnessData::useDefault)
      fStiffness = theStiffnessSettings.stiffnesses[i];
    if(tStiffness == StiffnessData::useDefault)
      tStiffness = theStiffnessSettings.stiffnesses[i];

    if(t == JointAngles::off || t == JointAngles::ignore)
      t = lastJointAngles.angles[i];
    if(f == JointAngles::off || f == JointAngles::ignore)
      f = lastJointAngles.angles[i];
    if(target.angles[i] == JointAngles::off || target.angles[i] == JointAngles::ignore)
      target.angles[i] = lastJointAngles.angles[i];

    ASSERT(target.angles[i] != JointAngles::off && target.angles[i] != JointAngles::ignore);
    ASSERT(t != JointAngles::off && t != JointAngles::ignore);
    ASSERT(f != JointAngles::off && f != JointAngles::ignore);

    target.angles[i] += -fromRatio * t + fromRatio * f;
    if(interpolateStiffness)
      target.stiffnessData.stiffnesses[i] += int(-fromRatio * float(tStiffness) + fromRatio * float(fStiffness));
    else
      target.stiffnessData.stiffnesses[i] = tStiffness;
  }
}

void MotionCombinator::debugReleaseArms(JointRequest& jointRequest)
{
  /*if(theKeyStates.pressed[theKeyStates.lHandBack] ||
    theKeyStates.pressed[theKeyStates.lHandLeft] ||
    theKeyStates.pressed[theKeyStates.lHandRight])
    for(unsigned i = Joints::firstLeftArmJoint; i <= Joints::lHand; i++)
      jointRequest.stiffnessData.stiffnesses[i] = 0;

  if(theKeyStates.pressed[theKeyStates.rHandBack] ||
    theKeyStates.pressed[theKeyStates.rHandLeft] ||
    theKeyStates.pressed[theKeyStates.rHandRight])
    for(unsigned i = Joints::firstRightArmJoint; i <= Joints::rHand; i++)
      jointRequest.stiffnessData.stiffnesses[i] = 0;*/

  if(theKeyStates.pressed[theKeyStates.headFront]
     || theKeyStates.pressed[theKeyStates.headMiddle]
     || theKeyStates.pressed[theKeyStates.headRear])
    for(unsigned i = Joints::firstLeftArmJoint; i <= Joints::rHand; i++)
      jointRequest.stiffnessData.stiffnesses[i] = 0;
}

void MotionCombinator::eraseStiffness(JointRequest& jointRequest)
{
  for(Joints::Joint j : theDamageConfigurationBody.jointsToEraseStiffness)
    jointRequest.stiffnessData.stiffnesses[j] = 0;
}
