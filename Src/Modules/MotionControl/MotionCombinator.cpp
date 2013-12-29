/**
* @file Modules/MotionControl/MotionCombinator.cpp
* This file implements a module that combines the motions created by the different modules.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#include "MotionCombinator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Infrastructure/JointDataDeg.h"

using std::abs;

MotionCombinator::MotionCombinator()
{
  currentRecoveryTime = recoveryTime + 1;
  headJawInSavePosition = false;
  headPitchInSavePosition = false;
}

void MotionCombinator::update(JointRequest& jointRequest)
{
  specialActionOdometry += theSpecialActionsOutput.odometryOffset;

  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
  jointRequests[MotionRequest::bike] = &theBikeEngineOutput;
  jointRequests[MotionRequest::indykick] = &theIndykickEngineOutput;
  jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
  jointRequests[MotionRequest::stand] = &theWalkingEngineStandOutput;
  jointRequests[MotionRequest::getUp] = &theGetUpEngineOutput;
  jointRequests[MotionRequest::takeBall] = &theBallTakingOutput;

  jointRequest.angles[JointData::HeadYaw] = theHeadJointRequest.pan;
  jointRequest.angles[JointData::HeadPitch] = theHeadJointRequest.tilt;

  copy(*jointRequests[theMotionSelection.targetMotion], jointRequest);

  int i;
  for(i = 0; i < MotionRequest::numOfMotions; ++i)
    if(theMotionSelection.ratios[i] == 1.f)
    {
      // default values
      motionInfo.motion = MotionRequest::Motion(i);
      motionInfo.isMotionStable = true;
      motionInfo.upcomingOdometryOffset = Pose2D();

      lastJointData = theFilteredJointData;

      if(theMotionSelection.ratios[MotionRequest::walk] == 1.f)
      {
        odometryData += theWalkingEngineOutput.odometryOffset;
        motionInfo.walkRequest = theWalkingEngineOutput.executedWalk;
        motionInfo.upcomingOdometryOffset = theWalkingEngineOutput.upcomingOdometryOffset;
      }
      else if(theMotionSelection.ratios[MotionRequest::bike] == 1.f)
      {
        odometryData += theBikeEngineOutput.odometryOffset;
        motionInfo.bikeRequest = theBikeEngineOutput.executedBikeRequest;
        motionInfo.isMotionStable = theBikeEngineOutput.isStable;
      }
      else if(theMotionSelection.ratios[MotionRequest::indykick] == 1.0f)
      {
        motionInfo.indykickRequest = theIndykickEngineOutput.executedIndykickRequest;
      }
      else if(theMotionSelection.ratios[MotionRequest::specialAction] == 1.0f)
      {
        odometryData += specialActionOdometry;
        specialActionOdometry = Pose2D();
        motionInfo.specialActionRequest = theSpecialActionsOutput.executedSpecialAction;
        motionInfo.isMotionStable = theSpecialActionsOutput.isMotionStable;
      }
      else if(theMotionSelection.ratios[MotionRequest::stand] == 1.f)
      {
        motionInfo.motion = MotionRequest::stand;
      }
      else if(theMotionSelection.ratios[MotionRequest::getUp] == 1.f)
      {
        motionInfo.motion = MotionRequest::getUp;
        motionInfo.isMotionStable = false;
        odometryData += theGetUpEngineOutput.odometryOffset;
      }
      else if(theMotionSelection.ratios[MotionRequest::takeBall] == 1.f)
      {
        motionInfo.motion = MotionRequest::takeBall;
        motionInfo.isMotionStable = true;
        odometryData += theBallTakingOutput.odometryOffset;
      }
      break;
    }

  if(i == MotionRequest::numOfMotions)
  {
    for(i = 0; i < MotionRequest::numOfMotions; ++i)
      if(i != theMotionSelection.targetMotion && theMotionSelection.ratios[i] > 0.)
      {
        bool interpolateHardness = !(i == MotionRequest::specialAction && theMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead); // do not interpolate from play_dead
        interpolate(*jointRequests[i], *jointRequests[theMotionSelection.targetMotion], theMotionSelection.ratios[i], jointRequest, interpolateHardness);
      }
  }

  if(emergencyOffEnabled)
  {
    if(theFallDownState.state == FallDownState::falling && motionInfo.motion != MotionRequest::specialAction)
    {
      saveFall(jointRequest);
      centerHead(jointRequest);
      currentRecoveryTime = 0;
    }
    else if((theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::onGround) && (motionInfo.motion != MotionRequest::specialAction))
    {
      centerHead(jointRequest);
    }
    else
    {
      if(theFallDownState.state == FallDownState::upright)
      {
        headJawInSavePosition = false;
        headPitchInSavePosition = false;
      }

      if(currentRecoveryTime < recoveryTime)
      {
        currentRecoveryTime += 1;
        float ratio = (1.f / float(recoveryTime)) * currentRecoveryTime;
        for(int i = 0; i < JointData::numOfJoints; i ++)
        {
          jointRequest.jointHardness.hardness[i] = 30 + int (ratio * float(jointRequest.jointHardness.hardness[i] - 30));
        }
      }
    }
  }

#ifndef RELEASE
  float sum(0);
  int count(0);
  for(int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++)
  {
    if(jointRequest.angles[i] != JointData::off && jointRequest.angles[i] != JointData::ignore && lastJointRequest.angles[i] != JointData::off && lastJointRequest.angles[i] != JointData::ignore)
    {
      sum += abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:legsOnly", sum / count);
  for(int i = 0; i < JointData::LHipYawPitch; i++)
  {
    if(jointRequest.angles[i] != JointData::off && jointRequest.angles[i] != JointData::ignore && lastJointRequest.angles[i] != JointData::off && lastJointRequest.angles[i] != JointData::ignore)
    {
      sum += abs(jointRequest.angles[i] - lastJointRequest.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:deviations:JointRequest:all", sum / count);

  sum = 0;
  count = 0;
  for(int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++)
  {
    if(lastJointRequest.angles[i] != JointData::off && lastJointRequest.angles[i] != JointData::ignore)
    {
      sum += abs(lastJointRequest.angles[i] - theFilteredJointData.angles[i]);
      count++;
    }
  }
  PLOT("module:MotionCombinator:differenceToJointData:legsOnly", sum / count);

  for(int i = 0; i < JointData::LHipYawPitch; i++)
  {
    if(lastJointRequest.angles[i] != JointData::off && lastJointRequest.angles[i] != JointData::ignore)
    {
      sum += abs(lastJointRequest.angles[i] - theFilteredJointData.angles[i]);
      count++;
    }
  }
  lastJointRequest = jointRequest;
  PLOT("module:MotionCombinator:differenceToJointData:all", sum / count);
#endif

#ifndef NDEBUG
  if(!jointRequest.isValid())
  {
    {
      OutMapFile stream("jointRequest.log");
      stream << jointRequest;
      OutMapFile stream2("motionSelection.log");
      stream2 << theMotionSelection;
    }
    ASSERT(false);
  }
#endif

#ifndef RELEASE
  JointDataDeg jointRequestDeg(jointRequest);
#endif
  MODIFY("representation:JointRequestDeg", jointRequestDeg);
}

void MotionCombinator::copy(const JointRequest& source, JointRequest& target) const
{
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    if(source.angles[i] != JointData::ignore)
      target.angles[i] = source.angles[i];
    target.jointHardness.hardness[i] = source.angles[i] != JointData::off ? source.jointHardness.hardness[i] : 0;
    if(target.jointHardness.hardness[i] == HardnessData::useDefault)
      target.jointHardness.hardness[i] = theHardnessSettings.hardness[i];
  }
}

void MotionCombinator::interpolate(const JointRequest& from, const JointRequest& to,
                                   float fromRatio, JointRequest& target, bool interpolateHardness) const
{
  for(int i = 0; i < JointData::numOfJoints; ++i)
  {
    float f = from.angles[i];
    float t = to.angles[i];

    if(t == JointData::ignore && f == JointData::ignore)
      continue;

    if(t == JointData::ignore)
      t = target.angles[i];
    if(f == JointData::ignore)
      f = target.angles[i];

    int fHardness = f != JointData::off ? from.jointHardness.hardness[i] : 0;
    int tHardness = t != JointData::off ? to.jointHardness.hardness[i] : 0;
    if(fHardness == HardnessData::useDefault)
      fHardness = theHardnessSettings.hardness[i];
    if(tHardness == HardnessData::useDefault)
      tHardness = theHardnessSettings.hardness[i];

    if(t == JointData::off || t == JointData::ignore)
      t = lastJointData.angles[i];
    if(f == JointData::off || f == JointData::ignore)
      f = lastJointData.angles[i];
    if(target.angles[i] == JointData::off || target.angles[i] == JointData::ignore)
      target.angles[i] = lastJointData.angles[i];

    ASSERT(target.angles[i] != JointData::off && target.angles[i] != JointData::ignore);
    ASSERT(t != JointData::off && t != JointData::ignore);
    ASSERT(f != JointData::off && f != JointData::ignore);

    target.angles[i] += -fromRatio * t + fromRatio * f;
    if(interpolateHardness)
      target.jointHardness.hardness[i] += int(-fromRatio * float(tHardness) + fromRatio * float(fHardness));
    else
      target.jointHardness.hardness[i] = tHardness;
  }
}

void MotionCombinator::update(OdometryData& odometryData)
{
  this->odometryData.rotation += theFallDownState.odometryRotationOffset;
  this->odometryData.rotation = normalize(this->odometryData.rotation);

  odometryData = this->odometryData;

#ifndef RELEASE
  Pose2D odometryOffset(odometryData);
  odometryOffset -= lastOdometryData;
  PLOT("module:MotionCombinator:odometryOffsetX", odometryOffset.translation.x);
  PLOT("module:MotionCombinator:odometryOffsetY", odometryOffset.translation.y);
  PLOT("module:MotionCombinator:odometryOffsetRotation", toDegrees(odometryOffset.rotation));
  lastOdometryData = odometryData;
#endif
}

void MotionCombinator::saveFall(JointRequest& jointRequest)
{
  for(int i = 0; i < JointData::numOfJoints; i++)
    jointRequest.jointHardness.hardness[i] = 30;

  // if ARME is moving the arms, do not lower hardness to ensure motion can be finished
  if (theArmMotionEngineOutput.arms[ArmMotionRequest::left].move)
  {
    for(int i = 0; i < 4; ++i)
      jointRequest.jointHardness.hardness[JointData::LShoulderPitch + i] = theArmMotionEngineOutput.arms[ArmMotionRequest::left].hardness[i];
  }
  if (theArmMotionEngineOutput.arms[ArmMotionRequest::right].move)
  {
    for(int i = 0; i < 4; ++i)
      jointRequest.jointHardness.hardness[JointData::RShoulderPitch + i] = theArmMotionEngineOutput.arms[ArmMotionRequest::right].hardness[i];
  }
}

void MotionCombinator::centerHead(JointRequest& jointRequest)
{
  jointRequest.angles[JointData::HeadYaw] = 0;
  jointRequest.angles[JointData::HeadPitch] = 0;
  if(theFallDownState.direction == FallDownState::front)
    jointRequest.angles[JointData::HeadPitch] = 0.4f;
  else if(theFallDownState.direction == FallDownState::back)
    jointRequest.angles[JointData::HeadPitch] = -0.3f;
  if(abs(theFilteredJointData.angles[JointData::HeadYaw]) > 0.1f && !headJawInSavePosition)
    jointRequest.jointHardness.hardness[JointData::HeadYaw] = 100;
  else
  {
    headJawInSavePosition = true;
    jointRequest.jointHardness.hardness[JointData::HeadYaw] = 25;
  }
  if(abs(theFilteredJointData.angles[JointData::HeadPitch] - jointRequest.angles[JointData::HeadPitch]) > 0.1f && !headPitchInSavePosition)
    jointRequest.jointHardness.hardness[JointData::HeadPitch] = 100;
  else
  {
    headPitchInSavePosition = true;
    jointRequest.jointHardness.hardness[JointData::HeadPitch] = 25;
  }
}

MAKE_MODULE(MotionCombinator, Motion Control)
