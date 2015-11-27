/**
 * @file Modules/MotionControl/MotionSelector.cpp
 * This file implements a module that is responsible for controlling the motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author Jesse Richter-Klug
 */

#include <algorithm>
#include "MotionSelector.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(MotionSelector, motionControl)

PROCESS_LOCAL MotionSelector* MotionSelector::theInstance = 0;

void MotionSelector::stand()
{
  if(theInstance && SystemCall::getMode() == SystemCall::physicalRobot)
    theInstance->forceStand = true;
}

void MotionSelector::update(MotionSelection& motionSelection)
{
  static int interpolationTimes[MotionRequest::numOfMotions];
  interpolationTimes[MotionRequest::walk] = 790;
  interpolationTimes[MotionRequest::kick] = 200;
  interpolationTimes[MotionRequest::dmpKick] = 200;
  interpolationTimes[MotionRequest::specialAction] = 200;
  interpolationTimes[MotionRequest::stand] = 600;
  interpolationTimes[MotionRequest::getUp] = 600;

  static int armInterPolationTimes[ArmMotionRequest::numOfArmMotions];
  armInterPolationTimes[ArmMotionRequest::keyFrame] = 300; // TODO think about times
  armInterPolationTimes[ArmMotionRequest::none] = 200; // TODO think about times

  static const int playDeadDelay(2000);

  if(lastExecution)
  {
    MotionRequest::Motion requestedMotion = theMotionRequest.motion;
    if(theMotionRequest.motion == MotionRequest::walk && !theGroundContactState.contact)
      requestedMotion = MotionRequest::stand;

    if(forceStand && (lastMotion == MotionRequest::walk || lastMotion == MotionRequest::stand))
    {
      requestedMotion = MotionRequest::stand;
      forceStand = false;
    }

    // check if the target motion can be the requested motion (mainly if leaving is possible)
    if((lastMotion == MotionRequest::walk && (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
       (lastMotion == MotionRequest::stand && motionSelection.ratios[MotionRequest::stand] == 1.f) || // stand can always be left if fully active
       (lastMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) ||
       (lastMotion == MotionRequest::kick && theKickEngineOutput.isLeavingPossible) ||
       (lastMotion == MotionRequest::dmpKick && theDmpKickEngineOutput.isLeavingPossible) ||
       (lastMotion == MotionRequest::getUp && theGetUpEngineOutput.isLeavingPossible)) //never immediatly leave kick or get up
    {
      motionSelection.targetMotion = requestedMotion;
    }

    if(requestedMotion == MotionRequest::specialAction)
    {
      motionSelection.specialActionRequest = theMotionRequest.specialActionRequest;
    }
    else
    {
      motionSelection.specialActionRequest = SpecialActionRequest();
      if(motionSelection.targetMotion == MotionRequest::specialAction)
        motionSelection.specialActionRequest.specialAction = SpecialActionRequest::numOfSpecialActionIDs;
    }

    const bool afterPlayDead(prevMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead);

    const int bodyInterpolationTime(afterPlayDead ? playDeadDelay : interpolationTimes[motionSelection.targetMotion]);
    interpolate(motionSelection.ratios, MotionRequest::numOfMotions, bodyInterpolationTime, motionSelection.targetMotion);

    if(motionSelection.ratios[MotionRequest::specialAction] < 1.f)
    {
      if(motionSelection.targetMotion == MotionRequest::specialAction)
        motionSelection.specialActionMode = MotionSelection::first;
      else
        motionSelection.specialActionMode = MotionSelection::deactive;
    }
    else
      motionSelection.specialActionMode = MotionSelection::active;

    if(motionSelection.specialActionMode == MotionSelection::active && motionSelection.specialActionRequest.specialAction != SpecialActionRequest::numOfSpecialActionIDs)
      lastActiveSpecialAction = motionSelection.specialActionRequest.specialAction;

    auto selectArmMotions = [&](Arms::Arm const arm)
    {
      ArmMotionRequest::ArmMotion requestedArmMotion = theArmMotionRequest.armMotion[arm];

      bool forceNone(false);
      switch(motionSelection.targetMotion)
      {
        case MotionRequest::kick:
        case MotionRequest::dmpKick:
        case MotionRequest::getUp:
          forceNone = true;
        case MotionRequest::specialAction:
          forceNone = !theSpecialActionsOutput.isArmLeavingAllowed;
        default:
          break;
      }

      if(forceNone)
        requestedArmMotion = ArmMotionRequest::none;

      // check if the target armmotion can be the requested armmotion (mainly if leaving is possible)
      /*if(lastArmMotion[arm] == ArmMotionRequest::keyFrame ||
         lastArmMotion[arm] == ArmMotionRequest::none ||
         forceNone)*/ //TODO - If u have a real condition ur free to use this
      {
        if(requestedArmMotion != ArmMotionRequest::keyFrame && !theArmKeyFrameEngineOutput.arms[arm].isFree && requestedMotion != MotionRequest::getUp &&
           !(requestedMotion == MotionRequest::specialAction && theArmMotionRequest.armKeyFrameRequest.arms[arm].motion == ArmKeyFrameRequest::keeperStand))
        {
          armMotionSelection.targetArmMotion[arm] = ArmMotionRequest::keyFrame;
          armMotionSelection.armKeyFrameRequest.arms[arm].fast = forceNone;
          armMotionSelection.armKeyFrameRequest.arms[arm].motion = ArmKeyFrameRequest::reverse;
        }
        else
        {
          armMotionSelection.targetArmMotion[arm] = requestedArmMotion;
          if(armMotionSelection.targetArmMotion[arm] == ArmMotionRequest::keyFrame)
            armMotionSelection.armKeyFrameRequest.arms[arm] = theArmMotionRequest.armKeyFrameRequest.arms[arm];
        }
      }

      const bool afterPlayDead(prevMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead &&
                               prevArmMotion[arm] == ArmMotionRequest::none);

      const int armInterpolationTime(afterPlayDead ? playDeadDelay : armInterPolationTimes[armMotionSelection.targetArmMotion[arm]]);
      interpolate(&(armMotionSelection.armRatios[arm * armMotionSelection.rightArmRatiosOffset]), ArmMotionRequest::numOfArmMotions, armInterpolationTime, armMotionSelection.targetArmMotion[arm]);
    };
    selectArmMotions(Arms::left);
    selectArmMotions(Arms::right);
  }

  if(lastMotion != motionSelection.targetMotion)
    prevMotion = lastMotion;

  if(lastArmMotion[Arms::left] != armMotionSelection.targetArmMotion[Arms::left])
    prevArmMotion[Arms::left] = lastArmMotion[Arms::left];

  if(lastArmMotion[Arms::right] != armMotionSelection.targetArmMotion[Arms::right])
    prevArmMotion[Arms::right] = lastArmMotion[Arms::right];

  lastMotion = motionSelection.targetMotion;
  lastArmMotion[Arms::left] = armMotionSelection.targetArmMotion[Arms::left];
  lastArmMotion[Arms::right] = armMotionSelection.targetArmMotion[Arms::right];

  PLOT("module:MotionSelector:ratios:walk", motionSelection.ratios[MotionRequest::walk]);
  PLOT("module:MotionSelector:ratios:stand", motionSelection.ratios[MotionRequest::stand]);
  PLOT("module:MotionSelector:ratios:specialAction", motionSelection.ratios[MotionRequest::specialAction]);
  PLOT("module:MotionSelector:lastMotion", lastMotion);
  PLOT("module:MotionSelector:prevMotion", prevMotion);
  PLOT("module:MotionSelector:targetMotion", motionSelection.targetMotion);

  lastExecution = theFrameInfo.time;

#ifndef NDEBUG
  const Rangef& ratioLimits = Rangef::ZeroOneRange();
  for(int i = 0; i < MotionRequest::numOfMotions; ++i)
    ASSERT(ratioLimits.isInside(motionSelection.ratios[i]));
#endif
}

void MotionSelector::interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion)
{
  // increase / decrease all ratios according to target motion
  const unsigned deltaTime(theFrameInfo.getTimeSince(lastExecution));
  float delta(static_cast<float>(deltaTime) / interpolationTime);
  ASSERT(SystemCall::getMode() == SystemCall::logfileReplay || delta > 0.00001f);
  float sum(0);
  for(int i = 0; i < amount; i++)
  {
    if(i == targetMotion)
      ratios[i] += delta;
    else
      ratios[i] -= delta;
    ratios[i] = std::max(ratios[i], 0.0f); // clip ratios
    sum += ratios[i];
  }
  ASSERT(sum != 0);
  // normalize ratios
  for(int i = 0; i < amount; i++)
  {
    ratios[i] /= sum;
    if(std::abs(ratios[i] - 1.f) < 0.00001f)
      ratios[i] = 1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
  }
}