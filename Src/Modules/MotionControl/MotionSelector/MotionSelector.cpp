/**
 * @file Modules/MotionControl/MotionSelector.cpp
 * This file implements a module that is responsible for controlling the motion.
 * @author Thomas Röfer
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include <algorithm>
#include "MotionSelector.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(MotionSelector, motionControl)

MotionSelector::MotionSelector()
{
  lastArmMotion.fill(ArmMotionSelection::specialActionArms);
  prevArmMotion.fill(ArmMotionSelection::specialActionArms);
  lastActiveArmKeyFrame.fill(ArmKeyFrameRequest::useDefault);
}

void MotionSelector::update(LegMotionSelection& legMotionSelection)
{
  if(SystemCall::getMode() == SystemCall::physicalRobot
     && !forceSitDown
     && theCognitionFrameInfo.time
     && theFrameInfo.time > 120000
     && theFrameInfo.getTimeSince(theCognitionFrameInfo.time) > emergencySitDownDelay)
  {
    forceSitDown = true;
    OUTPUT_ERROR("No Data from Cognition to Motion for more than 3 seconds.");
  }

  if(lastExecution)
  {
    MotionRequest::Motion requestedLegMotion = theMotionRequest.motion;

    if(theMotionRequest.motion == MotionRequest::walk && !theGroundContactState.contact)
      requestedLegMotion = MotionRequest::stand;

    if(forceSitDown && (lastLegMotion == MotionRequest::walk || lastLegMotion == MotionRequest::stand))
    {
      requestedLegMotion = MotionRequest::specialAction;
    }

    // check if the target motion can be the requested motion (mainly if leaving is possible)
    if((lastLegMotion == MotionRequest::walk && (theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
       (lastLegMotion == MotionRequest::stand && legMotionSelection.ratios[MotionRequest::stand] == 1.f) || // stand can always be left if fully active
       (lastLegMotion == MotionRequest::specialAction && theSpecialActionsOutput.isLeavingPossible) ||
       (lastLegMotion == MotionRequest::kick && theKickEngineOutput.isLeavingPossible) ||
       (lastLegMotion == MotionRequest::getUp && theGetUpEngineOutput.isLeavingPossible) ||
       (lastLegMotion == MotionRequest::fall && !theFallEngineOutput.active)) //never immediatly leave kick or get up
    {
      legMotionSelection.targetMotion = requestedLegMotion;
    }

    if(theFallEngineOutput.active) // fallengine activates itself
      requestedLegMotion = legMotionSelection.targetMotion = MotionRequest::fall;
    else if(theMotionInfo.motion == MotionRequest::fall && theFallDownState.state == FallDownState::squatting)
      requestedLegMotion = legMotionSelection.targetMotion = MotionRequest::getUp;

    if(requestedLegMotion == MotionRequest::specialAction)
    {
      legMotionSelection.specialActionRequest = theMotionRequest.specialActionRequest;
      if(forceSitDown && (lastLegMotion == MotionRequest::walk || lastLegMotion == MotionRequest::stand))
      {
        legMotionSelection.specialActionRequest.specialAction = SpecialActionRequest::sitDown;
        forceSitDown = false;
      }
    }
    else
    {
      legMotionSelection.specialActionRequest = SpecialActionRequest();
      if(legMotionSelection.targetMotion == MotionRequest::specialAction)
        legMotionSelection.specialActionRequest.specialAction = SpecialActionRequest::numOfSpecialActionIDs;
    }
    int interpolationTime = interpolationTimes[legMotionSelection.targetMotion];
    // When standing, walking may start instantly
    if(legMotionSelection.targetMotion == MotionRequest::walk && lastLegMotion == MotionRequest::stand && legMotionSelection.ratios[MotionRequest::stand] == 1.f)
      interpolationTime = 1;
    if(legMotionSelection.ratios[MotionRequest::getUp] > 0.f && legMotionSelection.ratios[MotionRequest::getUp] < 1.f)
      interpolationTime = interpolationAfterGetUp;

    const bool afterPlayDead = prevLegMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead;
    const int bodyInterpolationTime = afterPlayDead ? playDeadDelay : interpolationTime;
    interpolate(legMotionSelection.ratios.data(), MotionRequest::numOfMotions, bodyInterpolationTime, legMotionSelection.targetMotion);

    if(legMotionSelection.ratios[MotionRequest::specialAction] < 1.f)
    {
      if(legMotionSelection.targetMotion == MotionRequest::specialAction)
        legMotionSelection.specialActionMode = LegMotionSelection::first;
      else
        legMotionSelection.specialActionMode = LegMotionSelection::deactive;
    }
    else
      legMotionSelection.specialActionMode = LegMotionSelection::active;

    if(legMotionSelection.specialActionMode == LegMotionSelection::active && legMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::numOfSpecialActionIDs)
      lastActiveSpecialAction = legMotionSelection.specialActionRequest.specialAction;
  }

  if(lastLegMotion != legMotionSelection.targetMotion)
    prevLegMotion = lastLegMotion;

  lastLegMotion = legMotionSelection.targetMotion;

  PLOT("module:MotionSelector:lastLegMotion", lastLegMotion);
  PLOT("module:MotionSelector:prevLegMotion", prevLegMotion);
  PLOT("module:MotionSelector:targetLegMotion", legMotionSelection.targetMotion);

#ifndef NDEBUG
  const Rangef ratioLimits = Rangef::ZeroOneRange();
  for(int i = 0; i < MotionRequest::numOfMotions; ++i)
    ASSERT(ratioLimits.isInside(legMotionSelection.ratios[i]));
#endif
}

void MotionSelector::update(ArmMotionSelection& armMotionSelection)
{
  if(lastExecution)
  {
    for(unsigned i = 0; i < Arms::numOfArms; ++i)
    {
      Arms::Arm arm = static_cast<Arms::Arm>(i);

      const bool isNone = theArmMotionRequest.armMotion[arm] == ArmMotionRequest::none;

      ArmMotionSelection::ArmMotion requestedArmMotion;
      switch(theLegMotionSelection.targetMotion)
      {
        case MotionRequest::kick:
          requestedArmMotion = ArmMotionSelection::kickArms;
          break;
        case MotionRequest::getUp:
          requestedArmMotion = ArmMotionSelection::getUpArms;
          break;
        case MotionRequest::specialAction:
        {
          //forceNone = !theSpecialActionsOutput.isArmLeavingAllowed;
          if(isNone ||
             (theLegMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::standHigh &&
              theLegMotionSelection.specialActionRequest.specialAction != SpecialActionRequest::standHighLookUp)) //FIXME QUICKHACK
            requestedArmMotion = ArmMotionSelection::specialActionArms;
          else
            requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
        }
        break;
        case MotionRequest::walk:
        {
          if(isNone)
            requestedArmMotion = ArmMotionSelection::walkArms;
          else
            requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
          break;
        }
        case MotionRequest::stand:
        {
          if(isNone)
            requestedArmMotion = ArmMotionSelection::standArms;
          else
            requestedArmMotion = ArmMotionSelection::ArmMotion(ArmMotionSelection::firstNonBodyMotion + theArmMotionRequest.armMotion[arm] - 1);
          break;
        }
        case MotionRequest::fall:
          requestedArmMotion = ArmMotionSelection::fallArms;
          break;
        default:
          FAIL("Unknown target motion.");
          return;
      }

      static const unsigned armMotionsToClear = bit(ArmMotionSelection::keyPoseS);
      static const unsigned supersedeArmMotions = bit(ArmMotionSelection::keyPoseS) | bit(ArmMotionSelection::clearS);
      // check if the target armmotion can be the requested armmotion (mainly if leaving is possible)
      if(!(lastArmMotion[arm] == ArmMotionSelection::clearS && !theClearArmEngineOutput.cleared[arm]))
      {
        if(requestedArmMotion != ArmMotionSelection::keyFrameS && !theArmKeyFrameEngineOutput.arms[arm].isFree && theLegMotionSelection.targetMotion != MotionRequest::getUp
           && theLegMotionSelection.targetMotion != MotionRequest::kick && theLegMotionSelection.targetMotion != MotionRequest::fall)
        {
          armMotionSelection.targetArmMotion[arm] = ArmMotionSelection::keyFrameS;
          armMotionSelection.armKeyFrameRequest.arms[arm].fast = false;
          armMotionSelection.armKeyFrameRequest.arms[arm].motion = ArmKeyFrameRequest::reverse;
        }
        else if(!(bit(requestedArmMotion) & supersedeArmMotions)
                && bit(lastArmMotion[arm]) & armMotionsToClear
                && !(theClearArmEngineOutput.cleared[arm] && theClearArmEngineOutput.providingOutput[arm]))
          armMotionSelection.targetArmMotion[arm] = ArmMotionSelection::clearS;
        else
        {
          armMotionSelection.targetArmMotion[arm] = requestedArmMotion;
          if(armMotionSelection.targetArmMotion[arm] == ArmMotionSelection::keyFrameS)
            armMotionSelection.armKeyFrameRequest.arms[arm] = theArmMotionRequest.armKeyFrameRequest.arms[arm];
        }
      }

      const bool afterPlayDead = prevLegMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead &&
                                 prevArmMotion[arm] == ArmMotionSelection::specialActionArms;
      const int armInterpolationTime = afterPlayDead ? playDeadDelay : armInterPolationTimes[armMotionSelection.targetArmMotion[arm]];
      interpolate(armMotionSelection.armRatios[arm].data(), ArmMotionSelection::numOfArmMotions, armInterpolationTime, armMotionSelection.targetArmMotion[arm]);
    }
  }

  if(lastArmMotion[Arms::left] != armMotionSelection.targetArmMotion[Arms::left])
    prevArmMotion[Arms::left] = lastArmMotion[Arms::left];

  if(lastArmMotion[Arms::right] != armMotionSelection.targetArmMotion[Arms::right])
    prevArmMotion[Arms::right] = lastArmMotion[Arms::right];

  lastArmMotion[Arms::left] = armMotionSelection.targetArmMotion[Arms::left];
  lastArmMotion[Arms::right] = armMotionSelection.targetArmMotion[Arms::right];

  lastExecution = theFrameInfo.time;
}

void MotionSelector::interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion)
{
  // increase / decrease all ratios according to target motion
  const unsigned deltaTime = theFrameInfo.getTimeSince(lastExecution);
  float delta = static_cast<float>(deltaTime) / interpolationTime;
  //ASSERT(SystemCall::getMode() == SystemCall::logFileReplay || delta > 0.00001f);
  float sum = 0;
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
