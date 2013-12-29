/**
* @file Modules/MotionControl/MotionSelector.cpp
* This file implements a module that is responsible for controlling the motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
*/

#include <algorithm>
#include "MotionSelector.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(MotionSelector, Motion Control)

PROCESS_WIDE_STORAGE(MotionSelector) MotionSelector::theInstance = 0;

void MotionSelector::stand()
{
  if(theInstance)
  {
#ifdef TARGET_ROBOT
    theInstance->forceStand = true;
#endif
  }
}

void MotionSelector::update(MotionSelection& motionSelection)
{
  static int interpolationTimes[MotionRequest::numOfMotions];
  interpolationTimes[MotionRequest::walk] = 790;
  interpolationTimes[MotionRequest::bike] = 200;
  interpolationTimes[MotionRequest::indykick] = 10;
  interpolationTimes[MotionRequest::specialAction] = 200;
  interpolationTimes[MotionRequest::stand] = 600;
  interpolationTimes[MotionRequest::getUp] = 600;
  interpolationTimes[MotionRequest::takeBall] = 600;
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
    if((lastMotion == MotionRequest::walk && (!&theWalkingEngineOutput || theWalkingEngineOutput.isLeavingPossible || !theGroundContactState.contact)) ||
       lastMotion == MotionRequest::stand || // stand can always be left
       (lastMotion == MotionRequest::specialAction && (!&theSpecialActionsOutput || theSpecialActionsOutput.isLeavingPossible)) ||
       (lastMotion == MotionRequest::bike && (!&theBikeEngineOutput || theBikeEngineOutput.isLeavingPossible)) ||
       (lastMotion == MotionRequest::indykick && (!&theIndykickEngineOutput || theIndykickEngineOutput.isLeavingPossible)) ||
       (lastMotion == MotionRequest::getUp && (!&theGetUpEngineOutput || theGetUpEngineOutput.isLeavingPossible)) ||
       (lastMotion == MotionRequest::takeBall && (!&theBallTakingOutput || theBallTakingOutput.isLeavingPossible)) ||
       (requestedMotion == MotionRequest::takeBall && &theBallTakingOutput && lastMotion == MotionRequest::walk)) 
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

    // increase / decrease all ratios according to target motion
    const unsigned deltaTime(theFrameInfo.getTimeSince(lastExecution));
    const int interpolationTime = prevMotion == MotionRequest::specialAction && lastActiveSpecialAction == SpecialActionRequest::playDead ? playDeadDelay : interpolationTimes[motionSelection.targetMotion];
    float delta((float)deltaTime / interpolationTime);
    ASSERT(SystemCall::getMode() == SystemCall::logfileReplay || delta > 0.00001f);
    float sum(0);
    for(int i = 0; i < MotionRequest::numOfMotions; i++)
    {
      if(i == motionSelection.targetMotion)
        motionSelection.ratios[i] += delta;
      else
        motionSelection.ratios[i] -= delta;
      motionSelection.ratios[i] = std::max(motionSelection.ratios[i], 0.0f); // clip ratios
      sum += motionSelection.ratios[i];
    }
    ASSERT(sum != 0);
    // normalize ratios
    for(int i = 0; i < MotionRequest::numOfMotions; i++)
    {
      motionSelection.ratios[i] /= sum;
      if(std::abs(motionSelection.ratios[i] - 1.f) < 0.00001f)
        motionSelection.ratios[i] = 1.f; // this should fix a "motionSelection.ratios[motionSelection.targetMotion] remains smaller than 1.f" bug
    }

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
  }
  lastExecution = theFrameInfo.time;
  if(lastMotion != motionSelection.targetMotion)
    prevMotion = lastMotion;
  lastMotion = motionSelection.targetMotion;

  PLOT("module:MotionSelector:ratios:walk", motionSelection.ratios[MotionRequest::walk]);
  PLOT("module:MotionSelector:ratios:stand", motionSelection.ratios[MotionRequest::stand]);
  PLOT("module:MotionSelector:ratios:specialAction", motionSelection.ratios[MotionRequest::specialAction]);
  PLOT("module:MotionSelector:lastMotion", lastMotion);
  PLOT("module:MotionSelector:prevMotion", prevMotion);
  PLOT("module:MotionSelector:targetMotion", motionSelection.targetMotion);
}
