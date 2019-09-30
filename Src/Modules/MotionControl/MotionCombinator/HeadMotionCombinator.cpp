/**
 * @file Modules/MotionControl/HeadMotionCombinator.cpp
 * This file declares a module that combines the head motions created by the different modules.
 * @author Bernd Poppinga
 * based on a module created by @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author Thomas Röfer
 */

#include "HeadMotionCombinator.h"
#include "MotionCombinator.h"

MAKE_MODULE(HeadMotionCombinator, motionControl)

void HeadMotionCombinator::update(HeadJointRequest& jointRequest)
{
  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
  jointRequests[MotionRequest::kick] = &theKickEngineOutput;
  jointRequests[MotionRequest::fall] = &theFallEngineOutput;
  jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
  jointRequests[MotionRequest::stand] = &theStandLegRequest;
  jointRequests[MotionRequest::getUp] = &theGetUpEngineOutput;

  jointRequest.angles[Joints::headYaw] = theHeadMotionEngineOutput.pan;
  jointRequest.angles[Joints::headPitch] = theHeadMotionEngineOutput.tilt;
  MotionUtilities::copy(*jointRequests[theLegMotionSelection.targetMotion], jointRequest, theStiffnessSettings, Joints::headYaw, Joints::headPitch);

  ASSERT(jointRequest.isValid());

  if(theLegMotionSelection.ratios[theLegMotionSelection.targetMotion] == 1.f)
  {
    lastJointAngles = theJointAngles;
  }
  else // interpolate motions
  {
    const bool interpolateStiffness = !(theLegMotionSelection.targetMotion != MotionRequest::specialAction &&
                                        theLegMotionSelection.specialActionRequest.specialAction == SpecialActionRequest::playDead &&
                                        theLegMotionSelection.ratios[MotionRequest::specialAction] > 0.f); // do not interpolate from play_dead
    for(int i = 0; i < MotionRequest::numOfMotions; ++i)
      if(i != theLegMotionSelection.targetMotion && theLegMotionSelection.ratios[i] > 0.f)
      {
        MotionUtilities::interpolate(*jointRequests[i], *jointRequests[theLegMotionSelection.targetMotion], theLegMotionSelection.ratios[i],
                                     jointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, Joints::headYaw, Joints::headPitch);
      }
  }

  ASSERT(jointRequest.isValid());
}
