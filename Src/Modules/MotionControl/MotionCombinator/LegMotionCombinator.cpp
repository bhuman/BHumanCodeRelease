/**
 * @file Modules/MotionControl/LegMotionCombinator.cpp
 * This file declares a module that combines the leg motions created by the different modules.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author Thomas Röfer
 */

#include "LegMotionCombinator.h"
#include "MotionCombinator.h"
#include "Tools/RobotParts/Joints.h"

MAKE_MODULE(LegMotionCombinator, motionControl)

void LegMotionCombinator::update(LegJointRequest& jointRequest)
{
  const JointRequest* jointRequests[MotionRequest::numOfMotions];
  jointRequests[MotionRequest::walk] = &theWalkingEngineOutput;
  jointRequests[MotionRequest::kick] = &theKickEngineOutput;
  jointRequests[MotionRequest::specialAction] = &theSpecialActionsOutput;
  jointRequests[MotionRequest::stand] = &theStandLegRequest;
  jointRequests[MotionRequest::getUp] = &theGetUpEngineOutput;
  jointRequests[MotionRequest::fall] = &theFallEngineOutput;

  MotionUtilities::copy(*jointRequests[theLegMotionSelection.targetMotion], jointRequest, theStiffnessSettings, Joints::firstLegJoint, Joints::rAnkleRoll);
  switch(theLegMotionSelection.targetMotion)
  {
    case MotionRequest::walk:
      ASSERT(jointRequests[MotionRequest::walk]->isValid());
      break;
    case MotionRequest::kick:
      ASSERT(jointRequests[MotionRequest::kick]->isValid());
      break;
    case MotionRequest::specialAction:
      ASSERT(jointRequests[MotionRequest::specialAction]->isValid());
      break;
    case MotionRequest::stand:
      ASSERT(jointRequests[MotionRequest::stand]->isValid());
      break;
    case MotionRequest::getUp:
      ASSERT(jointRequests[MotionRequest::getUp]->isValid());
      break;
    case MotionRequest::fall:
      ASSERT(jointRequests[MotionRequest::fall]->isValid());
      break;
  }
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
                                     jointRequest, interpolateStiffness, theStiffnessSettings, lastJointAngles, Joints::firstLegJoint, Joints::rAnkleRoll);
        switch(i)
        {
          case MotionRequest::walk:
            ASSERT(jointRequests[MotionRequest::walk]->isValid());
            break;
          case MotionRequest::kick:
            ASSERT(jointRequests[MotionRequest::kick]->isValid());
            break;
          case MotionRequest::specialAction:
            ASSERT(jointRequests[MotionRequest::specialAction]->isValid());
            break;
          case MotionRequest::stand:
            ASSERT(jointRequests[MotionRequest::stand]->isValid());
            break;
          case MotionRequest::getUp:
            ASSERT(jointRequests[MotionRequest::getUp]->isValid());
            break;
          case MotionRequest::fall:
            ASSERT(jointRequests[MotionRequest::fall]->isValid());
            break;
        }
      }
  }

  ASSERT(jointRequest.isValid());
}
