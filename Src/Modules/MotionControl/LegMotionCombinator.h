/**
 * @file Modules/MotionControl/LegMotionCombinator.h
 * This file declares a module that combines the leg motions created by the different modules.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Tools/Module/Module.h"

MODULE(LegMotionCombinator,
{,
  REQUIRES(GetUpEngineOutput),
  REQUIRES(HeadJointRequest),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegMotionSelection),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StandLegRequest),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkLegRequest),

  PROVIDES(LegJointRequest),
});

class LegMotionCombinator : public LegMotionCombinatorBase
{
private:
  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */

  void update(LegJointRequest& legJointRequest);
};
