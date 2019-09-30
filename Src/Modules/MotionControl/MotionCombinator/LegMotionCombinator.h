/**
 * @file Modules/MotionControl/LegMotionCombinator.h
 * This file declares a module that combines the leg motions created by the different modules.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/FallEngineOutput.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Tools/Module/Module.h"

MODULE(LegMotionCombinator,
{,
  REQUIRES(GetUpEngineOutput),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegMotionSelection),
  REQUIRES(FallEngineOutput),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StandLegRequest),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkingEngineOutput),

  PROVIDES(LegJointRequest),
});

class LegMotionCombinator : public LegMotionCombinatorBase
{
private:
  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */

  void update(LegJointRequest& legJointRequest) override;
};
