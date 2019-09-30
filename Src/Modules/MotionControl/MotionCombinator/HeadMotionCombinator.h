/**
 * @file Modules/MotionControl/HeadMotionCombinator.h
 * This file declares a module that combines the head motions created by the different modules.
 * @author Bernd Poppinga
 * based on a module created by @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author Thomas Röfer
 */

#pragma once

#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/HeadMotionEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/FallEngineOutput.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Module/Module.h"

MODULE(HeadMotionCombinator,
{,
  REQUIRES(GetUpEngineOutput),
  REQUIRES(JointAngles),
  REQUIRES(HeadMotionEngineOutput),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegMotionSelection),
  REQUIRES(FallEngineOutput),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(StandLegRequest),

  PROVIDES(HeadJointRequest),
});

class HeadMotionCombinator : public HeadMotionCombinatorBase
{
private:
  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */

  void update(HeadJointRequest& headJointRequest) override;
};
