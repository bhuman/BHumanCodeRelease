/**
 * @file Modules/MotionControl/ArmMotionCombinator.h
 * This file declares a module that combines the arm motions created by the different modules.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * based on a module created by @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 */

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/PointAtEngineOutput.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Tools/Module/Module.h"

MODULE(ArmMotionCombinator,
{,
  REQUIRES(ArmKeyFrameEngineOutput),
  REQUIRES(ArmMotionSelection),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(JointAngles),
  REQUIRES(KickEngineOutput),
  REQUIRES(LegMotionSelection),
  REQUIRES(PointAtEngineOutput),
  REQUIRES(SpecialActionsOutput),
  REQUIRES(StandArmRequest),
  REQUIRES(StiffnessSettings),
  REQUIRES(WalkArmRequest),

  PROVIDES(ArmJointRequest),
  PROVIDES(ArmMotionInfo),
});

class ArmMotionCombinator : public ArmMotionCombinatorBase
{
private:
  NonArmeMotionEngineOutput theNonArmeMotionEngineOutput;
  JointAngles lastJointAngles; /**< The measured joint angles the last time when not interpolating. */
  JointRequest lastJointRequest;

  void update(ArmJointRequest& armJointRequest);
  void update(ArmMotionInfo& armMotionInfo);
};
