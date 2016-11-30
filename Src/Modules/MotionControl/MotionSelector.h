/**
 * @file Modules/MotionControl/MotionSelector.h
 * This file declares a module that is responsible for controlling the motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/ArmKeyFrameEngineOutput.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(MotionSelector,
{,
  USES(ArmKeyFrameEngineOutput),
  USES(SpecialActionsOutput),
  USES(WalkingEngineOutput),
  USES(KickEngineOutput),
  USES(GetUpEngineOutput),
  REQUIRES(FrameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(ArmMotionRequest),
  REQUIRES(GroundContactState),
  PROVIDES(LegMotionSelection),

  REQUIRES(LegMotionSelection),
  PROVIDES(ArmMotionSelection),
});

class MotionSelector : public MotionSelectorBase
{
public:
  MotionSelector();
  ~MotionSelector() { theInstance = nullptr; }

  /**
   * Can be used to overwrite all other motion requests with a stand request.
   * Must be called again in every frame a stand is desired.
   */
  static void sitDown();

private:
  static thread_local MotionSelector* theInstance; /**< The only instance of this module. */

  bool forceSitDown;
  MotionRequest::Motion lastLegMotion = MotionRequest::specialAction;
  MotionRequest::Motion prevLegMotion = MotionRequest::specialAction;
  std::array<ArmMotionSelection::ArmMotion, Arms::numOfArms> lastArmMotion;
  std::array<ArmMotionSelection::ArmMotion, Arms::numOfArms> prevArmMotion;
  unsigned lastExecution;
  SpecialActionRequest::SpecialActionID lastActiveSpecialAction = SpecialActionRequest::playDead;
  std::array<ArmKeyFrameRequest::ArmKeyFrameId, Arms::numOfArms> lastActiveArmKeyFrame;

  void update(LegMotionSelection& legMotionSelection);
  void update(ArmMotionSelection& armMotionSelection);

  void interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion);
};
