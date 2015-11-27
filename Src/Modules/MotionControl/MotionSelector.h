/**
 * @file Modules/MotionControl/MotionSelector.h
 * This file declares a module that is responsible for controlling the motion.
 * @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
 * @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
 * @author Jesse Richter-Klug (updated the arm things)
 */

#pragma once

#include "Representations/MotionControl/DmpKickEngineOutput.h"
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
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(MotionSelector,
{,
  USES(ArmKeyFrameEngineOutput),
  USES(SpecialActionsOutput),
  USES(WalkingEngineOutput),
  USES(KickEngineOutput),
  USES(GetUpEngineOutput),
  USES(DmpKickEngineOutput),
  REQUIRES(FrameInfo),
  REQUIRES(MotionRequest),
  REQUIRES(ArmMotionRequest),
  REQUIRES(GroundContactState),
  PROVIDES(MotionSelection),
  REQUIRES(MotionSelection),
  PROVIDES(ArmMotionSelection),
});

class MotionSelector : public MotionSelectorBase
{
private:
  static PROCESS_LOCAL MotionSelector* theInstance; /**< The only instance of this module. */

  ArmMotionSelection armMotionSelection;

  bool forceStand;
  MotionRequest::Motion lastMotion;
  MotionRequest::Motion prevMotion;
  ArmMotionRequest::ArmMotion lastArmMotion[Arms::numOfArms];
  ArmMotionRequest::ArmMotion prevArmMotion[Arms::numOfArms];
  unsigned lastExecution;
  SpecialActionRequest::SpecialActionID lastActiveSpecialAction;
  ArmKeyFrameRequest::ArmKeyFrameId lastActiveArmKeyFrame[Arms::numOfArms];

  void update(MotionSelection& motionSelection);
  void update(ArmMotionSelection& armMotionSelection) { armMotionSelection = this->armMotionSelection; }

  void interpolate(float* ratios, const int amount, const int interpolationTime, const int targetMotion);

public:
  /**
  * Can be used to overwrite all other motion requests with a stand request.
  * Must be called again in every frame a stand is desired.
  */
  static void stand();

  MotionSelector() : armMotionSelection(),
    lastMotion(MotionRequest::specialAction), prevMotion(MotionRequest::specialAction),
    lastActiveSpecialAction(SpecialActionRequest::playDead)
  {
    lastArmMotion[Arms::left] = lastArmMotion[Arms::right] = prevArmMotion[Arms::left] = prevArmMotion[Arms::right] = ArmMotionRequest::none;
    lastActiveArmKeyFrame[Arms::left] = lastActiveArmKeyFrame[Arms::right] = ArmKeyFrameRequest::useDefault;
    theInstance = this;
  }
};
