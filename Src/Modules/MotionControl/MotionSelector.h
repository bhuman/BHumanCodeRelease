/**
* @file Modules/MotionControl/MotionSelector.h
* This file declares a module that is responsible for controlling the motion.
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
* @author <A href="mailto:allli@tzi.de">Alexander Härtl</A>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/SpecialActionsOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/BikeEngineOutput.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/BallTakingOutput.h"
#include "Representations/MotionControl/IndykickEngineOutput.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/Sensing/GroundContactState.h"

MODULE(MotionSelector)
  USES(SpecialActionsOutput)
  USES(WalkingEngineOutput)
  USES(BikeEngineOutput)
  USES(GetUpEngineOutput)
  USES(BallTakingOutput)
  USES(IndykickEngineOutput)
  REQUIRES(FrameInfo)
  REQUIRES(MotionRequest)
  REQUIRES(GroundContactState)
  PROVIDES_WITH_MODIFY(MotionSelection)
END_MODULE

class MotionSelector : public MotionSelectorBase
{
private:
  static PROCESS_WIDE_STORAGE(MotionSelector) theInstance; /**< The only instance of this module. */

  bool forceStand;
  MotionRequest::Motion lastMotion;
  MotionRequest::Motion prevMotion;
  unsigned lastExecution;
  SpecialActionRequest::SpecialActionID lastActiveSpecialAction;
  void update(MotionSelection& motionSelection);

public:
  /**
  * Can be used to overwrite all other motion requests with a stand request.
  * Must be called again in every frame a stand is desired.
  */
  static void stand();

  /**
  * Default constructor.
  */
  MotionSelector() : lastMotion(MotionRequest::specialAction), prevMotion(MotionRequest::specialAction),
    lastActiveSpecialAction(SpecialActionRequest::playDead)
  {
    theInstance = this;
  }
};
