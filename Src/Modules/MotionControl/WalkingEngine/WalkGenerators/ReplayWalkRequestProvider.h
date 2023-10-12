/**
 * @file ReplayWalkRequestProvider.h
 *
 * This file declares a module, that can replay a recorded walk request on a robot to test the walking.
 * For information how to use this module, take a look in our wiki.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/ReplayWalkRequestGenerator.h"
#include "Framework/Module.h"
#include "Math/Pose2f.h"

STREAMABLE(ReplayRequest,
{,
  (WalkKickStep) walkKickStep,
  (bool)(false) isWalkKickPhase,
  (Pose2f) step,
  (float)(0.f) delay,
});

STREAMABLE(WalkPhaseData,
{,
  (std::string)("NONE") description,
  (bool) isLeftPhaseStart,
  (std::vector<ReplayRequest>) walkRequests,
});

MODULE(ReplayWalkRequestProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(KeyStates),
  REQUIRES(WalkGenerator),
  USES(WalkStepData),
  PROVIDES(ReplayWalkRequestGenerator),
  LOADS_PARAMETERS(
  {,
    (std::vector<WalkPhaseData>) motionRequests,
  }),
});

class ReplayWalkRequestProvider : public ReplayWalkRequestProviderBase
{
private:
  int currentWalkRequest = -1; /**< Current walk request block. */
  int indexCurrentWalkRequest; /**< Index for the current request inside the walk request block. */
  unsigned lastSwitch = 0;
  bool lastBumperState = false;
  bool initRecord = true;
  bool recordWalkPhase = false;
  unsigned lastSaveTimestamp = 0;
  unsigned lastStepTargetCopyTimestamp;
  Pose2f actualLastStepTarget;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param generator The representation updated.
   */
  void update(ReplayWalkRequestGenerator& request) override;
};
