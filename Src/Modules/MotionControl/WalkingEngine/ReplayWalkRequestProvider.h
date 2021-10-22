/**
 * @file ReplayWalkRequestProvider.h
 *
 * This file declares a module, that can replay a recorded walk request on a robot to test the walking.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/ReplayWalkRequestGenerator.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE(ReplayRequest,
{,
  (WalkKickStep) walkKickStep,
  (bool)(false) isWalkKickPhase,
  (Pose2f) step,
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
  REQUIRES(GameInfo),
  REQUIRES(KeyStates),
  REQUIRES(MotionRequest),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkKickGenerator),
  USES(WalkStepData),
  PROVIDES(ReplayWalkRequestGenerator),
  REQUIRES(RobotInfo),
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
