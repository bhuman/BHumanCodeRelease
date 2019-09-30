/**
 * @file FallEngine.h
 * A minimized motion engine for falling.
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/FallEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"

MODULE(FallEngine,
{,
  REQUIRES(ArmMotionRequest),
  REQUIRES(FallDownState),
  REQUIRES(JointAngles),
  REQUIRES(StiffnessSettings),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(GameInfo),
  REQUIRES(MotionRequest),
  USES(WalkingEngineOutput),
  USES(MotionInfo),
  USES(JointRequest),
  PROVIDES(FallEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) waitAfterFalling,
    (int)(10000) noGameControllerThreshold,
    (Angle)(0_deg) forwardThreshold,
    (Angle)(-0_deg) backwardsThreshold,
  }),
});

class FallEngine : public FallEngineBase
{
public:
  void update(FallEngineOutput& output) override;

private:

  bool headYawInSafePosition = false;
  bool headPitchInSafePosition = false;
  bool shoulderInSafePosition = false;
  JointAngles lastJointsBeforeFall;

  bool handleSpecialCases();

  void safeFall(FallEngineOutput& output) const;
  void safeArms(FallEngineOutput& output) const;
  //void safeFallBack(FallEngineOutput& output) const;
  //void protectedFallFront(FallEngineOutput& output);
  //void safeFallFront(FallEngineOutput& output);
  //void stand(FallEngineOutput& output);
  //void safeFallSidewards(FallEngineOutput& output);
  void centerHead(FallEngineOutput& jointRequest);

  void calcDirection(FallEngineOutput& output);
};
