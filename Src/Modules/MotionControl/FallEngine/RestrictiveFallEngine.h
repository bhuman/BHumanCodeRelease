/**
 * @file RestrictiveFallEngine.h
 *
 * A minimized motion engine for falling.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Framework/Module.h"

MODULE(RestrictiveFallEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GetUpGenerator),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(MotionInfo),
  REQUIRES(RobotDimensions),
  REQUIRES(WalkStepData),
  PROVIDES(FallGenerator),
  LOADS_PARAMETERS(
  {,
    (int) waitAfterFalling, /**< Get up is allowed after this much time has passed (in ms). */
    (int) transitionDelay, /**< Leaving the fall phase is further delayed by this time (in ms). */
    (int) soundTimeDelay,
    (int) unstiffTime, /**< Use damping mode for this duration before interpolating to a stand pose. */
    (int) stopSoundTime,
    (Angle) maxGyroToStartGetUp, /**< Get up is allowed when the gyros are lower than this value. */
    (Angle) maxPositionDifference,
    (Angle) jointSpeed,
    (Angle) uprightThreshold,
    (float) standHeight,
    (bool) allowGetUp,
    (Rangea) stiffnessTorsoRange,
    (Rangef) stiffnessRange,
    (Angle) headFallAngle,
  }),
});

class RestrictiveFallEngine : public RestrictiveFallEngineBase
{
  void update(FallGenerator& fallGenerator) override;
};

struct RestrictiveFallPhase : MotionPhase
{
  explicit RestrictiveFallPhase(const RestrictiveFallEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;
  void setHeadStiffness(JointRequest& request);

  const RestrictiveFallEngine& engine;
  unsigned startTime; /**< Start of the fall. */
  unsigned startTransitionTime; /**< Condition to leave the fall phase is fulfilled since this time. */
  unsigned lastSoundTimestamp = 0;
  unsigned lastStopSoundTimestamp = 0;
  JointRequest lastRequest; /**< Last request. */
  JointRequest targetRequest; /**< Target. */

  bool allowLeavingFallPhase = false;
};
