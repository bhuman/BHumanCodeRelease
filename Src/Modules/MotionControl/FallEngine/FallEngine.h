/**
 * @file FallEngine.h
 *
 * A minimized motion engine for falling.
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"

MODULE(FallEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(MotionInfo),
  REQUIRES(StiffnessSettings),
  PROVIDES(FallGenerator),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) waitAfterFalling, /**< Get up is allowed after this much time has passend. */
    (Angle)(40_deg) fallDirectionChangeThreshold, /**< Fall direction motion is allowed to change when the body rotation exceeds this torso tilt rotation. */
    (Angle)(30_deg) maxGyroToStartGetUp, /**< Get up is allowed when the gyros are lower than this value. */
  }),
});

class FallEngine : public FallEngineBase
{
  void update(FallGenerator& fallGenerator) override;
};

struct FallPhase : MotionPhase
{
  explicit FallPhase(const FallEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  void safeBody(JointRequest& request);
  void safeArms(JointRequest& request);

  const FallEngine& engine;
  FallDownState::Direction fallDirection;
  unsigned startTime;
  JointRequest request;

  bool waitingForGetUp = false;
  bool headYawInSafePosition = false;
  bool headPitchInSafePosition = false;
  bool leftShoulderRollLowStiffness = false;
  bool leftShoulderPitchLowStiffness = false;
  bool rightShoulderRollLowStiffness = false;
  bool rightShoulderPitchLowStiffness = false;
};
