/**
 * @file FreezeEngine.h
 *
 * A motion engine to just hold the last request for a second, or start a fall motion if the robot is hold not upright.
 * This is very helpful whenever a robot will lose the connection between the upper and lower body
 *
 * @author Philip
 */

#pragma once

#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/FreezeGenerator.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Sensing/InertialData.h"
#include "Framework/Module.h"

MODULE(FreezeEngine,
{,
  REQUIRES(FallGenerator),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(MotionRobotHealth),
  PROVIDES(FreezeGenerator),
  DEFINES_PARAMETERS(
  {,
    (int)(4000) freezeTime,
    (Vector2f)(Vector2f(20_deg, 20_deg)) uprightBodyBoundary,
  }),
});

class FreezeEngine : public FreezeEngineBase
{
  void update(FreezeGenerator& theFreezeGenerator) override;
};

struct FreezePhase : MotionPhase
{
  explicit FreezePhase(const FreezeEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;

  const FreezeEngine& engine;
  bool startFallMotion = false; /**< Start a fall motion phase. */
  unsigned reconnectTime; /**< Start of the fall. */
  JointRequest startRequest; /**< The start request. */
};
