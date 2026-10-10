/**
 * @file PrepareEngine.h
 *
 * Bring the robot into a static standing pose to carry the robot around safely.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/PrepareGenerator.h"
#include "Representations/Sensing/InertialData.h"

MODULE(PrepareEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  PROVIDES(PrepareGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(2000) interpolationTime,
    (float)(4000) energySaving,
    (Angle)(-83_deg) shoulderRoll,
    (Angle)(0_deg) elbowYaw,
    (Rangea)(15_deg, 60_deg) stiffnessTorsoRange,
    (Rangef)(25.f, 100.f) stiffnessRange,
    (Angle)(20_deg) headFallAngle,
    (Angle)(8_deg) maxJointDiff,
  }),
});

class PrepareEngine : public PrepareEngineBase
{
public:
  PrepareEngine();
  void update(PrepareGenerator& thePrepareGenerator) override;

  JointAngles target;
};

struct PreparePhase : MotionPhase
{
  explicit PreparePhase(PrepareEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;

  PrepareEngine& engine;

  unsigned phaseStart = 0;
  JointAngles start;
};
