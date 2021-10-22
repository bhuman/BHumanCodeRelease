/**
 * @file KickEngine.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Philip Reichenberg
 */

#pragma once

#include "KickEngineData.h"
#include "KickEngineParameters.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/KickGenerator.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Streams/InStreams.h"

MODULE(KickEngine,
{,
  USES(JointRequest),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(KickGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(500.f) maxInterpolationTime, // Max interpolation time when the kickPhase started
    (Angle)(50_deg) interpolationSpeed, // Max speed the joints are allowed to move when interpolating at the start of the kickPhase
    (float)(200) minFootGroundContactTime, // For ForwardFast kick, both feet must have ground contact for at least this time
  }),
});

class KickEngine : public KickEngineBase
{
public:
  std::vector<KickEngineParameters> params;
  KickEngine();

  void update(KickGenerator& kickGenerator) override;
};

struct KickPhase : MotionPhase
{
  explicit KickPhase(KickEngine& engine, const KickRequest& kickRequest, const MotionPhase& lastPhase, const JointRequest& jointRequest);

private:

  KickEngine& engine;

  KickEngineData data;
  bool compensate = false;
  bool compensated = false;
  bool isInterpolating = true;
  bool adjustStartArmPosition = true; // if the arms are on the back, move them to the side
  int boostState = 0;
  unsigned startTime = 0;
  unsigned bothFeetGroundContactTimestamp = 0;
  KickRequest currentKickRequest;
  bool leftArmInterpolation = false;
  bool rightArmInterpolation = false;

  JointRequest previousRequest; /**< JointRequest of previous engine. */
  JointRequest jointRequestOutput; /**< JointRequest of previous engine. */
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;
  unsigned freeLimbs() const override;
  float calcInterpolationIntoStand(JointRequest& jointRequest, JointRequest& prevRequest, const Angle& interpolationSpeedFactor);
  void interpolationState(JointRequest& jointRequest, JointRequest& prevRequest, const Angle& interpolationSpeedFactor);
};
