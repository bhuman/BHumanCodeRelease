/**
 * @file FallEngine.h
 *
 * A minimized motion engine for falling.
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Configuration/StaticJointPoses.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/FallGenerator.h"
#include "Representations/MotionControl/GetUpGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Framework/Module.h"

MODULE(FallEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GetUpGenerator),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(MotionInfo),
  REQUIRES(StaticJointPoses),
  REQUIRES(StiffnessSettings),
  PROVIDES(FallGenerator),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) waitAfterFalling, /**< Get up is allowed after this much time has passed. */
    (Angle)(40_deg) fallDirectionChangeThreshold, /**< Fall direction motion is allowed to change when the body rotation exceeds this torso tilt rotation. */
    (Angle)(10_deg) maxGyroToStartGetUp, /**< Get up is allowed when the gyros are lower than this value. */
    (float)(1000) standInterpolationDuration, /**< Interpolate into stand over this duration. */
    (Angle)(-45_deg) lateFallYAngleBack, /**< Change leg angles the moment the robots is about to touch the ground. */
    (Angle)(35_deg) lateFallYAngleFront, /**< Change leg angles the moment the robots is about to touch the ground. */
    (int)(100) highStiffnessDuration, /**< Duration high stiffness for the arms are used. */
    (int)(140) highStiffnessDurationElbowRoll, /**< Duration high stiffness for the elbow are used. */
    (Rangei)(Rangei(5, 30)) stiffnessArm, /**< Min and max stiffness for the arms. */
    (Rangei)(Rangei(20, 30)) stiffnessLeg, /**< Min and max stiffness for the legs. */
    (Rangei)(Rangei(20, 50)) stiffnessHead, /**< Min and max stiffness for the head. */
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
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;
  void safeBody(JointRequest& request);
  void safeArms(JointRequest& request);
  void setHeadStiffness(JointRequest& request);

  const FallEngine& engine;
  FallDownState::Direction fallDirection; /**< Direction of the fall.*/
  unsigned startTime; /**< Start of the fall.*/
  JointRequest request; /**< The request.*/
  JointAngles afterFallStart; /**< The robot is falling really long. Use this start joint positions to interpolate back into a standing position.*/

  bool mirrorFrontFall = false; /**< Mirror front fall motion? */
  bool lateFallMotion = false; /**< Start with the late fall motion, to compensate for some of the impact force.*/
  bool afterFallMotion = false; /**< Interpolate into a standing position.*/
  bool waitingForGetUp = false; /**< Get up can start.*/
  bool headYawInSafePosition = false; /**< Head yaw stiffness can be set low.*/
  bool headPitchInSafePosition = false; /**< Head pitch stiffness can be set low.*/
  bool leftShoulderRollLowStiffness = false; /**< Left shoulder roll stiffness can be set low.*/
  bool rightShoulderRollLowStiffness = false; /**< Right shoulder roll stiffness can be set low.*/
};
