/**
 * @file FallEngine.h
 *
 * A minimized motion engine for falling.
 *
 * @author Bernd Poppinga
 */

#pragma once

#include "Representations/Configuration/StaticJointPoses.h"
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
  PROVIDES(FallGenerator),
  LOADS_PARAMETERS(
  {,
    (int) waitAfterFalling, /**< Get up is allowed after this much time has passed. */
    (Angle) fallDirectionChangeThreshold, /**< Fall direction motion is allowed to change when the body rotation exceeds this torso tilt rotation. */
    (Angle) maxGyroToStartGetUp, /**< Get up is allowed when the gyros are lower than this value. */
    (Angle) lateFallYAngleBack, /**< Change leg angles the moment the robots is about to touch the ground. */
    (Angle) lateFallYAngleFront, /**< Change leg angles the moment the robots is about to touch the ground. */
    (int) highStiffnessDuration, /**< Duration high stiffness for the arms are used. */
    (int) highStiffnessDurationElbowRoll, /**< Duration high stiffness for the elbow are used. */
    (Rangei) stiffnessArm, /**< Min and max stiffness for the arms. */
    (Rangei) stiffnessLeg, /**< Min and max stiffness for the legs. */
    (Rangei) stiffnessHead, /**< Min and max stiffness for the head. */
    (Angle) shoulderPitchSideFallCorrectionRange,
    (Angle) shoulderPitchSideFallCorrectionValue,
    (Angle) shoulderRollBoostAngle, /**< Overwrite shoulder roll by this value. */
    (Angle) elbowYawBoostAngle, /**< Overwrite shoulder yaw by this value. */
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

  bool mirrorFrontFall = false; /**< Mirror front fall motion? */
  bool lateFallMotion = false; /**< Start with the late fall motion, to compensate for some of the impact force.*/
  bool waitingForGetUp = false; /**< Get up can start.*/
  bool headYawInSafePosition = false; /**< Head yaw stiffness can be set low.*/
  bool headPitchInSafePosition = false; /**< Head pitch stiffness can be set low.*/
  bool leftShoulderRollLowStiffness = false; /**< Left shoulder roll stiffness can be set low.*/
  bool rightShoulderRollLowStiffness = false; /**< Right shoulder roll stiffness can be set low.*/
};
