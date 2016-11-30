/**
 * @file WalkingEngine.h
 * Declaration a module that creates the walking motions
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#pragma once

#include "WalkingEngineUtils.h"
#include "../WalkKickEngine/WalkKickEngine.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineState.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Range.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Module/Module.h"

using WalkingEngineUtils::LegPosture;
using WalkingEngineUtils::MotionType;
using WalkingEngineUtils::PendulumPhase;
using WalkingEngineUtils::PhaseType;
using WalkingEngineUtils::Posture;
using WalkingEngineUtils::StepSize;
using WalkingEngineUtils::SubPhaseParameters;

MODULE(WalkingEngine,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(MotionRequest),
  REQUIRES(LegMotionSelection),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkKicks),
  PROVIDES(WalkingEngineState),
  REQUIRES(WalkingEngineState),
  PROVIDES(WalkArmRequest),
  PROVIDES(StandArmRequest),
  LOADS_PARAMETERS(
  {
    virtual void onRead() {}, /**< Walking Engine calls init() whenever parameters are updated parameters */

    (float) comHeight, /**< The heicht of the center of mass */
    (float) feetDistance, /**< The distance between the feet on the y-axis */
    (float) standBodyTilt, /**< The tilt of the torso when standing */
    (Vector2f) standArmJointAngles, /**< The joint angles of the left arm when standing */

    (Vector2f) walkRef, /**< The position of the pendulum pivot point in Q */
    (Vector2f) walkRefAtFullSpeedX, /**< The position of the pendulum pivot point when walking forwards with maximum speed */
    (float) walkRefBackwards,
    (Rangef) walkRefXPlanningLimit, /**< The limit for shifting the pendulum pivot point towards the x-axis when planning the next step size */
    (Rangef) walkRefXLimit, /**< The limit for shifting the pendulum pivot point towards the x-axis when balancing */
    (Rangef) walkRefYLimit, /**< The limit for shifting the pendulum pivot point towards the y-axis when balancing */
    (Rangef) walkStepSizeXPlanningLimit, /**< The minimum and maximum step size used to plan the next step size */
    (Rangef) walkStepSizeXLimit, /**< The minimum and maximum step size when balancing */
    (float) walkStepDuration, /**< the duration of a full step cycle (two half steps) */
    (float) walkStepDurationAtFullSpeedX, /**< the duration of a full step cycle when walking forwards with maximum speed */
    (float) walkStepDurationAtFullSpeedY, /**< the duration of a full step cycle when walking sidewards with maximum speed */
    (Vector2f) walkHeight, /**< the height of the 3d linear inverted pendulum plane (for the pendulum motion towards the x-axis and the pendulum motion towards the y-axis) */
    (float) walkArmRotationAtFullSpeedX, /**< The maximum deflection for the arm swinging motion */
    (SubPhaseParameters) walkMovePhase, /**< The beginning and length of the trajectory used to move the swinging foot to its new position */
    (SubPhaseParameters) walkLiftPhase, /**< The beginning and length of the trajectory used to lift the swinging foot */
    (Vector3f) walkLiftOffset, /**< The height the swinging foot is lifted */
    (Vector3f) walkLiftOffsetAtFullSpeedX, /**< The height the swinging foot is lifted when walking full speed in x-direction */
    (Vector3f) walkLiftOffsetAtFullSpeedY, /**< The height the swinging foot is lifted when walking full speed in y-direction */
    (Vector3f) walkLiftRotation, /**< The amount the swinging foot is rotated while getting lifted */
    (float) walkSupportRotation, /**< A rotation added to the supporting foot to boost the com acceleration */
    (Vector3f) walkComLiftOffset, /**< The height the center of mass is lifted within a single support phase */

    (Pose2f) speedMax, /**< The maximum walking speed (in "size of two steps") */
    (float) speedMaxBackwards, /**< The maximum walking speed for backwards walking (in "size of two steps") */
    (Pose2f) speedMaxChange, /**< The maximum walking speed acceleration/descelleration */
    (Pose2f) speedMaxChangeBeforeTarget, /**< The maximum walking speed deceleration that is used to avoid overshooting of the walking target */

    (bool) balance, /**< Whether sensory feedback should be used or not */
    (Vector2f) balanceCom, /**< A measured center of mass position adoption factor */
    (Vector2f) balanceComVelocity, /**< A measured center of mass velocity adoption factor */
    (Vector2f) balanceRef, /**< A pendulum pivot point p-control factor */
    (Vector2f) balanceNextRef, /**< A pendulum pivot point of the upcoming single support phase p-control factor */
    (Vector2f) balanceStepSize, /**< A step size i-control factor */

    (float) observerMeasurementDelay, /**< The delay between setting a joint angle and the ability of measuring the result */
    (Vector4f) observerProcessDeviation, /**< The noise of the filtering process that estimates the position of the center of mass */
    (Vector2f) observerMeasurementDeviation, /**< The measurement uncertainty of the computed "measured" center of mass position */

    (Pose2f) odometryScale, /**< A scaling factor for computed odometry data */
  }),
});

/**
 * A module that creates walking motions using a three-dimensional linear inverted pendulum
 */
class WalkingEngine : public WalkingEngineBase
{
public:
  WalkingEngine();

private:
  class PendulumPlayer
  {
  public:
    WalkingEngine* engine;

    PendulumPhase phase;
    PendulumPhase nextPhase;

    PendulumPlayer() : phase(Array2f::Ones()), nextPhase(Array2f::Ones()) {};

    void seek(float deltaTime);

    void getPosture(LegPosture& posture, float* leftArmAngle, float* rightArmAngle, Pose2f* stepOffset);
    void getPosture(Posture& posture);

  private:
    /**
     * A smoothed blend function with f(0)=0, f'(0)=0, f(1)=1, f'(1)=0, f(2)=0, f'(2)=0
     * @param x The function parameter [0...2]
     * @return The function value
     */
    float blend(float x);
  };

  // computed parameters
  RotationMatrix standBodyRotation; /**< The basic orientation of the torso */
  float walkPhaseDuration; /**< The basic duration of a single support phase */
  float walkPhaseDurationAtFullSpeedX; /**< The duration of single support phase when walking full with full speed in x-direction */
  float walkPhaseDurationAtFullSpeedY; /**< The duration of single support phase when walking full with full speed in y-direction */
  Rangef walkXvdXPlanningLimit; /**< A limit of the center of mass velocity used to plan the center of mass trajectory */
  Rangef walkXvdXLimit; /**< A limit of the center of mass to protect the walking engine from executing steps that are too large */

  /**
   * Intercept parameter streaming to compute derived paramaters.
   * Note that this does not work during the construction of the module.
   * @param in The stream from which the object is read
   * @param out The stream to which the object is written.
   */
  void onRead() override;

  /** Initialize derived parameters. */
  void init();

  void reset();

  /**
   * The central update method to generate the walking motion.
   */
  void update(WalkingEngineState& walkingEngineState) override;
  MotionType currentMotionType;
  ArmJointRequest armJointRequest;
  PendulumPlayer pendulumPlayer;
  PendulumPlayer predictedPendulumPlayer;
  WalkKickEngine wke;

  void update(WalkArmRequest& walkArmRequest) override { static_cast<ArmJointRequest&>(walkArmRequest) = armJointRequest; }
  void update(StandArmRequest& standOutput) override { static_cast<ArmJointRequest&>(standOutput) = armJointRequest; }

  void updateMotionRequest();
  MotionType requestedMotionType;
  Pose2f requestedWalkTarget;
  Pose2f lastCopiedWalkTarget;

  void updatePendulumPlayer();
  Vector2f observedComOffset = Vector2f::Zero();

  void computeMeasuredPosture();
  Vector3f measuredLeftToCom = Vector3f::Zero();
  Vector3f measuredRightToCom = Vector3f::Zero();

  void computeExpectedPosture();
  Vector3f expectedLeftToCom = Vector3f::Zero();
  Vector3f expectedRightToCom = Vector3f::Zero();
  Vector2f expectedComVelocity = Vector2f::Zero();
  Pose2f observedStepOffset;

  void computeEstimatedPosture();
  Vector3f estimatedLeftToCom = Vector3f::Zero();
  Vector3f estimatedRightToCom = Vector3f::Zero();
  Vector2f estimatedComVelocity = Vector2f::Zero();
  Vector3f lastExpectedLeftToCom = Vector3f::Zero();
  Vector3f lastExpectedRightToCom = Vector3f::Zero();
  Vector2f lastExpectedComVelocity = Vector2f::Zero();
  Matrix3f covX = Matrix3f::Identity();
  Matrix3f covY = Matrix3f::Identity();

  void computeError();
  Vector3f errorLeft = Vector3f::Zero();
  Vector3f errorRight = Vector3f::Zero();
  Vector2f errorVelocity = Vector2f::Zero();

  void correctPendulumPlayer();

  void updatePredictedPendulumPlayer();
  void applyCorrection(PendulumPhase& phase, PendulumPhase& nextPhase);
  Vector2f measuredPx = Vector2f::Zero(); /**< The measured com position (in Q) */
  Vector2f measuredR = Vector2f::Zero(); /**< The measured "zmp" */

  void generateTargetPosture();
  Posture targetPosture;

  void generateArmJointRequest();

  void generateWalkingEngineState(WalkingEngineState& walkingEngineState);

  void generateFirstPendulumPhase(PendulumPhase& phase);
  void generateNextPendulumPhase(const PendulumPhase& phase, PendulumPhase& nextPhase, const Pose2f& lastStepOffset);
  RingBuffer<PendulumPhase, 5> phaseBuffer;
  void computeNextPendulumParametersY(PendulumPhase& nextPhase, float walkPhaseDurationX, float walkPhaseDurationY) const;
  void computeNextPendulumParametersX(PendulumPhase& nextPhase) const;

  void updatePendulumPhase(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const;
  void repairPendulumParametersY(PendulumPhase& phase, const PendulumPhase& nextPhase) const;
  void updatePendulumParametersY(PendulumPhase& phase, PendulumPhase& nextPhase) const;
  void updatePendulumParametersX(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const;
  void updatePendulumParametersToStand(LIP& com, float& r, float targetPosition, float targetVelocity, float timeRemaining) const;

  /**
   * @parameter lastStepOffset The distance the swing foot traveled in the last step.
   */
  void generateNextStepSize(PhaseType nextSupportLeg, StepSize& stepSize, const Pose2f& lastStepOffset);

  void computeOdometryOffset();
  Pose2f odometryOffset;
  Pose3f lastFootLeft;
  Pose3f lastFootRight;
  Vector3f lastOdometryOrigin = Vector3f::Zero();
  Pose2f upcomingOdometryOffset;

  void drawZmp();
  void drawStats();
  void plot();
};
