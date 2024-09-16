/**
 * @file WalkStepAdjustment.h
 * This file declares our Walk Step Adjustment. See our paper <Step Adjustment for a Robust Humanoid Walk> for more details.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/Rotation.h"
#include "Math/Range.h"
#include "Streaming/AutoStreamable.h"
#include "Tools/Motion/LowPassFilterPR.h"

STREAMABLE(SoleRotationParameter,
{,
  (Angle) minTorsoRotation, /**< The torso must have a minimum of this rotation in y axis, to allow the swing sole rotation compensation in y axis. */
  (Angle) maxTorsoRotation, /**< The torso must have a maximum of this rotation in y axis, to use the maximum compensation. */
  (float) soleCompensationBackwardReduction, /**< Reduction factor for back tilt compensation. */
  (Vector2a) soleCompensationSpeed, /**< Allowed speed for the compensation. */
  (Angle) maxRollAdjustment, /**< Max allowed roll adjustment. */
  (float) measuredErrorFactor, /**< Factor for usage of measured rotation error. */
  (float) reductionTimeFactor, /**< At the end of a walking step, only this much of the adjustment is allowed. */
  (Rangef) measuredErrorTimeScaling, /**< Scale swing sole rotation error over time. */
  (Rangef) timeScaling, /**< Scale swing sole rotation over time. */
  (Rangef) timeScalingRoll, /**< Scale swing sole rotation over time. */
  (Rangef) sideSizeXRotationScaling,

  (Rangea) tiltErrorDiffOffset,
  (float) tiltErrorDiffScaling,
  (Rangea) gyroScaling,
  (Rangea) torsoRange,
  (Rangef) deltaRange,
  (Rangef) maxStepRatioToStart,
  (Angle) minGyro,
  (float) minSideStepSize,
  (Rangef) deltaRangeSecondStep,

  (Rangef) scalingClipRange, /**< Clip the scaling values into this range. */
  (Rangef) minMaxSumScaling, /**< Sum of scalings must be bigger than the min value and get scaled between [ûkneeScalingMin .. 1] */
});

STREAMABLE(WalkStepAdjustmentParams,
{,
  (float) maxVelX, /**< How fast are the feet allowed to move forward, when they get adjusted to ensure stability (in mm/s)? Should be higher than forward maxSpeed / 2. */
  (float) minVelX, /**< Min additional feet speed, that the walk adjustment can make in both directions (in mm/s). */
  (float) removeSpeedX, /**< The walk adjustment can be removed with this extra speed (in mm/s). */
  (float) comLowPassRatio, /**< To which ratio keep old com measurement? */
  (float) unstableBackWalkThreshold, /**< When last max step adjustment was lower than this value, start the gyro balancing with the knee and hip pitch. */
  (Rangef) desiredFootArea, /**< In which area of the feet can the com move, before the feet must be adjusted to ensure stability (in %)? */
  (float) hipBalanceBackwardFootArea, /**< In which area of the feet can the com move, while the hip pitch is used to balancing when walking unstable. */
  (float) unstableWalkThreshold, /**< Thresholds for the lastLeftAdjustment and lastRightAdjustment values to trigger unstable sound. */
  (float) reduceWalkingSpeedTimeWindow, /**< If the step adjustment adjusted the feet too much two separate times in this time duration, then reduce the walking speed. */
});

class WalkStepAdjustment
{
private:
  float lowPassFilteredComX; // low pass filtered com in x axis
  float swingFootXTranslationChange; // last swing x translation change. Used to scale the sole rotation compensation
  float backwardsWalkingRotationCompensationFactor; // current factor for the back rotation compensation
  float forwardsWalkingRotationCompensationFactor; // current factor for the forward rotation compensation
  bool allowTouchingTheBallForBalancing = false; // If the robot is tilting too much forward, allow walking into the ball to prevent falling

  int adjustmentStopCounter = 0;
  int adjustmentResumeCounter = 0;

  bool kneeBalanceActive = false;
  float kneeBalanceFactor = 0.f;

  RingBuffer<Angle, 3> lastLeftPitchRequest;
  RingBuffer<Angle, 3> lastLeftRollRequest;
  RingBuffer<Angle, 3> lastRightPitchRequest;
  RingBuffer<Angle, 3> lastRightRollRequest;

public:

  Pose3f lastLeft; // left foot pose from previous motion frame
  Pose3f lastRight; // right foot pose from previous motion frame
  // Flags used by the walkPhase to determine the "special" gyro balancing in WalkPhaseBase::addGyroBalance()
  bool isForwardBalance = false;
  bool forwardBalanceWasActive = false;
  bool isBackwardBalance = false;
  bool backwardBalanceWasActive = false;

  float lastLeftAdjustmentX; // left foot adjustment, to prevent falling
  float lastRightAdjustmentX; // right foot adjustment, to prevent falling

  float highestAdjustmentX; // highest adjustment of current step, with sign. If current adjustment is 0, then highestAdjustment is reset to 0 too.
  float highestNegativeAdjustmentX; // highest negative adjustment of current step
  float previousHighestAdjustmentX; // highest adjustment of previous step, with sign

  float kneeHipBalanceCounter; // count motion phases since the last time the step adjustment adjusted backwards

  float balanceComIsForward = 0.f;
  float hipBalanceIsSafeBackward; // hip can be used for balancing backward
  float hipBalanceIsSafeForward; // hip can be used for balancing forward

  float delta = 0.f;

  float comPercentPosition[Legs::numOfLegs] = { 0.5f, 0.5f };
  bool lastWasKneeBalance = false;

  unsigned lastLargeBackwardStep;
  unsigned lastNormalStep;
  int reduceWalkingSpeed;

  LowPassFilterPR swingRotYFilter;
  LowPassFilterPR swingRotXFilter;
  LowPassFilterPR kneeBalanceFilter;

  WalkStepAdjustment();

  /*
   * Adjust the position of the feet, to ensure that the robot does not fall.
   * If the com is too far to the front, adjust the foot that is more to the front (most of the time it will be the support foot)
   * If the com is too far to the back, adjust the foot that is more to the back (most of the time it will be the swing foot)
   * Adjustment is made in x-translation in robot coordinate for the corresponding foot. In case of a turn step, the offset is counter rotated
   * so foot is actually moving forward/backward, independent of the current feet rotation.
   * @param left The left foot request from the walkGenerator
   * @param right The right foot request from the walkGenerator
   * @param stepTime the current ratio, of how much the current step finished
   * @param com x- and y-axis of com in foot plane
   * @param footOffset Defines the edges of the feet
   * @param clipForwardPosition max allowed forward position
   * @param isLeftPhase is left foot swing foot
   * @param footSupport the footSupport representation
   * @param fsrData the fsr pressure data
   * @param frameInfo the frameInfo representation
   * @param hipRot the additional hip pitch rotation
   * @param isStepAdjustmentAllowed Use the step adjustment, or only update the variables?
   * @param reduceWalkingSpeedStepAdjustmentSteps Number of walking steps to reduce the walking speed
   * @param ball The ball position relative to the swing foot zero position
   * @param clipAtBallDistanceX The step adjustment shall not move the feet to close to the ball. Adjustment is clipped below the ball x-translation.
   * @param groundContact does the robot has ground contact?
   * @param walkStepParams the parameters for the walk step adjustment
   */
  void addBalance(Pose3f& left, Pose3f& right, const float stepTime, const Vector2f& com, const FootOffset& footOffset,
                  const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                  const FsrData& fsrData, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                  const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f& ball, const float clipAtBallDistanceX,
                  const bool groundContact, const WalkStepAdjustmentParams& walkStepParams);

  /**
   * Reset the WalkStepAdjustment based on the last one
   * @param walkData The last WalkStepAdjustment object
   * @param reset Should the whole WalkStepAdjustment object get reset (except for the modeling part), or just update based on the last one?
   * @param frameInfo Threshold for how much backwards adjustment is allowed before the walking speed is reduced
   * @param unstableWalkThreshold Threshold
   */
  void init(const WalkStepAdjustment& walkData, const bool reset, const FrameInfo& frameInfo,
            const float unstableWalkThreshold);

  /**
   * If the torso is tilted too much forward or backward, then compensate the swing foot sole y rotation, to prevent it to move into the ground
   * Also when tilted too much to the side, the x rotation is adjusted too (WIP)
   * @param swingRotationY Current planned requested swing sole y rotation
   * @param oldSwingRotationY Last planned requested swing sole y rotation
   * @param swingRotationX Current planned requested swing sole x rotation
   * @param oldSwingRotationX Last planned requested swing sole x rotation
   * @param torsoRotationX Current torso x rotation
   * @param torsoRotationY Current torso y rotation
   * @param stepRatio Current % step ratio
   * @param hipRotation Hip rotation to balance arm positions
   * @param isLeftPhase Is left foot the swing one?
   * @param soleRotParams Sole compensation parameters
   * @param measuredSwingSole measured swing sole
   */
  void modifySwingFootRotation(Angle& swingRotationY, const Angle oldSwingRotationY,
                               Angle& swingRotationX, const Angle oldSwingRotationX, const Angle torsoRotationX,
                               const Angle torsoRotationY, const float stepRatio,
                               const Angle hipRotation, const bool isLeftPhase,
                               const SoleRotationParameter& soleRotParams, const Pose3f& measuredSwingSole,
                               const float plannedSideStepChange, Angle& kneeBalance, const Angle oldKneeBalance, const Angle yGyro);
};
