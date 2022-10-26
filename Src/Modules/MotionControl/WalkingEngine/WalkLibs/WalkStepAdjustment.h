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
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Range.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(SoleRotationParameter,
{,
  (Angle) minTorsoRotation, /**< The torso must have a minimum of this rotation in y axis, to allow the swing sole rotation compensation in y axis. */
  (Angle) minSoleRotation, /**< The support sole rotation must have a minimum of this rotation in y axis, to allow the swing sole rotation compensation in y axis. */
  (float) soleBackwardsCompensationTorsoFactor, /**< Factor for how much the torso rotation has an influence. */
  (float) soleForwardCompensationReturnZeroRation, /**< At this % duration of the step, interpolate the forward compensation back to 0. */
  (float) soleBackwardsCompensationReturnZeroRatio, /**< At this % duration of the step, interpolate the backward compensation back to 0. */
  (float) soleCompensationReduction, /**< If the swing foot moves in direction X, if shall compensate less in the opposite direction. */
  (float) soleCompensationIncreasement, /**< If the swing foot moves in direction X, if shall compensate more (up to 100%) in the same direction. */
  (Angle) maxBackCompensation, /**< Clip the back compensation (which increased the pitch rotation). */
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
  (float) reduceWalkingSpeedTimeWindow, /**< If the step adjustment adjusted the feet too much two separat times in this time duration, then reduce the walking speed. */
});

class WalkStepAdjustment
{
private:
  Vector3f lastMeasuredComInFoot = Vector3f::Zero(); // last measured com, rotated with the last rotation matrix
  Vector3f lastMeasuredCom = Vector3f::Zero(); // last measured com
  std::vector<Vector3f> supportPolygon; // simple support polygon
  Vector2f supportFootCenter; // center of the support polygon
  RotationMatrix lastMeasuredRotationMatrix; // rotation matrix from previous motion frame
  int updateSteps = 3; // predicted torso tilt 3 frames into the future
  float lowPassFilteredComX; // low pass filtered com in x axis
  Pose3f lastLeft; // left foot pose from previous motion frame
  Pose3f lastRight; // right foot pose from previous motion frame
  float swingFootXTranslationChange; // last swing x translation change. Used to scale the sole rotation compensation
  float backwardsWalkingRotationCompensationFactor; // current factor for the back rotation compensation
  float forwardsWalkingRotationCompensationFactor; // current factor for the forward rotation compensation
  bool allowTouchingTheBallForBalancing = false; // If the robot is tilting too much forward, allow walking into the ball to prevent falling

public:

  // Flags used by the walkPhase to determine the "special" gyro balancing in WalkPhaseBase::addGyroBalance()
  bool isForwardBalance = false;
  bool forwardBalanceWasActive = false;
  bool isBackwardBalance = false;
  bool backwardBalanceWasActive = false;

  float lastLeftAdjustmentX; // left foot adjustment, to prevent falling
  float lastRightAdjustmentX; // right foot adjustment, to prevent falling

  float highestAdjustmentX; // highest adjustment of current step, with sign. If current adjustment is 0, then highestAdjustment is reset to 0 too.
  float highestNegativeAdjustmentX; // highest negativ adjustment of current step
  float previousHighestAdjustmentX; // highest adjustment of previous step, with sign

  float kneeHipBalanceCounter; // count motion phases since the last time the step adjustment adjusted backwards

  float balanceComIsForward = 0.f;
  float hipBalanceIsSafeBackward; // hip can be used for balancing backward
  float hipBalanceIsSafeForward; // hip can be used for balancing forward

  float sideBalanceFactor = 0.f;

  float delta = 0.f;

  unsigned lastLargeBackwardStep;
  unsigned lastNormalStep;
  int reduceWalkingSpeed;

  WalkStepAdjustment();

  /**
   * LIPM model to predict the rotation matrix
   * @param rotationMatrix current rotation matrix
   * @param robotModel current robot model
   * @param footOffset the calibrated foot offsets
   * @param isLeftPhase is left foot the swing foot?
   * @param prediction if false, only predict for one frame
   *
   * @return predicted rotation matrix for updateSteps motion frames into the future
   */
  RotationMatrix predictRotation(RotationMatrix rotationMatrix, const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase, const bool prediction);

  /**
   * Calulate the intersection point of the CoM movement vector and the the support polygon edges
   * @param currentCom current CoM
   * @param lastCom previous CoM
   * @param intersection3D the resulting intersection point
   *
   * @return whether the movement vector intersects the support polygon
   */
  [[nodiscard]] bool getTiltingPoint(const Vector3f& currentCom, const Vector3f& lastCom, Vector3f& intersection3D);

  /**
   * Calculate the support polygon
   * @param robotModel the robot model
   * @param footOffset the foot offsets
   * @param isLeftPhase is the left foot the swing foot
   */
  void calcSupportPolygon(const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase);

  /*
   * Adjust the position of the feet, to ensure that the robot does not fall.
   * If the com is too far to the front, adjust the foot that is more to the front (most of the time it will be the support foot)
   * If the com is too far to the back, adjust the foot that is more to the back (most of the time it will be the swing foot)
   * Adjustment is made in x-translation in robot coordinate for the corresponding foot. In case of a turn step, the offset is counter rotated
   * so foot is actually moving forward/backward, independent of the current feet rotation.
   * @param left The left foot request from the walkGenerator
   * @param right The right foot request from the walkGenerator
   * @param stepTime the current ratio, of how much the current step finished
   * @param comX x- and y-axis of com in foot plane
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
   * @param theJointPlayFactor factor how bad the joint state is
   * @param groundContact does the robot has ground contact?
   * @param walkStepParams the parameters for the walk step adjustment
   */
  void addBalance(Pose3f& left, Pose3f& right, const float stepTime, const Vector2f& com, const FootOffset& footOffset,
                  const Rangef& clipForwardPosition, const bool isLeftPhase, const FootSupport& footSupport,
                  const FsrData& fsrData, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                  const int reduceWalkingSpeedStepAdjustmentSteps, const Vector2f& ball, const float clipAtBallDistanceX,
                  const float& theJointPlayFactor, const bool groundContact, const WalkStepAdjustmentParams& walkStepParams);

  /**
   * Reset the WalkStepAdjustment based on the last one
   * @param walkData The last WalkStepAdjustment object
   * @param reset Should the whole WalkStepAdjustment object get resetted (except for the modeling part), or just update based on the last one?
   * @param frameInfo Threshold for how much backwards adjustment is allowed before the walking speed is reduced
   * @param unstableWalkThreshold Threshold
   */
  void init(const WalkStepAdjustment& walkData, const bool reset, const FrameInfo& frameInfo,
            const float unstableWalkThreshold);

  /**
   * If the torso and the support foot sole are tilted too much forward, then compensate the swing foot sole forward rotation, to prevent it to move into the ground
   * @param supportSole The support foot sole
   * @param requestedSupportSole The requested support foot sole
   * @param swingRotation The swing foot sole y rotation offset
   * @param oldSwingRotation The old swing foot sole y rotation offset
   * @param torsoRotation The inertialData y angle
   * @param stepRatio Current ratio of the step (t / stepDuration)
   * @param soleRotationOffsetSpeed Allowed speed to adjust the sole rotation
   * @param useTorsoAngle Can we trust the torso rotation and use it for the rotation error?
   * @param hipRotation current hip rotation to compensate for the arms
   * @param soleRotParams the sole compensation parameters
   * @param swingSole the full swing sole request
   */
  void modifySwingFootRotation(const Pose3f& supportSole, const Pose3f& requestedSupportSole, Angle& swingRotation, const Angle oldSwingRotation,
                               const Angle torsoRotation, const float stepRatio, const Angle soleRotationOffsetSpeed,
                               const bool useTorsoAngle, const Angle hipRotation,
                               const SoleRotationParameter& soleRotParams, const Pose3f& swingSole);
};
