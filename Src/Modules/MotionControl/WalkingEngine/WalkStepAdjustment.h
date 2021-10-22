/**
 * @file WalkStepAdjustment.h
 * This file declares something.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Range.h"

class WalkStepAdjustment
{
private:
  Pose3f lastMeasuredComInFoot; // last measured com, rotated with the last rotation matrix
  Pose3f lastMeasuredCom; // last measured com
  std::vector<Vector3f> supportPolygon; // simple support polygon
  Vector2f supportFootCenter; // center of the support polygon
  Pose3f lastMeasuredRotationMatrix; // rotation matrix from previous motion frame
  int updateSteps = 3; // predicted torso tilt 3 frames into the future
  float lowPassFilteredComX; // low pass filtered com in x axis
  int hipBalanceCounter; // frame counter to balance with the hip pitch joint
  Pose3f lastLeft; // left foot pose from previous motion frame
  Pose3f lastRight; // right foot pose from previous motion frame
  float swingFootXTranslationChange;
  float backwardsWalkingRotationCompensationFactor;
  float forwardsWalkingRotationCompensationFactor;

public:

  // either no hip pitch balancing or only balancing with negativ gyro values
  ENUM(HipGyroBalanceState,
  {,
    noHipBalance,
    backwardHipBalance,
  });

  float lastLeftAdjustmentX; // left foot adjustment, to prevent falling
  float lastRightAdjustmentX; // right foot adjustment, to prevent falling

  float highestAdjustmentX; // highest adjustment of current step, with sign. If current adjustment is 0, then highestAdjustment is reset to 0 too.
  float highestNegativeAdjustmentX; // highest negativ adjustment of current step
  float previousHighestAdjustmentX; // highest adjustment of previous step, with sign

  float kneeHipBalanceCounter; // count motion phases since the last time the step adjustment adjusted backwards

  float hipBalanceIsSafeBackward; // hip can be used for balancing backward
  float hipBalanceIsSafeForward; // hip can be used for balancing forward

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
  Pose3f predictRotation(Pose3f& rotationMatrix, const RobotModel& robotModel, const FootOffset& footOffset, const bool isLeftPhase, const bool prediction);

  /**
   * Calulate the intersection point of the CoM movement vector and the the support polygon edges
   * @param currentCom current CoM
   * @param lastCom previous CoM
   *
   * @return intersection point
   */
  Vector3f getTiltingPoint(const Pose3f& currentCom, const Pose3f& lastCom);

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
   * @param comX x-axis of com in foot plane
   * @param hipBalanceSteps Counter for how many steps the hip balance was last active
   * @param footArea max and min offset relativ to the foot sole origin, in which the com can move freely (in %).
   * @param hipBalanceBackwardFootArea min offset relativ to the foot sole origin, in which the com can move freely (in %), for when the hip can be used for balancing.
   * @param footOffset Defines the edges of the feet
   * @param maxVelX In case a foot was adjusted and the com is inside the footArea again, the foot shall move to the actually requested position from the walkGenerator.
   *        To prevent a too high velocity, the foot is moving with maxVelX as velocity, until it reached the requested position.
   * @param minVelX Even if the feet are already moving in the target direction with a speed equal or higher maxVelX, the additional movement can be atleast equal to minVelX
   * @param comLowPassRatio To which ratio keep old com value?
   * @param clipForwardPosition max allowed forward position
   * @param isLeftPhase is left foot swing foot
   * @param footSupport the footSupport representation
   * @param frameInfo the frameInfo representation
   * @param hipRot the additional hip pitch rotation
   * @param isStepAdjustmentAllowed Use the step adjustment, or only update the variables?
   * @param unstableWalkThreshold Threshold for how much backwards adjustment is allowed before the walking speed is reduced
   * @param reduceWalkingSpeedTimeWindow Time frame (in ms) to check if a walk speed reduction is needed
   * @param reduceWalkingSpeedStepAdjustmentSteps Number of walking steps to reduce the walking speed
   * @param ball The ball position relative to the swing foot zero position
   * @param clipAtBallDistanceX The step adjustment shall not move the feet to close to the ball. Adjustment is clipped below the ball x-translation.
   */
  void addBalance(Pose3f& left, Pose3f& right, const float stepTime, const float comX, const float unstableBackWalkThreshold,
                  const Rangef& footArea, const float hipBalanceBackwardFootArea, const FootOffset& footOffset,
                  const float maxVelX, const float minVelX, const float removeSpeedX,
                  const float comLowPassRatio, const Rangef& clipForwardPosition, const bool isLeftPhase,
                  const FootSupport& footSupport, const FrameInfo& frameInfo, const Angle hipRot, const bool isStepAdjustmentAllowed,
                  const float unstableWalkThreshold, const int reduceWalkingSpeedTimeWindow, const int reduceWalkingSpeedStepAdjustmentSteps,
                  const Vector2f& ball, const float clipAtBallDistanceX);

  /**
   * Reset the WalkStepAdjustment based on the last one
   * @param walkData The last WalkStepAdjustment object
   * @param hipBalance Hip balance state
   * @param unstableBackWalkThreshold Threshold for last step adjustment, to start gyro balancing with knee and hip pitch
   * @param reset Should the whole WalkStepAdjustment object get resetted (except for the modelling part), or just update based on the last one?
   * @param frameInfo Threshold for how much backwards adjustment is allowed before the walking speed is reduced
   * @param unstableWalkThreshold Threshold
   */
  void init(const WalkStepAdjustment& walkData, HipGyroBalanceState& hipBalance,
            const int hipBalanceSteps, const bool reset, const FrameInfo& frameInfo,
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
   * @param minTorsoRotation Min torso y rotation to allow the compensation
   * @param minSoleRotation Min support sole y rotation to allow the compensation
   * @param useTorsoAngle Can we trust the torso rotation and use it for the rotation error?
   * @param isLeftPhase Is the left foot the swing foot?
   * @param soleBackwardsCompensationTorsoFactor Reduce Torsorotation to prevent an overcompensation
   * @param soleForwardCompensationReturnZeroRation Reduce forward compensation after this step duration (in %)
   * @param soleBackwardsCompensationReturnZeroRatio Reduce backward compensation after this step duration (in %)
   * @param soleBackwardsCompensationFeetXDifference Interpolate the backward compensation to 100%, based on how much the feet are apart from each other
   * @param soleBackwardsCompensationFeetShift For the interpolation over the feet span
   * @param soleCompensationReduction Reduce by this value the factors for the compensations
   * @param soleCompensationIncreasement Increase by this value the factors for the compensations
   */
  void modifySwingFootRotation(const Pose3f& supportSole, const Pose3f& requestedSupportSole, Angle& swingRotation, const Angle oldSwingRotation,
                               const Angle torsoRotation, const float stepRatio, const Angle soleRotationOffsetSpeed,
                               const Angle minTorsoRotation, const Angle minSoleRotation, const bool useTorsoAngle, const bool isLeftPhase,
                               const float soleBackwardsCompensationTorsoFactor, const float soleForwardCompensationReturnZeroRation,
                               const float soleBackwardsCompensationReturnZeroRatio, const float soleBackwardsCompensationFeetXDifference,
                               const float soleBackwardsCompensationFeetShift, const float soleCompensationReduction, const float soleCompensationIncreasement);
};
