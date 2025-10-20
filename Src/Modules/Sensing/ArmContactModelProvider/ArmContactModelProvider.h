/**
 * @file ArmContactModelProvider.h
 *
 * This module detects whether the robot touches a potential obstacle with an arm, even while the arm is put on the
 * back of the robot during challenges or other situations.
 * This is done by comparing the current arm position and the position where the arm should be.
 * The difference of those values is then stored into a buffer for each arm and if that buffer represents
 * an error large enough, a positive arm contact for that arm is reported.
 *
 * If the robot fell down, all error buffers are reset. Additionally, arm contact may only be reported
 * if the robot is standing or walking and not penalized.
 *
 * Declaration of class ArmContactModelProvider.
 * @author <a href="mailto:lrust@uni-bremen.de">Lukas Rust</a>
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Math/Pose3f.h"
#include "Framework/Module.h"
#include "Math/RingBuffer.h"
#include "Math/RingBufferWithSum.h"
#include "Streaming/EnumIndexedArray.h"

MODULE(ArmContactModelProvider,
{,
  REQUIRES(ArmMotionRequest),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  USES(JointRequest),
  USES(MotionInfo),
  PROVIDES(ArmContactModel),
  LOADS_PARAMETERS(
  {,
    (int) frameBufferSize, /**< Buffer of the requested angles. */
    (int) errorBufferSize, /**< Buffer of the errors. */
    (unsigned int) minimalContactDuration,
    (float) yErrorThresholdBase, /**< Minimal yErrorThreshold. The y threshold base cant never be less than this! */
    (float) xErrorThresholdBase, /**< Minimal xErrorThreshold. The x threshold base cant never be less than this! */
    (float) yErrorThresholdExtension, /**< The y threshold extension which is added when the robot is moving in -y or y direction, this means left or right. */
    (float) xErrorThresholdExtension, /**< The x threshold extension which is added when the robot is moving in -x or x direction, this means back or forward. */
    (int) soundWaitTime, /**< Play the arm sound only once in this time frame. */
  }),
});

/**
 * @class ArmContactModelProvider
 *
 */
class ArmContactModelProvider: public ArmContactModelProviderBase
{
  //Timestamp when the arm sound was played last.
  unsigned int lastArmSoundTimestamp = 0;

  RobotModel requestedRobotModel;

  //Buffer of the requested hand positions
  RingBuffer<Vector3f> angleBuffer[Arms::numOfArms];

  //The buffer for the UNdimmed errors calculated by the subtracting the actual and requested hand position
  RingBufferWithSum<Vector2f> errorBuffer[Arms::numOfArms];

  //Was the keyFrameMotion to put the arms on the back active last frame? 0 is left 1 is right arm.
  bool armOnBackLastFrame[Arms::numOfArms] = {false, false};

  //How many frames ago the robot started to move his hands on the back or from the back to there "normal" positions? 0 is left 1 is right.
  int adjustingHandCounter[Arms::numOfArms] = {0, 0};

  /**
   * Is the robot currently adjusting his hands to his back or not?
   * When this is true, the errors are ignored in this frame because the arms move to fast during this phase and would cause false positives for arm contacts.
   */
  bool adjustingHand[Arms::numOfArms] = {false, false};

  //The dimmed errors sensed right now. Calculated by multiplying the errorBuffer values with the speed Factor
  Vector2f dimmedError[Arms::numOfArms];

  /**
   * Current speed of the REQUESTED positions, not the actual positions. used to calculate the speed Factor.
   * This is useful, because when the arms should move move quite fast, there is a natural error because of the control latency.
   * So when the speed is high the error should be reduced to enable minimal thresholds.
   */
  Vector2f handSpeed[Arms::numOfArms];

  //Storage for the requested hand positions
  Vector2f handPosRequested[Arms::numOfArms];

  //Storage of the hand Position last frame, necessary to calculate the speed of the hand positions;
  Vector2f lastHandPosRequested[Arms::numOfArms];

  //The factor to dim the errors regarding the requested speed of the arms.
  Vector2f speedFactor;

  //Building a RobotModel with the actual joint angles the robot is sensing right now with the limbs angle encoders and returning the hand position.
  Vector3f calculateActualHandPosition(Arms::Arm arm) const;

  //Decides whether the robot is currently moving his arms to the back or away. During this time the errors should be ignored.
  void decideArmMovementDone(ArmContactModel& model);

  //Calculates the xOffsets
  /**
   * Offsets added to the undimmed error.
   * These are useful because the robots angles are usually a little bit lose and fall into certain direction predictably in certain statuses.
   */
  float calcXOffset(bool armOnBack) const;

  //Calculates the y Offsets
  /**
   * Offsets added to the undimmed error.
   * These are useful because the robots angles are usually a little bit lose and fall into certain direction predictably in certain statuses.
   */
  float calcYOffset(Arms::Arm arm, bool armOnBack) const;

  //Compares the dimmed error Value with the thresholds and decides if there is a push in a certain direction
  void determinePushDirection(ArmContactModel& model);

  //Calculates the difference between actual and requested hand position. Fills the errorBuffer with UNdimmed error values.
  void calculateForce();

  //Building a RobotModel with the requested joint angles. Those are the positions which the robot desires to reach, not were they are right now! Returns the requested hand position.
  Vector3f calculateRequestedHandPosition(Arms::Arm arm) const;

  void update(ArmContactModel& model);

  void updateError(ArmContactModel& model);

  //Resets the wholeRepresentation (ArmContactModel) when the robot is in a state in which this Module should not regularly run.
  void reset(ArmContactModel& model);

  //Calculates the speedFactor regarding the speed of the requested hand Positions
  void calcCorrectionFactor(Arms::Arm arm);

  //Calculates the average requested position of the arms of the last five frames to eliminate stray.
  Vector2f flattenSpeed(const RingBuffer<Vector3f>& angleBuffer) const;

  //Calculates the walkDirection factor with the information from theMotionInfo.speed.
  /**
   * Range [0,1]. represents how much of the Threshold extension should be added to the base values depending on the walk direction of the robot.
   * For example, when the robot is moving forward, the y error threshold should be minimal,
   * because the speed in this direction is very low and the errors therefore very precise
   * If the robot is moving sideways the factor for the threshold extension is nearly 0, because the speed in the x direction is very low.
   */
  Vector2f calcWalkDirectionFactor() const;
public:
  ArmContactModelProvider();
};
