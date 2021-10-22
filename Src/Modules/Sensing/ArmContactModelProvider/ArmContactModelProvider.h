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

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Streams/EnumIndexedArray.h"

MODULE(ArmContactModelProvider,
{,
  REQUIRES(ArmMotionRequest),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(RobotPose),
  REQUIRES(DamageConfigurationBody),
  USES(ArmMotionInfo),
  USES(JointRequest),
  USES(MotionInfo),
  USES(OdometryData),
  PROVIDES(ArmContactModel),
  DEFINES_PARAMETERS(
  {,
    (int)(4) frameDelay, //Delay with which the hand positions shpould be accessed
    (unsigned int)(4) minimalContactDuration,
    (float)(13.f) yErrorThresholdBase, //Minimal yErrorThreshold. The y threshold base cant never be less than this!
    (float)(11.f) xErrorThresholdBase, //Minimal xErrorThreshold. The x threshold base cant never be less than this!
    (float)(3.f) yErrorThresholdExtension, //The y threshold extension which is added when the robot is moving in -y or y direction, this means left or right.
    (float)(3.f) xErrorThresholdExtension, //The x threshold extension which is added when the robot is moving in -x or x direction, this means back or forward.
    (int)(2000) soundWaitTime, //Play the arm sound only once in this time frame.
  }),
});

/**
 * @class ArmContactModelProvider
 *
 */
class ArmContactModelProvider: public ArmContactModelProviderBase
{
  /**
   * Buffer to store multiple hand positions.
   * This is necessary to eliminate flatten the handspeed with flattenSpeed()
   */
  static constexpr int frameBufferSize = 5;
  static constexpr int errorBufferSize = 100;

  //Timestamp when the arm sound was played last.
  unsigned int lastArmSoundTimestamp = 0;

  //Buffer of the requested hand positions
  RingBuffer<Vector3f, frameBufferSize> angleBuffer[2];

  //Resets the wholeRepresentation (ArmContactModel) when the robot is in a state in which this Module shouldnt regularly run.
  void reset(ArmContactModel& model);

  //Map of the push directions and wether there is a push in this direction right now or not
  ENUM_INDEXED_ARRAY(bool, ArmContactModel::PushDirection) directionMap;

  //Compares the dimmed error Value with the thresholds and decides if there is a push in a certain direction
  void determinePushDirection(ArmContactModel& model);

  //The buffer for the UNdimmed errors calculated by the subtracting the actual and requested hand position
  RingBufferWithSum<Vector2f, errorBufferSize> errorBuffer[2];

  void update(ArmContactModel& model);

  //Building a RobotModel with the actual joint angles the robot is sensing right now with the limbs angle encoders and returning the hand position. German: Ist-Wert der Arme.
  Vector3f calculateActualHandPosition(bool left);

  //Building a RobotModel with the requested joint angles. Those are the positions which the robot desires to reach, not were they are right now! Returns the requested hand position. German: Soll-Wert
  Vector3f calculateRequestedHandPosition(bool left);

  //Calculates the difference between actual and requested hand position. Fills the errorBuffer with UNdimmed error values.
  void calculateForce();

  //Was the keyFrameMotion to put the arms on the back active last frame? 0 is left 1 is right arm.
  bool armOnBackLastFrame[2];

  //How many frames ago the robot started to ove his hands on the back or from the back to there "normal" positions? 0 is left 1 is right.
  int adjustingHandCounter[2];

  /**
   * Is the robot currently adjusting his hands to his back or not?
   * When this is true, the errors are ignored in this frame because the arms move to fast during this phase and would cause false positives for arm contacts.
   */
  bool adjustingHand[2];

  //The dimmed errors sensed right now. Calculated by multiplying the errorBuffer values with the speed Factor
  Vector2f dimmedError[2];

  /**
   * Offsets added to the undimmed error.
   * These are useful because the robots angles are usually a little bit lose and fall into certain direction predictably in certains statuses.
   */
  Vector2f yOffsets;

  /**
   * Offsets added to the undimmed error.
   * These are useful because the robots angles are usually a little bit lose and fall into certain direction predictably in certains statuses.
   */
  Vector2f xOffsets;

  /**
   * Current speed of the REQUESTED positions, not the actual positions. used to calculate the speed Factor.
   * This is useful, because when the arms should move move quite fast, there is a natural error because of the control latency.
   * So when the speed is high the error should be reduced to enable minimal thresholds.
   */
  Vector2f handSpeed[2];

  //Storage for the requested hand positions
  Vector2f handPosRequested[2];

  //Storage of the hand Position last frame, necessary to calculate the speed of the hand positions;
  Vector2f lastHandPosRequested[2];

  //The factor to dim the errors regarding the requested speed of the arms.
  Vector2f speedFactor;

  //Time stamp when the ArmContactModelProvider was last inaktive.
  unsigned int lastInActiveTimestamp = 0;

  /**
   * Range [0,1]. represents how much of the Threshold extension should be added to the base values depending on the walk direction of the robot.
   * For example, when the robot is moving forward, the y error threshold should be minimal,
   * because the speed in this direction is very low and the errors therefore very precise
   * If the robot is moving sideways the factor for the threshold extension is nearly 0, because the speed in the x direction is very low.
   */
  Vector2f directionFactor;
  void updateError(ArmContactModel& model);

  //Calculates the speedFactor regarding the speed of the requested hand Positions
  void calcCorrectionFactor(bool left);

  //Calculates the y Offsets
  Vector2f calcYOffsets(bool armOnBack);

  //Calculates the xOffsets
  Vector2f calcXOffsets(bool armOnBack);

  //Calculates the average requested position of the arms of the last five frames to eliminate stray.
  Vector2f flattenSpeed(RingBuffer<Vector3f, frameBufferSize>& angleBuffer);

  //Decides wether the robot is currently moving his arms to the back or away. During this time the errors should be ignored.
  void decideArmMovementDone(ArmContactModel& model);

  //Calculates the walkDirection factor with the information from theMotionInfo.speed.
  void calcWalkDirectionFactor();
public:
  ArmContactModelProvider() : errorBuffer{ RingBufferWithSum<Vector2f, errorBufferSize>(Vector2f::Zero()), RingBufferWithSum<Vector2f, errorBufferSize>(Vector2f::Zero()) } {};
};
