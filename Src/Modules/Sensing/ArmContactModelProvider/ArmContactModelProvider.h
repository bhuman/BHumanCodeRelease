/**
 * @file ArmContactModelProvider.h
 *
 * This module detects whether the robot touches a potential obstacle with an arm.
 * This is done with comparing the current arm position and the position where the arm should be.
 * The difference of those values is then stored into a buffer for each arm and if that buffer represents
 * an error large enough, a positive arm contact for that arm is reported.
 *
 * If the robot fell down, all error buffers are resetted. Additionally, arm contact may only be reported
 * if the robot is standing or walking and not penalized.
 *
 * Declaration of class ArmContactModelProvider.
 * @author <a href="mailto:fynn@informatik.uni-bremen.de">Fynn Feldpausch</a>
 * @author <a href="mailto:simont@informatik.uni-bremen.de">Simon Taddiken</a>
 * @author <a href="mailto:arneboe@informatik.uni-bremen.de">Arne Böckmann</a>
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/RingBufferWithSum.h"

/** maximum numbers of frames to buffer */
#define FRAME_BUFFER_SIZE 5

/** number of angle differences to buffer */
#define ERROR_BUFFER_SIZE 100

MODULE(ArmContactModelProvider,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  REQUIRES(RobotInfo),
  REQUIRES(RobotModel),
  REQUIRES(DamageConfigurationBody),
  USES(ArmMotionInfo),
  USES(ArmMotionSelection),
  USES(JointRequest),
  USES(MotionInfo),
  USES(OdometryData),
  PROVIDES(ArmContactModel),
  LOADS_PARAMETERS(
  {,
    (Angle) errorXThreshold,           /**< Maximum divergence of arm angleX (in degrees) that is not treated as an obstacle detection */
    (Angle) errorYThreshold,           /**< Maximum divergence of arm angleY (in degrees) that is not treated as an obstacle detection */
    (unsigned) malfunctionThreshold,   /**< Duration of contact in frames after a contact is ignored */
    (unsigned) frameDelay,             /**< The size of the delay in frames */
    (bool) debugMode,                  /**< Enable debug mode */
    (float) speedBasedErrorReduction,  /**< At this translational hand speed, the angular error will be ignored (in mm/s). */
    (int) waitAfterMovement,           /**< wait this amount of time (in ms) after ARME moved an arm before contact can be detected again */
    (bool) detectWhilePenalized,
  }),
});

/**
 * @class ArmContactModelProvider
 *
 */
class ArmContactModelProvider: public ArmContactModelProviderBase
{
  struct ArmAngles
  {
    float leftX,  /**< X angle of left arm */
          leftY,  /**< Y angle of left arm */
          rightX, /**< X angle of right arm */
          rightY; /**< Y angle of right arm */

    ArmAngles() = default;
    ArmAngles(float leftX, float leftY, float rightX, float rightY) : leftX(leftX), leftY(leftY), rightX(rightX), rightY(rightY) {};
  };

  RingBuffer<ArmAngles, FRAME_BUFFER_SIZE> angleBuffer; /**< Buffered arm angles to eliminate delay */
  /**
   * Error vector:
   * y-Component: error of shoulder roll
   * x-Component: error of shoulder pitch
   */
  RingBufferWithSum<Vector2f, ERROR_BUFFER_SIZE> errorBuffer[2];     /**< Buffered error over ERROR_BUFFER_SIZE frames */
  bool movingLastFrame[2] = {false, false};                          /**< Whether left (0) and right (1) arm was moving last frame */
  unsigned lastMovement[2] = {0, 0};                                 /**< time of last left (0) and right (1) arm movement */
  Vector2f lastHandPos [2] = { Vector2f::Zero(), Vector2f::Zero() }; /**< Last 2-D position of the left (0) and right (1) hand */
  Pose2f lastOdometry;                                               /**< Odometry inthe previous frame. */
  const int soundDelay = 1000;                                       /**< Length of debug sound */
  unsigned int lastSoundTime = 0;                                    /**< Time of last debug sound */
  int lastGameState = STATE_INITIAL;                                /**< Game state in previous frame. */

  /**
   * Resets the arm contact model to default values and clears the error buffers.
   * @param armContactModel The model to reset.
   */
  void resetAll(ArmContactModel& armContactModel);

  /** Fills the error buffers with differences of current and requested
   * joint angles
   * @param left Check the left or the right arm for collisions?
   * @param factor A factor used to reduce the error entered into the buffer.
   */
  void checkArm(bool left, float factor);

  /** Executes this module.
   * @param ArmContactModel The data structure that is filled by this module.
   */
  void update(ArmContactModel& armContactModel) override;

  /**
   * Calculates the angular speed of the robots hands.
   * @param foreArm The arm which hand should be checked.
   * @param odometryOffset current odometry offset.
   * @param The speed of the hand calculated in the previous frame. This will be updated at the end of this method.
   */
  float calculateCorrectionFactor(const Pose3f& foreArm, const Pose2f& odometryOffset, Vector2f& lastArmPos);

  /** Dertemines the push direction for an arm. That is, the direction in which the specified arm is being
   * pushed. That means that the obstacle causing the contact is located on the opposite direction of
   * the push direction.
   * The directions are given as compass directions with NORTH being the direction in which the robot
   * currently looks.
   *
   * @param left Check the left arm (true) or the right (false)
   * @param Is error.x above the x-axis threshold?
   * @param Is error.y above the y-axis threshold?
   * @param Current error vector.
   * @return The direction in which the specified arm is being pushed.
   */
  ArmContactModel::PushDirection getDirection(bool left, bool contactX, bool contactY, Vector2f error);

  void prepare(ArmContactModel& model);

public:
  ArmContactModelProvider() : errorBuffer{ RingBufferWithSum<Vector2f, ERROR_BUFFER_SIZE>(Vector2f::Zero()), RingBufferWithSum<Vector2f, ERROR_BUFFER_SIZE>(Vector2f::Zero()) } {};
};
