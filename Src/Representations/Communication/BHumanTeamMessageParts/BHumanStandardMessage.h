/**
 * @file BHumanStandardMessage.h
 *
 * The file declares the B-Human standard message.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Streams/AutoStreamable.h"
#include <cstdint>

#define BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER  "BHUM"
#define BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION 11      /**< This should be incremented with each change. */
#define BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 6   /**< The maximum number of players per team. */
#define BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES 7 /**< The maximum number of obstacles that can be transmitted. */

/*
 * Important remarks about units:
 *
 * For each parameter, the respective comments describe its unit.
 * The following units are used:
 *  - Distances:  Millimeters (mm)
 *  - Angles:     Radian
 *  - Time:       Milliseconds (ms)
 *  - Speed:      Millimeters per second (mm/s)
 *  - Timestamp:  Milliseconds since system / software start (ms)
 */

/*
 * Important remarks about streaming sizes:
 * NOT all values are streamed as a whole!
 * If a unit is not streamed in its natural range, a commentary will indicate
 * the range as it is interpreted.
 * The general comment pattern is "[rangeFrom..rangeTo (in precision)]".
 *
 * E.g.
 *  uint32_t value0;    //-- no comment     (This will be streamed as full 4 Byte)
 *  bool     value1;    //-- no comment     (This may be streamed as 1 Bit)
 *  uint32_t value2;    // < [3..12]        (This will be streamed with minumum value
 *                                                                and maximum value 12)
 *  uint16_t value3;    // < [3..12] bla    (Same as above)
 *  uint32_t value2;    // < [4..10 (2)]    (This will be streamed with a precision of 2, a minumum
 *                                            value of 4 and a maximum value of 10.
 *                                            This means after streaming it can hold 2,4,6,8 and 10)
 *  uint32_t time1      // < [delta 0..-10] (This will be streamed in relation to the timestamp of
 *                                            the message in range of 0 to -10.)
 */

/** The definintion of an NTP message we send - in response to a previously received request. */
STREAMABLE(BNTPMessage,
{,
  (uint32_t) requestOrigination,  /**<                        The timestamp of the generation of the request. */
  (uint32_t) requestReceipt,      /**< [delta 0..-4095]       The timestamp of the receipt of the request. */
  (uint8_t) receiver,             /**< [#_MAX_NUM_OF_PLAYERS] The robot to which this message should be sent. */
});

STREAMABLE(BHumanStandardMessage,
{
  /**
   * Returns the size of this struct when it is written.
   * @return The size of ...
   */
  int sizeOfBHumanMessage() const;

  /**
   * Converts this struct for communication usage.
   * @param data Pointer to dataspace,
   *        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
   * -asserts: writing sizeOfBHumanMessage() bytes
   */
  void write(void* data) const;

  /**
   * Reads the message from data.
   * @param data The message.
   * @return Whether the header and the versions are convertible.
   */
  bool read(const void* data);

  /** Constructor. */
  BHumanStandardMessage(),

  (char[4])  header,      /**< BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER */
  (uint8_t)  version,     /**< BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION */
  (uint8_t)  magicNumber, /**< The magic number. */
  (unsigned) timestamp,   /**< Timestamp when this message has been sent (relative to the clock frame of the sending robot). */

  (bool)     isPenalized,             /**< The name says it all. */
  (bool)     isUpright,               /**< The name says it all. */
  (bool)     hasGroundContact,        /**< The name says it all. */
  (unsigned) timeOfLastGroundContact, /**< [delta 0..-16320 (64)] The name says it all. */

  (float)                robotPoseValidity,   /**< [0..1 (0.0039)] The validity of the RobotPose. */
  (float)                robotPoseDeviation,  /**< The deviation of the RobotPose. */
  (std::array<float, 6>) robotPoseCovariance, /**< The covariance matrix of the RobotPose. */
  (unsigned)             timestampLastJumped, /**< [delta 0..-32640 (128)] The timestamp when the localization jumped. */

  (unsigned)             ballTimeWhenLastSeen,    /**< The name says it all. */
  (unsigned)             ballTimeWhenDisappeared, /**< The name says it all. */
  (unsigned char)        ballSeenPercentage,      /**< The name says it all */
  (Vector2f)             ballVelocity,            /**< [-32768..32767 (1)] The ball velocity .*/
  (Vector2f)             ballLastPercept,         /**< [-32768..32767 (1)] The position where the last ball percept was. */
  (std::array<float, 3>) ballCovariance,          /**< The covariance matrix of the ball position. */

  (unsigned char) confidenceOfLastWhistleDetection, /**< The name says it all. */
  (unsigned char) channelsUsedForWhistleDetection,  /**< The name says it all. */
  (unsigned) lastTimeWhistleDetected,               /**< [delta 0..-65535] The name says it all. */

  (unsigned char)                                    teamActivity,              /**< What team play the robot is doing. */
  (unsigned)                                         timeWhenReachBall,         /**< [delta 0..524280 (8)] The estimate when this robot reaches the ball. */
  (unsigned)                                         timeWhenReachBallStriker,  /**< [delta 0..524264 (8)] The estimate when this robot reaches the ball if it is striker. */
  (bool[BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS]) teammateRolesIsGoalkeeper, /**< The role assignment for the whole team. */
  (bool[BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS]) teammateRolesPlayBall,     /**< The role assignment for the whole team. */
  (int[BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS])  teammateRolesPlayerIndex,  /**< [-1..6] The role assignment for the whole team. */
  (int)                                              captain,                   /**< [-1..6] The captain that provided the teammate roles. */
  (unsigned)                                         teammateRolesTimestamp,    /**< [delta 0..-8191] The timestamp when the teammate roles have been computed. */
  (bool)                                             isGoalkeeper,              /**< Whether this robot is currently a goalkeeper. */
  (bool)                                             playBall,                  /**< Whether this robot currently plays the ball. */
  (int)                                              supporterIndex,            /**< [-1..6] The index of this robot in the supporter set. */

  (unsigned char) activity,   /**< What the robot is doing in general. */
  (int)           passTarget, /**< [-1..14] Which teammate this robot wants to pass to (or -1). */
  (Vector2f)      walkingTo,  /**< [-32768..32767 (1)] Where the robot wants to walk. */
  (Vector2f)      shootingTo, /**< [-32768..32767 (1)] Where the robot wants to kick the ball. */

  /**
   * Obstacle has the attributes covariance, center, left, right, velocity, lastSeen and type.
   * covariance is streamed as float with the nondiagonal entries as one value.
   * center is streamed in [-32768..32767 (1)].
   * left is streamed in [-32768..32764 (4)].
   * right is streamed in [-32768..32764 (4)].
   * velocity is not streamed at all.
   * lastSeen is streamed in [delta 0..-16320 (64)].
   * type is streamed in [0..255].
   */
  (std::vector<Obstacle>) obstacles,

  (char) say,
  (unsigned int) nextTeamTalk,

  (bool) requestsNTPMessage,              /**< Whether this robot requests NTP replies from the others. */
  (std::vector<BNTPMessage>) ntpMessages, /**< The NTP replies of this robot to other robots. */
});
