/**
 * @file TeammateData.h
 *
 * Representation of information received from my teammates
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include <cstdint>
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/Whistle.h"

/**
 * @struct Teammate
 * Description of all information about/from a teammate
 */
STREAMABLE(Teammate,
{
  ENUM(Status,
  {,
    INACTIVE,                              /** OK   : I receive packets, but robot is penalized */
    ACTIVE,                                /** GOOD : OK + Teammate is not penalized */
    FULLY_ACTIVE,                          /** BEST : GOOD + Teammate is standing/walking and has ground contact :-) */
  }),

  (int)(-1) number,                                   /**< The number of this player */
  (bool)(false) isGoalkeeper,                         /**< The name says it all */
  (bool)(true) isBHumanPlayer,                        /**< The name says it all */
  (bool)(true) isPenalized,                           /**< The name says it all */
  (bool)(true) isUpright,                             /**< The name says it all */
  (unsigned)(0) timeWhenLastPacketReceived,           /**< The name says it all */
  (unsigned)(0) timeOfLastGroundContact,              /**< The name says it all */
  (bool)(true) hasGroundContact,                      /**< The name says it all */
  (Status)(INACTIVE) status,                          /**< The name says it all */
  (RobotPose) pose,                                   /**< The pose in global field coordinates */
  (BallModel) ball,                                   /**< Model of the ball (in coordinates relative to my teammate's pose) */
  (ObstacleModelCompressed) obstacleModel,            /**< Model of obstacles (in coordinates relative to my teammate's pose) */
  (SideConfidence) sideConfidence,                    /**< The belief about playing in the right direction */
  (BehaviorStatus) behaviorStatus,                    /**< Information about the behavior */
  (SPLStandardBehaviorStatus) standardBehaviorStatus, /**< Behvaior information as specified by SPLStandardMessage */
  (Whistle) whistle,                                  /**< Output of the WhistleRecognizer */
  (TeammateRoles) teammateRoles,                      /**< The Roles the teammates should use */
});

/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(TeammateData,
{
  /** Drawing function for representation */
  void draw() const,

  (std::vector<Teammate>) teammates,        /**< An unordered(!) list of all teammates that are currently communicating with me */
  (int)(0) numberOfActiveTeammates,         /**< The number of teammates (in the list) that are at not INACTIVE */
  (bool)(false) sendThisFrame,              /**< The team communication will be sent in this frame. TODO: Find a better place!*/
});

/**
 * @struct TeamDataSenderOutput
 * An empty dummy representation for the TeamDataSender module
 */
STREAMABLE(TeamDataSenderOutput,
{,
});
