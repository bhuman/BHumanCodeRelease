/**
 * @file TeammateData.h
 * Declaration of a class representing information about the teammates.
 * @author Colin Graf
 */

#pragma once

#include <stdint.h>
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/GoalPercept.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Modeling/ObstacleClusters.h"

/**
 * @class TeammateData
 * A class representing information about the teammates.
 */
STREAMABLE(TeammateData,
{
public:
  ENUM(Player,
    coach,
    firstPlayer,
    player1 = firstPlayer,
    player2,
    player3,
    player4,
    player5,
    player6
  );

  STREAMABLE(DropInData,
  {
    ,
    (bool)(false) keeperAvailable,
    (unsigned)(0) keeperNumber,
  });

  /** Players are B-Human players by default. */
  TeammateData();

  /** drawing function for representation*/
  void draw() const;

  static unsigned getIntentionForRole(Role::RoleType role),

  (unsigned)(0) currentTimestamp,
  (unsigned)(4000) networkTimeout, /**< The time without packets received after which a teammate is considered absent. */
  (unsigned)(0) numOfConnectedTeammates, /**< The number of robots of which messages were received recently. _Not_ including this robot itself. */
  (unsigned)(0) numOfActiveTeammates, /**< numOfConnectedTeammates minus all robots that are currently penalized. */
  (unsigned)(0) numOfFullyActiveTeammates, /**< numOfActiveTeammates minus all robots that are currently not upright or do not have ground contact */
  (unsigned)(numOfPlayers) firstTeammate, /**< player number of first team mate */
  (bool)(false) sendThisFrame, /**< The team communication will be sent in this frame. */
  (bool)(false) wasConnected, /**< Whether we have been connected to a team mate. */
  (unsigned[numOfPlayers]) timeStamps, /**< The times when messages from different robots arrived. */
  (bool[numOfPlayers]) isActive, /**< true, if messages from the respective team mate were recently received and the team mate is currently not penalized. */
  (bool[numOfPlayers]) isFullyActive, /**< true, if 'isActive' and has not been fallen down and has ground contact */
  (BallModel[numOfPlayers]) ballModels, /**< The last received ball model of each team mate. */
  (ObstacleModel[numOfPlayers]) obstacleModels, /**< The last received obstacle model of each team mate. */
  (RobotPose[numOfPlayers]) robotPoses, /**< The last received robot pose of each team mate. */
  (SideConfidence[numOfPlayers]) robotsSideConfidence, /**< The last received sideConfidence of each team mate. */
  (BehaviorStatus[numOfPlayers]) behaviorStatus, /**< The last received behavior status of each team mate. */
  (bool[numOfPlayers]) isBHumanPlayer, /**< Tells us if a teammate is a B-Human player. */
  (bool[numOfPlayers]) isPenalized, /**< Tells us if a teammate is penalized. */
  (bool[numOfPlayers]) hasGroundContact, /**< Tells us if a teammate has ground contact. */
  (bool[numOfPlayers]) isUpright, /**< Tells us if a teammate is fallen down. */
  (FieldCoverage::GridInterval[numOfPlayers]) fieldCoverages, /**< The last received field coverage grid of each team mate. */
  (ObstacleClusters[numOfPlayers]) obstacleClusters, /**< The last received obstacle clusters */
  (unsigned[numOfPlayers]) timeLastGroundContact, /**< The time since last ground contact of a team mate. */
  (float[numOfPlayers]) cameraHeights, /**< camera heights of team mates */
  (Vector2<>[numOfPlayers]) walkingTo, /**< the robot's target position on the field in the field coordinate system.
                                       If the robot does not have any target, this attribute should be set to the robot's position. */
  (Vector2<>[numOfPlayers]) shootingTo, /**< the target position of the next shot (either pass or goal shot) in the field coordinate system.
                                        If the robot does not intend to shoot, this attribute should be set to the robot's position. */
  (uint16_t[numOfPlayers]) intention, /**< DROPIN_INTENTION_DEFAULT 0, DROPIN_INTENTION_KEEPER 1, DROPIN_INTENTION_DEFENSIVE 2,
                                      DROPIN_INTENTION_KICK 3, DROPIN_INTENTION_LOST 4 */
  (DropInData) dropInData,
});

/**
 * @class TeamDataSenderOutput
 * An empty dummy representation for the TeamDataSender module
 */
STREAMABLE(TeamDataSenderOutput,
{,
});

/**
* @class RobotPoseCompressed
* A compressed version of RobotPose used in team communication
*/
STREAMABLE(TeammateDataCompressed,
{
public:
  TeammateDataCompressed() = default;
  TeammateDataCompressed(const TeammateData& teammateData);
  operator TeammateData() const,

  (unsigned)(0) currentTimestamp,
  (unsigned)(TeammateData::numOfPlayers) firstTeammate,
  (unsigned[TeammateData::numOfPlayers]) timeStamps,
  (bool[TeammateData::numOfPlayers]) isActive,
  (bool[TeammateData::numOfPlayers]) isFullyActive,
  (BallModelCompressed[TeammateData::numOfPlayers]) ballModels,
  (RobotPoseCompressed[TeammateData::numOfPlayers]) robotPoses,
  (SideConfidence[TeammateData::numOfPlayers]) robotsSideConfidence,
  (Role, RoleType[TeammateData::numOfPlayers]) roles,
  (Vector2<>[TeammateData::numOfPlayers]) walkingTo,
  (uint16_t[TeammateData::numOfPlayers]) intention, /**< DROPIN_INTENTION_DEFAULT 0, DROPIN_INTENTION_KEEPER 1, DROPIN_INTENTION_DEFENSIVE 2,
                                      DROPIN_INTENTION_KICK 3, DROPIN_INTENTION_LOST 4 */
  (TeammateData::DropInData) dropInData,
});
