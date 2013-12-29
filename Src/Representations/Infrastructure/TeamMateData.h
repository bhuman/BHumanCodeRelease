/**
 * @file TeamMateData.h
 * Declaration of a class representing information about the teammates.
 * @author Colin Graf
 */

#pragma once

#include "Representations/Modeling/RobotsModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Modeling/ObstacleClusters.h"

/**
 * @class TeamMateData
 * A class representing information about the teammates.
 */
STREAMABLE(TeamMateData,
{
public:
  ENUM(Player,
    noPlayer,
    firstPlayer,
    player1 = firstPlayer,
    player2,
    player3,
    player4,
    player5
  );

  /** drawing function for representation*/
  void draw() const,

  (unsigned int)(0) currentTimestamp,
  (unsigned int)(4000) networkTimeout, /**< The time without packets received after which a teammate is considered absent. */
  (unsigned)(0) numOfConnectedTeamMates, /**< The number of robots of which messages were received recently. _Not_ including this robot itself. */
  (unsigned)(0) numOfActiveTeamMates, /**< numOfConnectedTeamMates minus all robots that are currently penalized. */
  (unsigned)(0) numOfFullyActiveTeamMates, /**< numOfActiveTeamMates minus all robots that are currently not upright or do not have ground contact */
  (unsigned)(numOfPlayers) firstTeamMate, /**< player number of first team mate */
  (bool)(false) sendThisFrame, /**< The team communication will be sent in this frame. */
  (bool)(false) wasConnected, /**< Whether we have been connected to a team mate. */
  (unsigned int[numOfPlayers]) timeStamps, /**< The times when messages from different robots arrived. */
  (bool[numOfPlayers]) isActive, /**< true, if messages from the respective team mate were recently received and the team mate is currently not penalized. */
  (bool[numOfPlayers]) isFullyActive, /**< true, if 'isActive' and has not been fallen down and has ground contact */
  (BallModel[numOfPlayers]) ballModels, /**< The last received ball model of each team mate. */
  (ObstacleModel[numOfPlayers]) obstacleModels, /**< The last received obstacle model of each team mate. */
  (RobotsModel[numOfPlayers]) robotsModels, /**< The last received robots model of each team mate. */
  (RobotPose[numOfPlayers]) robotPoses, /**< The last received robot pose of each team mate. */
  (SideConfidence[numOfPlayers]) robotsSideConfidence, /**< The last received sideConfidence of each team mate. */
  (BehaviorStatus[numOfPlayers]) behaviorStatus, /**< The last received behavior status of each team mate. */
  (bool[numOfPlayers]) isPenalized, /**< Tells us if a teammate is penalized. */
  (bool[numOfPlayers]) hasGroundContact, /**< Tells us if a teammate has ground contact. */
  (bool[numOfPlayers]) isUpright, /**< Tells us if a teammate is fallen down. */
  (TeamHeadControlState[numOfPlayers]) teamHeadControlStates,
  (FieldCoverage::GridInterval[numOfPlayers]) fieldCoverages, /**< The last received field coverage grid of each team mate. */
  (ObstacleClusters[numOfPlayers]) obstacleClusters, /**< The last received obstacle clusters */
  (unsigned int[numOfPlayers]) timeLastGroundContact, /**< The time since last ground contact of a team mate. */
  (float[numOfPlayers]) cameraHeights, /**<camera heights of team mates> */

  // Initialization
  for(int i = 0; i < numOfPlayers; ++i)
  {
    timeStamps[i] = 0;
    timeLastGroundContact[i] = 0;
  }
});

/**
 * @class TeamDataSenderOutput
 * An empty dummy representation for the TeamDataSender module
 */
STREAMABLE(TeamDataSenderOutput,
{,
});

