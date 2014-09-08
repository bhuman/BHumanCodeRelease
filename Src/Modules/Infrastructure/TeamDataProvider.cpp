/**
 * @file TeamDataProvider.cpp
 * This file implements a module that provides the data received by team communication.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstdint>
#include <limits>
#include "TeamDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"
#include "Tools/Math/Transformation.h"
#include <array>

MAKE_MODULE(TeamDataProvider, Cognition Infrastructure)

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * teammateData.array[robotNumber].
 */
#define UNPACK(representation, array) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  theTeammateData.array[robotNumber] = the##representation##Compressed;

/**
 * This macro converts a timeStamp into local time via ntp.
 */
#define REMOTE_TO_LOCAL_TIME(timeStamp, robotNumber) \
  if(theTeammateData.isBHumanPlayer[robotNumber]) \
  { \
    if(timeStamp) \
      timeStamp = ntp.getRemoteTimeInLocalTime(timeStamp); \
  }

PROCESS_WIDE_STORAGE(TeamDataProvider) TeamDataProvider::theInstance = 0;

TeamDataProvider::TeamDataProvider() :
  timeStamp(0), robotNumber(-1), lastSentTimeStamp(0)
{
  theInstance = this;
}

TeamDataProvider::~TeamDataProvider()
{
  theInstance = 0;
}

void TeamDataProvider::update(TeammateData& teammateData)
{
  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[5].bestOffset);

  teammateData = theTeammateData;

  fillOwnData(teammateData);

  teammateData.currentTimestamp = theFrameInfo.time;
  teammateData.numOfConnectedTeammates = 0;
  teammateData.numOfActiveTeammates = 0;
  teammateData.numOfFullyActiveTeammates = 0;
  teammateData.firstTeammate = TeammateData::numOfPlayers;
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
  {
    if(i == theRobotInfo.number)
    {
      continue;
    }
    teammateData.isActive[i] = false;
    teammateData.isFullyActive[i] = false;
    // Check if network connection is working (-> connected):
    if(teammateData.timeStamps[i] && theFrameInfo.getTimeSince(teammateData.timeStamps[i]) < static_cast<int>(teammateData.networkTimeout))
    {
      teammateData.numOfConnectedTeammates++;
      // Check if teammate is not penalized (-> active):
      if(!teammateData.isPenalized[i] && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE)
      {
        teammateData.numOfActiveTeammates++;
        teammateData.isActive[i] = true;
        if(i < static_cast<int>(teammateData.firstTeammate))
          teammateData.firstTeammate = i;
        // Check if teammate has not been fallen down or lost ground contact (-> fully active):
        if(teammateData.hasGroundContact[i] && teammateData.isUpright[i])
        {
          teammateData.numOfFullyActiveTeammates++;
          teammateData.isFullyActive[i] = true;
        }
      }
    }
  }
  if(teammateData.numOfConnectedTeammates)
    teammateData.wasConnected = theTeammateData.wasConnected = true;
  teammateData.sendThisFrame =
#ifdef TARGET_ROBOT
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
#endif
    theFrameInfo.getTimeSince(lastSentTimeStamp) >= sendInterval;
  if(teammateData.sendThisFrame)
  {
    // Check if NTP has to respond
    ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut());
    if(theFrameInfo.getTimeSince(lastSentTimeStamp) >= 2 * sendInterval)
      lastSentTimeStamp = theFrameInfo.time;
    else
      lastSentTimeStamp += sendInterval;
  }

  // calculate time to reach ball for teammates
  if(Global::getSettings().isDropInGame)
  {
    for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
    {
      if(i == theRobotInfo.number)
      {
        if(teammateData.behaviorStatus[i].role == Role::striker)
          teammateData.behaviorStatus[i].estimatedTimeToReachBall = 0.f;
        else
          teammateData.behaviorStatus[i].estimatedTimeToReachBall = 1000.f;
      }
      else
      {
        // maybe do that only if the mesage was recceived this frame.
        teammateData.behaviorStatus[i].estimatedTimeToReachBall = estimatedDropInTimeToReachBall(teammateData, i);
      }
    }
    setRolesForTeammates(teammateData);
  }

  theTeammateData = teammateData;
}

void TeamDataProvider::fillOwnData(TeammateData& teammateData)
{
  int ownNumber = theRobotInfo.number;
  teammateData.timeStamps[ownNumber] = theFrameInfo.time;
  teammateData.isActive[ownNumber] = true;
  teammateData.isFullyActive[ownNumber] = theGroundContactState.contact && theFallDownState.state == theFallDownState.upright;
  teammateData.ballModels[ownNumber] = theBallModel;
  teammateData.robotPoses[ownNumber] = theRobotPose;
  teammateData.robotsSideConfidence[ownNumber] = theSideConfidence;
  //teammateData.behaviorStatus[ownNumber] = theBe;
  teammateData.isBHumanPlayer[ownNumber] = true;
  teammateData.isPenalized[ownNumber] = theRobotInfo.penalty != PENALTY_NONE;
  teammateData.hasGroundContact[ownNumber] = theGroundContactState.contact;
  teammateData.isUpright[ownNumber] = theFallDownState.state == theFallDownState.upright;
  teammateData.timeLastGroundContact[ownNumber] = theFrameInfo.time;
  teammateData.cameraHeights[ownNumber] = theCameraMatrix.translation.z;
  teammateData.walkingTo[ownNumber] = theRobotPose.translation;
  //teammateData.shootingTo[ownNumber] = ;
}

void TeamDataProvider::handleMessages(MessageQueue& teamReceiver)
{
  if(theInstance)
    teamReceiver.handleAllMessages(*theInstance);

  teamReceiver.clear();
}

bool TeamDataProvider::handleMessage(InMessage& message)
{
  // The robotNumber and the three flags hasGroundContact, isUpright and isPenalized should always be updated.
  switch(message.getMessageID())
  {
    case idNTPHeader:
      VERIFY(ntp.handleMessage(message));
      timeStamp = ntp.receiveTimeStamp;
      return false;

    case idNTPIdentifier:
    case idNTPRequest:
    case idNTPResponse:
      return ntp.handleMessage(message);

    case idRobot:
      message.bin >> robotNumber;
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
        {
          theTeammateData.timeStamps[robotNumber] = timeStamp;
          if(!theTeammateData.isBHumanPlayer[robotNumber])
          {
            char team;
            message.bin >> team;
            theTeammateData.behaviorStatus[robotNumber].teamColor = (team == 0) ? BehaviorStatus::blue : BehaviorStatus::red;
          }
        }
      return true;

    case idDropInPlayer:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
        {
          theTeammateData.isBHumanPlayer[robotNumber] = false;
          theTeammateData.timeStamps[robotNumber] = timeStamp = SystemCall::getCurrentSystemTime();
          char robotFallen;
          message.bin >> robotFallen; // 1 means that the robot is fallen, 0 means that the robot can play
          unsigned pseudoNetworkTimestanp;
          message.bin >> pseudoNetworkTimestanp;
          bool fallen = robotFallen >= 1;
          message.bin >> ballAge;
          theTeammateData.isPenalized[robotNumber] = !(theOwnTeamInfo.players[robotNumber].penalty == PENALTY_NONE);
          if(theTeammateData.hasGroundContact[robotNumber] && fallen)
            theTeammateData.timeLastGroundContact[robotNumber] = pseudoNetworkTimestanp;
          theTeammateData.hasGroundContact[robotNumber] = !fallen;
          theTeammateData.cameraHeights[robotNumber] = theCameraMatrix.translation.z;
          theTeammateData.obstacleClusters[robotNumber].obstacles.clear();
          theTeammateData.obstacleModels[robotNumber].obstacles.clear();
        }
      return true;

    case idTeammateIntention:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          message.bin >> theTeammateData.intention[robotNumber];
      return true;

    case idTeammateIsPenalized:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          message.bin >> theTeammateData.isPenalized[robotNumber];
      return true;

    case idTeammateHasGroundContact:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
        {
          message.bin >> theTeammateData.hasGroundContact[robotNumber];
          // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of team mates
          // at many different places in our code! For a detailed problem description, ask Tim.
          if(!theTeammateData.hasGroundContact[robotNumber] && theTeammateData.isBHumanPlayer[robotNumber])
            theTeammateData.hasGroundContact[robotNumber] = theFrameInfo.getTimeSince(theTeammateData.timeLastGroundContact[robotNumber]) < 2000;
        }
      return true;

    case idTeammateIsUpright:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          message.bin >> theTeammateData.isUpright[robotNumber];
      return true;

    case idTeammateTimeSinceLastGroundContact:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
        {
          message.bin >> theTeammateData.timeLastGroundContact[robotNumber];
          REMOTE_TO_LOCAL_TIME(theTeammateData.timeLastGroundContact[robotNumber], robotNumber);
        }
      return true;
  }

  // The messages in the following switch block should only be updated
  // if hasGroundContact == true and isPenalized == false, because the information of this message
  // can only be reliable if the robot is actively playing.
  if(!theTeammateData.isPenalized[robotNumber] && theTeammateData.hasGroundContact[robotNumber])
  {
    switch(message.getMessageID())
    {
      case idTeammateBallModel:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            BallModel oldBallModel = theTeammateData.ballModels[robotNumber];
            UNPACK(BallModel, ballModels);
            if(!theTeammateData.isBHumanPlayer[robotNumber] && (timeStamp - theTeammateData.ballModels[robotNumber].timeWhenLastSeen) <= 16)
            {
              theTeammateData.ballModels[robotNumber].lastPerception = oldBallModel.lastPerception;
            }
            REMOTE_TO_LOCAL_TIME(theTeammateData.ballModels[robotNumber].timeWhenLastSeen, robotNumber);
            REMOTE_TO_LOCAL_TIME(theTeammateData.ballModels[robotNumber].timeWhenDisappeared, robotNumber);

            BallModel& ballModel = theTeammateData.ballModels[robotNumber];
            if(!theTeammateData.isBHumanPlayer[robotNumber] &&
               (ballAge == -1 || ballModel.estimate.position.abs() < 30.f))
            {
              theTeammateData.ballModels[robotNumber].timeWhenLastSeen = 0;
              theTeammateData.ballModels[robotNumber].timeWhenDisappeared = 0;
            }
          }
        return true;

      case idTeammateObstacleModel:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            UNPACK(ObstacleModel, obstacleModels);
          }
        return true;

      case idObstacleClusters:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            UNPACK(ObstacleClusters, obstacleClusters);
          }
        return true;

      case idTeammateRobotPose:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            UNPACK(RobotPose, robotPoses);
          }
        return true;

      case idTeammateSideConfidence:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
            message.bin >> theTeammateData.robotsSideConfidence[robotNumber];
        return true;

      case idTeammateBehaviorStatus:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
            message.bin >> theTeammateData.behaviorStatus[robotNumber];
        return true;

      case idTeamCameraHeight:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
            message.bin >> theTeammateData.cameraHeights[robotNumber];
        return true;

      case idTeammateFieldCoverage:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            FieldCoverage::GridInterval& gridInterval = theTeammateData.fieldCoverages[robotNumber];
            message.bin >> gridInterval;
            REMOTE_TO_LOCAL_TIME(gridInterval.timestamp, robotNumber);
          }
        return true;

      case idWalkTarget:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            message.bin >> theTeammateData.walkingTo[robotNumber];
          }
        return true;

      case idKickTarget:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeammateData::firstPlayer && robotNumber < TeammateData::numOfPlayers)
          {
            message.bin >> theTeammateData.shootingTo[robotNumber];
          }
        return true;
    }
  }
  return true;
}

void TeamDataProvider::setRolesForTeammates(TeammateData& teammateData)
{
  //here the roles of the players are handled
  teammateData.behaviorStatus[theRobotInfo.number].role = Role::striker;

  // all not handlned players get role none.
  for(int i = TeammateData::firstPlayer; i < TeammateData::numOfPlayers; ++i)
  {
    if(i != theRobotInfo.number)
    {
      teammateData.behaviorStatus[i].role = Role::none;
    }
  }
}

float TeamDataProvider::estimatedDropInTimeToReachBall(TeammateData& teammateData, int robotNumber)
{
  float timeToReachBall;
  // penalized, fallen robots with unsure or bad TeammateReliability have the max value to get to the ball
  if(!teammateData.isFullyActive[robotNumber] ||
     !(theTeammateReliability.states[robotNumber] == TeammateReliability::OK || theTeammateReliability.states[robotNumber] == TeammateReliability::GOOD))
  {
    return std::numeric_limits<float>::max();
  }

  // TODO maybe use endposition of Ball
  Vector2<> bPos = Transformation::robotToField(teammateData.robotPoses[robotNumber], teammateData.ballModels[robotNumber].estimate.position);
  // distance the robot has to walk to get to the ball. When it's currently walking to a target the way gos over the target.
  const float distanceToTarget = (teammateData.robotPoses[robotNumber].translation - teammateData.walkingTo[robotNumber]).abs();
  const float distance = distanceToTarget + (teammateData.walkingTo[robotNumber] - bPos).abs();
  timeToReachBall = distance / dropInSpeedOfRobot;

  // add time for the rotation to the ball
  timeToReachBall += std::abs(teammateData.ballModels[robotNumber].estimate.position.angle()) / fromDegrees(dropInRotationSpeedOfRobot);

  // when the robot currently dos not see the ball extra time is added
  timeToReachBall += theFrameInfo.getTimeSince(teammateData.ballModels[robotNumber].timeWhenLastSeen);

  // the currend striker gets a time bonus
  if(teammateData.behaviorStatus[robotNumber].role == Role::striker)
  {
    timeToReachBall -= dropInBonusForTheStriker;
  }

  // add time when the relibility state is just OK
  if(theTeammateReliability.states[robotNumber] == TeammateReliability::OK)
  {
    timeToReachBall += dropInRelibilityOK;
  }
  else if(theTeammateReliability.states[robotNumber] == TeammateReliability::GOOD)
  {
    timeToReachBall += dropInRelibilityGOOD;
  }
  return timeToReachBall;
}
