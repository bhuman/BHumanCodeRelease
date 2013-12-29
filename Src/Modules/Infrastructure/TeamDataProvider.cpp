/**
 * @file TeamDataProvider.cpp
 * This file implements a module that provides the data received by team communication.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TeamDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/Team.h"

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * teamMateData.array[robotNumber].
 */
#define UNPACK(representation, array) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  theTeamMateData.array[robotNumber] = the##representation##Compressed;

/**
 * This macro converts a timeStamp into local time via ntp.
 */
#define REMOTE_TO_LOCAL_TIME(timeStamp) \
  if(timeStamp) { timeStamp = ntp.getRemoteTimeInLocalTime(timeStamp); }

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

void TeamDataProvider::update(TeamMateData& teamMateData)
{
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset1"); // 1-5: players
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset2");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset3");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset4");
  DECLARE_PLOT("module:TeamDataProvider:ntpOffset5");

  PLOT("module:TeamDataProvider:ntpOffset1", ntp.timeSyncBuffers[1].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset2", ntp.timeSyncBuffers[2].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset3", ntp.timeSyncBuffers[3].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset4", ntp.timeSyncBuffers[4].bestOffset);
  PLOT("module:TeamDataProvider:ntpOffset5", ntp.timeSyncBuffers[5].bestOffset);

  teamMateData = theTeamMateData;
  teamMateData.currentTimestamp = theFrameInfo.time;
  teamMateData.numOfConnectedTeamMates = 0;
  teamMateData.firstTeamMate = TeamMateData::numOfPlayers;
  for(int i = TeamMateData::firstPlayer; i < TeamMateData::numOfPlayers; ++i)
  {
    teamMateData.isActive[i] = false;
    teamMateData.isFullyActive[i] = false;
    // Check, if network connection is working (-> connected):
    if(teamMateData.timeStamps[i] && theFrameInfo.getTimeSince(teamMateData.timeStamps[i]) < static_cast<int>(teamMateData.networkTimeout))
    {
      teamMateData.numOfConnectedTeamMates++;
      // Check, if team mate is not penalized (-> active):
      if(!teamMateData.isPenalized[i] && theOwnTeamInfo.players[i - 1].penalty == PENALTY_NONE)
      {
        teamMateData.numOfActiveTeamMates++;
        teamMateData.isActive[i] = true;
        if(teamMateData.numOfActiveTeamMates == 1)
          teamMateData.firstTeamMate = i;
        // Check, if team mate has not been fallen down or lost ground contact (-> fully active):
        if(teamMateData.hasGroundContact[i] && teamMateData.isUpright[i])
        {
          teamMateData.numOfFullyActiveTeamMates++;
          teamMateData.isFullyActive[i] = true;
        }
      }
    }
  }
  if(teamMateData.numOfConnectedTeamMates)
    teamMateData.wasConnected = theTeamMateData.wasConnected = true;
  teamMateData.sendThisFrame = (ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut()) ||
                                theFrameInfo.getTimeSince(lastSentTimeStamp) >= 200) // TODO config file?
#ifdef TARGET_ROBOT
                               && !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead)
                               && !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead)
#endif
                               ;
  if(teamMateData.sendThisFrame)
    lastSentTimeStamp = theFrameInfo.time;
}

void TeamDataProvider::handleMessages(MessageQueue& teamReceiver)
{
  if(theInstance)
  {
    teamReceiver.handleAllMessages(*theInstance);
    TEAM_OUTPUT(idRobot, bin, theInstance->theRobotInfo.number);
  }

  teamReceiver.clear();
}

bool TeamDataProvider::handleMessage(InMessage& message)
{
  /*
  The robotNumber and the three flags hasGroundContact, isUpright and isPenalized should always be updated.
   */
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
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          theTeamMateData.timeStamps[robotNumber] = timeStamp;
      return true;

    case idGroundTruthBallModel:
    {
      Vector2<> position;
      message.bin >> theGroundTruthBallModel.timeWhenLastSeen >> position;
      REMOTE_TO_LOCAL_TIME(theGroundTruthBallModel.timeWhenLastSeen);
      if(theOwnTeamInfo.teamColor == TEAM_BLUE)
        position *= -1;
      theGroundTruthBallModel.lastPerception = theGroundTruthRobotPose.invert() * position;
      theGroundTruthBallModel.estimate.position = theGroundTruthBallModel.lastPerception;
    }
      return true;

    case idGroundTruthRobotPose:
    {
      char teamColor,
          id;
      unsigned timeStamp;
      Pose2D robotPose;
      message.bin >> teamColor >> id >> timeStamp >> robotPose;
      if(teamColor == (int) theOwnTeamInfo.teamColor && id == theRobotInfo.number)
      {
        if(theOwnTeamInfo.teamColor == TEAM_BLUE)
          robotPose = Pose2D(pi) + robotPose;
        (Pose2D&) theGroundTruthRobotPose = robotPose;
      }
    }
      return true;

    case idTeamMateIsPenalized:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          message.bin >> theTeamMateData.isPenalized[robotNumber];
      return true;

    case idTeamMateHasGroundContact:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          message.bin >> theTeamMateData.hasGroundContact[robotNumber];
          // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of team mates
          // at many different places in our code! For a detailed problem description, ask Tim.
          if(!theTeamMateData.hasGroundContact[robotNumber])
            theTeamMateData.hasGroundContact[robotNumber] = theFrameInfo.getTimeSince(theTeamMateData.timeLastGroundContact[robotNumber]) < 2000;
        }
      return true;

    case idTeamMateIsUpright:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          message.bin >> theTeamMateData.isUpright[robotNumber];
      return true;

    case idTeamMateTimeSinceLastGroundContact:
      if(robotNumber != theRobotInfo.number)
        if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
        {
          message.bin >> theTeamMateData.timeLastGroundContact[robotNumber];
          REMOTE_TO_LOCAL_TIME(theTeamMateData.timeLastGroundContact[robotNumber]);
        }
      return true;
  }

  /*
  The messages in the following switch block should only be updated
  if hasGroundContact == true and isPenalized == false, because the information of this message
  can only be reliable if the robot is actively playing.
   */
  if(!theTeamMateData.isPenalized[robotNumber] && theTeamMateData.hasGroundContact[robotNumber])
  {
    switch(message.getMessageID())
    {
      case idTeamMateBallModel:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            UNPACK(BallModel, ballModels);
            BallModel& ballModel = theTeamMateData.ballModels[robotNumber];
            REMOTE_TO_LOCAL_TIME(ballModel.timeWhenLastSeen);
            REMOTE_TO_LOCAL_TIME(ballModel.timeWhenDisappeared);
          }
        return true;

      case idTeamMateObstacleModel:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            UNPACK(ObstacleModel, obstacleModels);
          }
        return true;

      case idTeamMateRobotsModel:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            UNPACK(RobotsModel, robotsModels);
            for(size_t i = 0; i < theTeamMateData.robotsModels[robotNumber].robots.size(); i++)
            {
              REMOTE_TO_LOCAL_TIME(theTeamMateData.robotsModels[robotNumber].robots[i].timeStamp);
            }
          }
        return true;

      case idObstacleClusters:
        if(robotNumber != theRobotInfo.number)
        {
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            UNPACK(ObstacleClusters, obstacleClusters);
          }
        }
        return true;

      case idTeamMateRobotPose:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            UNPACK(RobotPose, robotPoses);
          }
        return true;

      case idTeamMateSideConfidence:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
            message.bin >> theTeamMateData.robotsSideConfidence[robotNumber];
        return true;

      case idTeamMateBehaviorStatus:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
            message.bin >> theTeamMateData.behaviorStatus[robotNumber];
        return true;

      case idTeamHeadControl:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
            message.bin >> theTeamMateData.teamHeadControlStates[robotNumber];
        return true;

      case idTeamCameraHeight:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
            message.bin >> theTeamMateData.cameraHeights[robotNumber];
        return true;

      case idTeamMateFieldCoverage:
        if(robotNumber != theRobotInfo.number)
          if(robotNumber >= TeamMateData::firstPlayer && robotNumber < TeamMateData::numOfPlayers)
          {
            FieldCoverage::GridInterval& gridInterval = theTeamMateData.fieldCoverages[robotNumber];
            message.bin >> gridInterval;
            REMOTE_TO_LOCAL_TIME(gridInterval.timestamp);
          }
        return true;
    }
  }

  return true;
}

MAKE_MODULE(TeamDataProvider, Cognition Infrastructure)
