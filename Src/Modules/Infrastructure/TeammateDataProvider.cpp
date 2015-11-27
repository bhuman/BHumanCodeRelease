/**
 * @file Modules/Infrastructure/TeammateDataProvider.cpp
 * This file implements a temporary wrapper from the old to the new TeammateData structure.
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "TeammateDataProvider.h"
#include "Tools/Math/Transformation.h"

/**
 * This macro converts a timestamp (from another robot) into local time via NTP.
 */
#define REMOTE_TO_LOCAL_TIME(timestamp) \
  if(currentTeammate->isBHumanPlayer && timestamp) \
    timestamp = ntp.getRemoteTimeInLocalTime(timestamp);

/**
 * This macro unpacks compressed representations. It reads
 * representationCompressed from the MessageQueue and unpacks it into
 * member.
 */
#define UNPACK(representation, member) \
  representation##Compressed the##representation##Compressed; \
  message.bin >> the##representation##Compressed; \
  currentTeammate->member = the##representation##Compressed;

PROCESS_LOCAL TeammateDataProvider* TeammateDataProvider::theInstance = 0;

TeammateDataProvider::TeammateDataProvider() : lastSentTimestamp(0), lastReceivedTimestamp(0),
                                               currentTeammate(0), teammateDataPtr(0)
{
  theInstance = this;
}

TeammateDataProvider::~TeammateDataProvider()
{
  theInstance = 0;
}

void TeammateDataProvider::update(TeammateData& teammateData)
{
  // Initialize:
  teammateDataPtr = &teammateData;

  // Iterate over deprecated list of teammate information and update some convenience information
  // (new information has already been coming via handleMessages)
  for(auto & teammate : teammateData.teammates)
  {
    if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
      teammate.status = Teammate::INACTIVE;
    else if(!teammate.isUpright || !teammate.hasGroundContact)
      teammate.status = Teammate::ACTIVE;
    else
      teammate.status = Teammate::FULLY_ACTIVE;
    teammate.isGoalkeeper = teammate.number == 1;
  }

  // Remove elements that are too old:
  auto teammate = teammateData.teammates.begin();
  while(teammate != teammateData.teammates.end())
  {
    if(theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
      teammate = teammateData.teammates.erase(teammate);
    else
      ++teammate;
  }

  // Other stuff
  teammateData.numberOfActiveTeammates = 0;
  teammate = teammateData.teammates.begin();
  while(teammate != teammateData.teammates.end())
  {
    if(teammate->status != Teammate::INACTIVE)
      teammateData.numberOfActiveTeammates++;
    ++teammate;
  }

  // Sending interval and NTP synchronization:
  teammateData.sendThisFrame =
#ifdef TARGET_ROBOT
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
#endif
    theFrameInfo.getTimeSince(lastSentTimestamp) >= sendInterval;
  if(teammateData.sendThisFrame)
  {
    // Check, if NTP has to respond
    ntp.doSynchronization(theFrameInfo.time, Global::getTeamOut());
    // Update timestamp
    if(theFrameInfo.getTimeSince(lastSentTimestamp) >= 2 * sendInterval)
      lastSentTimestamp = theFrameInfo.time;
    else
      lastSentTimestamp += sendInterval;
  }

  // Plot some stuff:
  PLOT("module:TeammateDataProvider:ntpOffset10", ntp.timeSyncBuffers[10].bestOffset);
  PLOT("module:TeammateDataProvider:ntpOffset11", ntp.timeSyncBuffers[11].bestOffset);
  PLOT("module:TeammateDataProvider:ntpOffset12", ntp.timeSyncBuffers[12].bestOffset);
}

void TeammateDataProvider::setCurrentTeammate(int robotNumber)
{
  // The robot itself is not a valid teammate.
  // Numbers outside the predefined scope are not valid, too.
  if((robotNumber == theRobotInfo.number) || (robotNumber < Global::getSettings().lowestValidPlayerNumber) ||
     (robotNumber > Global::getSettings().highestValidPlayerNumber))
  {
    currentTeammate = 0;
    return;
  }
  // Try to find the robot in the current list of robots:
  for(auto& teammate : (*teammateDataPtr).teammates)
  {
    if(teammate.number == robotNumber)
    {
      currentTeammate = &teammate;
      return;
    }
  }
  // This seems to be a new robot that is not part of the list yet:
  Teammate newTeammate;
  newTeammate.number = robotNumber;
  teammateDataPtr->teammates.push_back(newTeammate);
  currentTeammate = &(teammateDataPtr->teammates[teammateDataPtr->teammates.size()-1]);
}

void TeammateDataProvider::handleMessages(MessageQueue& teamReceiver)
{
  if(theInstance)
    teamReceiver.handleAllMessages(*theInstance);
  teamReceiver.clear();
}

bool TeammateDataProvider::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
    // Messages for clock synchronization:
    case idNTPHeader:
      VERIFY(ntp.handleMessage(message));
      lastReceivedTimestamp = ntp.receiveTimeStamp;
      return false;
    case idNTPIdentifier:
    case idNTPRequest:
    case idNTPResponse:
      return ntp.handleMessage(message);
    // Robot identification (set the currentTeammate to a new robot:)
    case idRobot:
      int robotNumber;
      message.bin >> robotNumber;
      setCurrentTeammate(robotNumber);
      if(currentTeammate)
        currentTeammate->timeWhenLastPacketReceived = lastReceivedTimestamp;
      return true;
    // Special handling of drop-in players:
    case idDropInPlayer:
      if(currentTeammate)
      {
        currentTeammate->isBHumanPlayer = false;
        currentTeammate->timeWhenLastPacketReceived = lastReceivedTimestamp = SystemCall::getCurrentSystemTime();
        bool fallen;
        message.bin >> fallen;
        currentTeammate->hasGroundContact = !fallen;
        currentTeammate->isUpright = !fallen;
        if(!fallen)
          message.bin >> currentTeammate->timeOfLastGroundContact;
        currentTeammate->obstacleModel.obstacles.clear();
      }
      return true;
    // Robot status:
    case idTeammateIsPenalized:
      if(currentTeammate)
          message.bin >> currentTeammate->isPenalized;
      return true;
    case idTeammateHasGroundContact:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->hasGroundContact;
        // This is a potentially evil quick workaround that should be replaced by a better handling of ground contacts of team mates
        // at many different places in our code! For a detailed problem description, ask Tim.
        if(!currentTeammate->hasGroundContact && currentTeammate->isBHumanPlayer)
          currentTeammate->hasGroundContact = theFrameInfo.getTimeSince(currentTeammate->timeOfLastGroundContact) < 2000;
      }
      return true;
    case idTeammateIsUpright:
      if(currentTeammate)
        message.bin >> currentTeammate->isUpright;
      return true;
    case idTeammateTimeOfLastGroundContact:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->timeOfLastGroundContact;
        REMOTE_TO_LOCAL_TIME(currentTeammate->timeOfLastGroundContact);
      }
      return true;
    // "Normal" representations:
    case idWhistle:
      if(currentTeammate)
      {
        message.bin >> currentTeammate->whistle;
        REMOTE_TO_LOCAL_TIME(currentTeammate->whistle.lastTimeWhistleDetected);
        REMOTE_TO_LOCAL_TIME(currentTeammate->whistle.lastTimeOfIncomingSound);
      }
      return true;
    case idBallModel:
      if(currentTeammate)
      {
        BallModel lastBallModel = currentTeammate->ball;
        UNPACK(BallModel, ball);
        REMOTE_TO_LOCAL_TIME(currentTeammate->ball.timeWhenLastSeen);
        REMOTE_TO_LOCAL_TIME(currentTeammate->ball.timeWhenDisappeared);
        if(!currentTeammate->isBHumanPlayer && (currentTeammate->ball.timeWhenLastSeen - lastBallModel.timeWhenLastSeen) < 16)
          currentTeammate->ball.lastPerception = lastBallModel.lastPerception;
      }
      return true;
    case idObstacleModelCompressed:
      if(currentTeammate)
        message.bin >> currentTeammate->obstacleModel;
      return true;
    case idRobotPose:
      if(currentTeammate)
      {
        UNPACK(RobotPose, pose);
      }
      return true;
    case idSideConfidence:
      if(currentTeammate)
        message.bin >> currentTeammate->sideConfidence;
      return true;
    case idBehaviorStatus:
      if(currentTeammate)
        message.bin >> currentTeammate->behaviorStatus;
      return true;
    case idSPLStandardBehaviorStatus:
      if(currentTeammate)
        message.bin >> currentTeammate->standardBehaviorStatus;
      return true;
    case idTeammateRoles:
      if(currentTeammate)
        message.bin >> currentTeammate->teammateRoles;
      return true;
    default:
      return true;
  }
}

MAKE_MODULE(TeammateDataProvider, cognitionInfrastructure)
