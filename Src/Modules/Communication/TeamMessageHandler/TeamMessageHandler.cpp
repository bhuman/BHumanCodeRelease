/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Platform/Time.h"

//#define SITTING_TEST
//#define SELF_TEST

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type,format,expression) \
  { Global::getTeamOut().format << expression;\
    Global::getTeamOut().finishMessage(type); }

MAKE_MODULE(TeamMessageHandler, communication)

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator)
{
  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
#endif
#endif // !SITTING_TEST
    theFrameInfo.getTimeSince(timeLastSent) >= sendInterval;

  outputGenerator.generate = [this, &outputGenerator](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, m);
  };
}

void TeamMessageHandler::generateMessage(BHumanMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theBSPLStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  outputGenerator.theBHumanStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theBHULKsStandardMessage.member = B_HUMAN_MEMBER;

  outputGenerator.theBHULKsStandardMessage.timestamp = Time::getCurrentSystemTime();
  outputGenerator.theBHULKsStandardMessage.headYawAngle = theJointAngles.angles[Joints::headYaw];

  outputGenerator.theBHULKsStandardMessage.hasGroundContact = theGroundContactState.contact && theMotionInfo.motion != MotionInfo::getUp && theMotionRequest.motion != MotionRequest::getUp;
  outputGenerator.theBHULKsStandardMessage.isUpright = theFallDownState.state == theFallDownState.upright;
  if(theGroundContactState.contact) //FIXME waiting for reponse of andi -> TASK: speaking with tim instead; if this is not deliberately it hast to be outputGenerator.theBHumanStandardMessage.hasGroundContact instead
    outputGenerator.theBHULKsStandardMessage.timeOfLastGroundContact = theFrameInfo.time;

  outputGenerator.theBHULKsStandardMessage.isPenalized = theRobotInfo.penalty != PENALTY_NONE;

  outputGenerator.theBSPLStandardMessage.fallen =
    !outputGenerator.theBHULKsStandardMessage.hasGroundContact || !outputGenerator.theBHULKsStandardMessage.isUpright;

  SEND_PARTICLE(BNTP);
  SEND_PARTICLE(RawGameInfo);
  SEND_PARTICLE(OwnTeamInfo);

  SEND_PARTICLE(BallModel);
  SEND_PARTICLE(RobotPose);

  SEND_PARTICLE(SideConfidence);
  SEND_PARTICLE(TeammateRoles);
  SEND_PARTICLE(BehaviorStatus);
  SEND_PARTICLE(SPLStandardBehaviorStatus);

  SEND_PARTICLE(Whistle);

  //Send this last of important data, because they are the biggest
  SEND_PARTICLE(ObstacleModel);
  SEND_PARTICLE(FieldCoverage);

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  SEND_PARTICLE(RobotHealth);
  SEND_PARTICLE(FieldFeatureOverview);

  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHULKsStandardMessage.sizeOfBHULKsMessage()
                          + outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
}

void TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const
{
  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theBHULKsStandardMessage.write(reinterpret_cast<void*>(m->data));
  int offset = outputGenerator.theBHULKsStandardMessage.sizeOfBHULKsMessage();
  outputGenerator.theBHumanStandardMessage.write(reinterpret_cast<void*>(m->data + offset));
  offset += outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage();

  const int restBytes = SPL_STANDARD_MESSAGE_DATA_SIZE - offset;
  ASSERT(restBytes > 10);

  int sizeOfArbitraryMessage;
  if((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes)
  {
    OUTPUT_ERROR("outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage() > restBytes "
                 "-- with size of " << sizeOfArbitraryMessage << "and restBytes:" << int(restBytes));

    do
      outputGenerator.theBHumanArbitraryMessage.queue.removeLastMessage();
    while((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes
          && !outputGenerator.theBHumanArbitraryMessage.queue.isEmpty());
  }

  ASSERT(sizeOfArbitraryMessage < restBytes);

  outputGenerator.theBHumanArbitraryMessage.write(reinterpret_cast<void*>(m->data + offset));

  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset + sizeOfArbitraryMessage);
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m->header[0]));

  outputGenerator.sentMessages++;
  timeLastSent = theFrameInfo.time;
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const RoboCup::SPLStandardMessage* const m)
  {
    if(readSPLStandardMessage(m))
      return parseMessageIntoBMate(getBMate(teamData));

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruder-alert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };

  maintainBMateList(teamData);
}

void TeamMessageHandler::maintainBMateList(TeamData& teamData) const
{
  //@author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
  {
    // Iterate over deprecated list of teammate information and update some convenience information
    // (new information has already been coming via handleMessages)
    for(auto& teammate : teamData.teammates)
    {
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        teammate.status = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        teammate.status = Teammate::FALLEN;
      else
        teammate.status = Teammate::PLAYING;

      teammate.isGoalkeeper = teammate.number == 1;
    }

    // Remove elements that are too old:
    auto teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
        teammate = teamData.teammates.erase(teammate);
      else
        ++teammate;
    }

    // Other stuff
    teamData.numberOfActiveTeammates = 0;
    teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->status != Teammate::PENALIZED)
        teamData.numberOfActiveTeammates++;
      ++teammate;
    }
  }
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError;  return false; }
bool TeamMessageHandler::readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m)
{
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->header[0]))
    PARSING_ERROR("BSPL" " message part reading failed");

#ifndef SELF_TEST
  if(receivedMessageContainer.theBSPLStandardMessage.playerNum == theRobotInfo.number)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received");

  if(receivedMessageContainer.theBSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("Invalid team number received");

  if(!receivedMessageContainer.theBHULKsStandardMessage.read(m->data))
    PARSING_ERROR(BHULKS_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

#ifdef TARGET_ROBOT
  if(receivedMessageContainer.theBHULKsStandardMessage.member == B_HUMAN_MEMBER)
#endif
  {
    size_t offset = receivedMessageContainer.theBHULKsStandardMessage.sizeOfBHULKsMessage();
    if(!receivedMessageContainer.theBHumanStandardMessage.read(m->data + offset))
      PARSING_ERROR(BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

    if(receivedMessageContainer.theBHumanStandardMessage.magicNumber != Global::getSettings().magicNumber)
      return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

    offset += receivedMessageContainer.theBHumanStandardMessage.sizeOfBHumanMessage();
    if(!receivedMessageContainer.theBHumanArbitraryMessage.read(m->data + offset))
      PARSING_ERROR(BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER " message part reading failed");
  }

  return true;
}

Teammate& TeamMessageHandler::getBMate(TeamData& teamData) const
{
  teamData.receivedMessages++;

  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theBSPLStandardMessage.playerNum)
      return teammate;

  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
  currentTeammate.number = receivedMessageContainer.theBSPLStandardMessage.playerNum;
  currentTeammate.mateType = receivedMessageContainer.theBHULKsStandardMessage.member == B_HUMAN_MEMBER ? Teammate::BHumanRobot : Teammate::HULKsRobot;
  theBNTP << receivedMessageContainer;

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketReceived = theFrameInfo.time;

  currentTeammate.isUpright = receivedMessageContainer.theBHULKsStandardMessage.isUpright;
  currentTeammate.timeOfLastGroundContact = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theBHULKsStandardMessage.timeOfLastGroundContact);
  currentTeammate.hasGroundContact = receivedMessageContainer.theBHULKsStandardMessage.hasGroundContact;

  currentTeammate.isPenalized = receivedMessageContainer.theBHULKsStandardMessage.isPenalized;
  currentTeammate.theRawGCData = receivedMessageContainer.theBHULKsStandardMessage.gameControlData;

  currentTeammate.headYawAngle = receivedMessageContainer.theBHULKsStandardMessage.headYawAngle;

  RECEIVE_PARTICLE(SideConfidence);
  RECEIVE_PARTICLE(RobotPose);
  RECEIVE_PARTICLE(BallModel);
  RECEIVE_PARTICLE(ObstacleModel);
  RECEIVE_PARTICLE(BehaviorStatus);
  RECEIVE_PARTICLE(SPLStandardBehaviorStatus);
  RECEIVE_PARTICLE(Whistle);
  RECEIVE_PARTICLE(TeammateRoles);
  RECEIVE_PARTICLE(FieldCoverage);
  RECEIVE_PARTICLE(RobotHealth);

  receivedMessageContainer.theBHumanArbitraryMessage.queue.handleAllMessages(currentTeammate);
}
