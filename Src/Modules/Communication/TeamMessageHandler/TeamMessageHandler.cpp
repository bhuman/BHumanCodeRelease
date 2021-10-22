/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Platform/Time.h"

//#define SITTING_TEST
//#define SELF_TEST

MAKE_MODULE(TeamMessageHandler, communication);

// BNTP, RobotStatus, RobotPose, FieldCoverage, RobotHealth and FieldFeatureOverview cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(FrameInfo); \
  _(BallModel); \
  _(ObstacleModel); \
  _(Whistle); \
  _(BehaviorStatus); \
  _(TeamBehaviorStatus); \
  _(TeamTalk);

struct TeamMessage
{};

void TeamMessageHandler::regTeamMessage()
{
  PUBLISH(regTeamMessage);
  const char* name = typeid(TeamMessage).name();
  TypeRegistry::addClass(name, nullptr);
#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) TypeRegistry::addAttribute(name, typeid(x).name(), "the" #x)

  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotStatus);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(REGISTER_TEAM_MESSAGE_REPRESENTATION);
}

TeamMessageHandler::TeamMessageHandler() :
  theBNTP(theFrameInfo, theRobotInfo)
{
  File f("teamMessage.def", "r");
  ASSERT(f.exists());
  std::string source(f.getSize(), 0);
  f.read(source.data(), source.length());
  teamCommunicationTypeRegistry.addTypes(source);
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage");
}

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator)
{
  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanStandardMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    theMotionRequest.motion != MotionRequest::playDead &&
    theMotionInfo.executedPhase != MotionPhase::playDead &&
#endif
#endif // !SITTING_TEST
    (theFrameInfo.getTimeSince(timeLastSent) >= sendInterval || theFrameInfo.time < timeLastSent);

  theRobotStatus.hasGroundContact = theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionRequest.motion != MotionRequest::getUp;
  theRobotStatus.isUpright = theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
  if(theRobotStatus.hasGroundContact)
    theRobotStatus.timeOfLastGroundContact = theFrameInfo.time;
  if(theRobotStatus.isUpright)
    theRobotStatus.timeWhenLastUpright = theFrameInfo.time;

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

  outputGenerator.theBHumanStandardMessage.timestamp = Time::getCurrentSystemTime();

  theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  // The other members of theRobotStatus are filled in the update method.
  theRobotStatus.sequenceNumbers.fill(-1);
  for(const Teammate& teammate : theTeamData.teammates)
    theRobotStatus.sequenceNumbers[teammate.number - Settings::lowestValidPlayerNumber] = teammate.sequenceNumber;
  theRobotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber] = outputGenerator.sentMessages % 15;

  outputGenerator.theBSPLStandardMessage.fallen = !theRobotStatus.hasGroundContact || !theRobotStatus.isUpright;

  outputGenerator.theBHumanStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theBHumanStandardMessage.compressedContainer, outputGenerator.theBHumanStandardMessage.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.theBHumanStandardMessage.out = &stream;

  SEND_PARTICLE(BNTP);

  SEND_PARTICLE(RobotStatus);

  if(sendMirroredRobotPose)
  {
    RobotPose theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    SEND_PARTICLE(MirroredRobotPose);
  }
  else
    SEND_PARTICLE(RobotPose);

  FOREACH_TEAM_MESSAGE_REPRESENTATION(SEND_PARTICLE);

  SEND_PARTICLE(FieldCoverage);

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  SEND_PARTICLE(RobotHealth);
  SEND_PARTICLE(FieldFeatureOverview);

  outputGenerator.theBHumanStandardMessage.out = nullptr;

  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
}

void TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const
{
  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theBHumanStandardMessage.write(reinterpret_cast<void*>(m->data));

  const int offset = outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage();
  const int restBytes = SPL_STANDARD_MESSAGE_DATA_SIZE - offset;

  int sizeOfArbitraryMessage;
  if((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes)
  {
    OUTPUT_ERROR("outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage() > restBytes "
                 "-- with size of " << sizeOfArbitraryMessage << " and restBytes " << restBytes);

    do
      outputGenerator.theBHumanArbitraryMessage.queue.removeLastMessage();
    while((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes
          && !outputGenerator.theBHumanArbitraryMessage.queue.isEmpty());
  }

  ASSERT(sizeOfArbitraryMessage <= restBytes);

  outputGenerator.theBHumanArbitraryMessage.write(reinterpret_cast<void*>(m->data + offset));

  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset + sizeOfArbitraryMessage);
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m->header[0]));

  outputGenerator.sentMessages++;
  if(theFrameInfo.getTimeSince(timeLastSent) >= 2 * sendInterval)
    timeLastSent = theFrameInfo.time;
  else
    timeLastSent += sendInterval;
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const SPLStandardMessageBufferEntry* const m)
  {
    if(readSPLStandardMessage(m))
    {
      theBNTP << receivedMessageContainer;
      // Don't accept messages from robots to which we do not know a time offset yet.
      if(!theBNTP[receivedMessageContainer.theBSPLStandardMessage.playerNum]->isValid())
        return;

      return parseMessageIntoBMate(getBMate(teamData));
    }

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
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
      Teammate::Status newStatus = Teammate::PLAYING;
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        newStatus = Teammate::FALLEN;

      if(newStatus != teammate.status)
      {
        teammate.status = newStatus;
        teammate.timeWhenStatusChanged = theFrameInfo.time;
      }

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
bool TeamMessageHandler::readSPLStandardMessage(const SPLStandardMessageBufferEntry* const m)
{
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->message.header[0]))
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

  if(!receivedMessageContainer.theBHumanStandardMessage.read(m->message.data))
    PARSING_ERROR(BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

  if(receivedMessageContainer.theBHumanStandardMessage.magicNumber != Global::getSettings().magicNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

  const size_t offset = receivedMessageContainer.theBHumanStandardMessage.sizeOfBHumanMessage();
  if(!receivedMessageContainer.theBHumanArbitraryMessage.read(m->message.data + offset))
    PARSING_ERROR(BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER " message part reading failed");

  receivedMessageContainer.timestamp = m->timestamp;

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

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketSent = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theBHumanStandardMessage.timestamp);
  currentTeammate.timeWhenLastPacketReceived = receivedMessageContainer.timestamp;

  CompressedTeamCommunicationIn stream(receivedMessageContainer.theBHumanStandardMessage.compressedContainer,
                                       receivedMessageContainer.theBHumanStandardMessage.timestamp, teamMessageType,
                                       [this](unsigned u) { return receivedMessageContainer.toLocalTimestamp(u); });
  receivedMessageContainer.theBHumanStandardMessage.in = &stream;

  RobotStatus robotStatus;
  robotStatus << receivedMessageContainer;

  currentTeammate.isPenalized = robotStatus.isPenalized;
  currentTeammate.isUpright = robotStatus.isUpright;
  currentTeammate.hasGroundContact = robotStatus.hasGroundContact;
  currentTeammate.timeWhenLastUpright = robotStatus.timeWhenLastUpright;
  currentTeammate.timeOfLastGroundContact = robotStatus.timeOfLastGroundContact;
  currentTeammate.sequenceNumber = robotStatus.sequenceNumbers[currentTeammate.number - Settings::lowestValidPlayerNumber];
  currentTeammate.returnSequenceNumber = robotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber];

  RECEIVE_PARTICLE(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(RECEIVE_PARTICLE);
  RECEIVE_PARTICLE(FieldCoverage);
  RECEIVE_PARTICLE(RobotHealth);

  receivedMessageContainer.theBHumanStandardMessage.in = nullptr;

  receivedMessageContainer.theBHumanArbitraryMessage.queue.handleAllMessages(currentTeammate);
}
