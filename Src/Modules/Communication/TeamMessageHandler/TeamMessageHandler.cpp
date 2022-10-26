/**
 * @file TeamMessageHandler.cpp
 *
 * Implements a module that both sends and receives team messages.
 * It ensures that less messages are sent than are allowed. It also checks whether
 * the data that would be sent is significantly different from the data that was last
 * sent. Otherwise, sending the message is skipped.
 *
 * @author Jesse Richter-Klug
 * @author Thomas Röfer
 */

#include "TeamMessageHandler.h"
#include "Representations/Communication/TeamData.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Streaming/Global.h"
#include "Streaming/OutMessage.h"
#include <algorithm>

//#define SITTING_TEST
//#define SELF_TEST

MAKE_MODULE(TeamMessageHandler, communication);

// GameControllerRBS, RobotStatus, RobotPose, RobotHealth, and FieldFeatureOverview cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(FrameInfo); \
  _(BallModel); \
  _(ObstacleModel); \
  _(Whistle); \
  _(BehaviorStatus); \
  _(StrategyStatus);

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
  theSPLMessageHandler(inTeamMessages, outTeamMessage),
  theGameControllerRBS(theFrameInfo, theGameControllerData)
{
  File f("teamMessage.def", "r");
  ASSERT(f.exists());
  std::string source(f.getSize(), 0);
  f.read(source.data(), source.length());
  teamCommunicationTypeRegistry.addTypes(source);
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage");
#ifndef TARGET_ROBOT
  theSPLMessageHandler.startLocal(Settings::getPortForTeam(Global::getSettings().teamNumber), static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  theSPLMessageHandler.start(Settings::getPortForTeam(Global::getSettings().teamNumber));
#endif
}

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator)
{
  DECLARE_PLOT("module:TeamMessageHandler:standardMessageDataBufferUsageInPercent");
  DECLARE_DEBUG_RESPONSE("module:TeamMessageHandler:statistics");
  MODIFY("module:TeamMessageHandler:statistics", statistics);

  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanStandardMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  outputGenerator.sendThisFrame = [this]
  {
    bool alwaysSend = this->alwaysSend;
    DEBUG_RESPONSE("module:TeamMessageHandler:alwaysSend")
      alwaysSend = true;
    return (notInPlayDead() &&
            !theGameState.isPenaltyShootout() &&
            !theGameState.isPenalized() &&
            ((alwaysSend && enoughTimePassed()) ||
             ((theGameState.isReady() || theGameState.isSet() || theGameState.isPlaying()) && withinPriorityBudget() && whistleDetected()) ||
             (enoughTimePassed() && theGameState.isPlaying() && robotPoseValid() && withinNormalBudget() &&
              (behaviorStatusChanged() || robotStatusChanged() || strategyStatusChanged() || robotPoseChanged() || ballModelChanged() || teamBallOld()))));
  };

  theRobotStatus.isUpright = (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting) &&
                             (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionInfo.executedPhase != MotionPhase::fall);
  if(theRobotStatus.isUpright)
    theRobotStatus.timeWhenLastUpright = theFrameInfo.time;

  outputGenerator.send = [this, &outputGenerator]()
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, &outTeamMessage);
    theSPLMessageHandler.send();

    // Plot usage of data buffer in percent:
    const float usageInPercent = 100.f * outTeamMessage.numOfDataBytes / static_cast<float>(SPL_STANDARD_MESSAGE_DATA_SIZE);
    PLOT("module:TeamMessageHandler:standardMessageDataBufferUsageInPercent", usageInPercent);
  };
}

void TeamMessageHandler::generateMessage(BHumanMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theBSPLStandardMessage.playerNum = static_cast<uint8_t>(theGameState.playerNumber);
  outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  outputGenerator.theBHumanStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theBHumanStandardMessage.timestamp = Time::getCurrentSystemTime();

  // The other members of theRobotStatus are filled in the update method.
  theRobotStatus.sequenceNumbers[theGameState.playerNumber - Settings::lowestValidPlayerNumber] = outputGenerator.sentMessages % 15;

  outputGenerator.theBSPLStandardMessage.fallen = !theRobotStatus.isUpright;

  outputGenerator.theBHumanStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theBHumanStandardMessage.compressedContainer, outputGenerator.theBHumanStandardMessage.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.theBHumanStandardMessage.out = &stream;

  SEND_PARTICLE(GameControllerRBS);

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

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  SEND_PARTICLE(RobotHealth);
  SEND_PARTICLE(FieldFeatureOverview);

  outputGenerator.theBHumanStandardMessage.out = nullptr;

  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());

  DEBUG_RESPONSE("module:TeamMessageHandler:statistics")
  {
    #define COUNT(name) \
      statistics.count(#name, the##name != lastSent.the##name)

    COUNT(RobotStatus.isUpright);
    COUNT(BehaviorStatus.activity);
    COUNT(BehaviorStatus.passTarget);
    statistics.count("BehaviorStatus.shootingTo",
                     globalBearingsChanged(theRobotPose, theBehaviorStatus.shootingTo,
                                           lastSent.theRobotPose, lastSent.theBehaviorStatus.shootingTo, true));
    COUNT(StrategyStatus.proposedTactic);
    //COUNT(StrategyStatus.acceptedTactic);
    COUNT(StrategyStatus.proposedMirror);
    COUNT(StrategyStatus.acceptedMirror);
    COUNT(StrategyStatus.proposedSetPlay);
    //COUNT(StrategyStatus.acceptedSetPlay);
    //COUNT(StrategyStatus.setPlayStep);
    COUNT(StrategyStatus.position);
    COUNT(StrategyStatus.role);
    statistics.count("RobotPose.translation", robotPoseChanged());
    statistics.count("GlobalBallEndPosition", ballModelChanged());
    statistics.count("TeamBallOld", teamBallOld());
  }
}

void TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m)
{
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
  if(theFrameInfo.getTimeSince(timeWhenLastSent) >= 2 * minSendInterval || theFrameInfo.time < timeWhenLastSent)
    timeWhenLastSent = theFrameInfo.time;
  else
    timeWhenLastSent += minSendInterval;
  backup();
}

void TeamMessageHandler::update(ReceivedTeamMessages& receivedTeamMessages)
{
  // read from team comm udp socket
  theSPLMessageHandler.receive();

  theGameControllerRBS.update();

  // push teammate data in our system
  receivedTeamMessages.messages.clear();
  receivedTeamMessages.unsynchronizedMessages = 0;
  while(!inTeamMessages.empty())
  {
    const RoboCup::SPLStandardMessage* const m = inTeamMessages.takeBack();

    if(readSPLStandardMessage(m))
    {
      theGameControllerRBS << receivedMessageContainer;

      // Don't accept messages from robots to which we do not know a time offset yet.
      if(dropUnsynchronizedMessages && !theGameControllerRBS[receivedMessageContainer.theBSPLStandardMessage.playerNum]->isValid())
      {
        ANNOTATION("TeamMessageHandler", "Got unsynchronized message from " << receivedMessageContainer.theBSPLStandardMessage.playerNum << ".");
        ++receivedTeamMessages.unsynchronizedMessages;
        continue;
      }

      receivedTeamMessages.messages.emplace_back();
      parseMessage(receivedTeamMessages.messages.back());
      continue;
    }

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) continue;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError; return false; }
bool TeamMessageHandler::readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m)
{
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->header[0]))
    PARSING_ERROR("BSPL" " message part reading failed");

#ifndef SELF_TEST
  if(receivedMessageContainer.theBSPLStandardMessage.playerNum == theGameState.playerNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received");

  if(receivedMessageContainer.theBSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("Invalid team number received");

  if(!receivedMessageContainer.theBHumanStandardMessage.read(m->data))
    PARSING_ERROR(BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

  if(receivedMessageContainer.theBHumanStandardMessage.magicNumber != Global::getSettings().magicNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

  const size_t offset = receivedMessageContainer.theBHumanStandardMessage.sizeOfBHumanMessage();
  if(!receivedMessageContainer.theBHumanArbitraryMessage.read(m->data + offset))
    PARSING_ERROR(BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER " message part reading failed");

  return true;
}

#define RECEIVE_PARTICLE(particle) teamMessage.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessage(ReceivedTeamMessage& teamMessage)
{
  teamMessage.number = receivedMessageContainer.theBSPLStandardMessage.playerNum;

  receivedMessageContainer.bSMB = theGameControllerRBS[teamMessage.number];

  CompressedTeamCommunicationIn stream(receivedMessageContainer.theBHumanStandardMessage.compressedContainer,
                                       receivedMessageContainer.theBHumanStandardMessage.timestamp, teamMessageType,
                                       [this](unsigned u) { return receivedMessageContainer.toLocalTimestamp(u); });
  receivedMessageContainer.theBHumanStandardMessage.in = &stream;

  RobotStatus robotStatus;
  robotStatus << receivedMessageContainer;

  teamMessage.isUpright = robotStatus.isUpright;
  teamMessage.timeWhenLastUpright = robotStatus.timeWhenLastUpright;
  teamMessage.sequenceNumber = robotStatus.sequenceNumbers[teamMessage.number - Settings::lowestValidPlayerNumber];
  teamMessage.returnSequenceNumber = robotStatus.sequenceNumbers[theGameState.playerNumber - Settings::lowestValidPlayerNumber];

  RECEIVE_PARTICLE(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(RECEIVE_PARTICLE);

  receivedMessageContainer.theBHumanStandardMessage.in = nullptr;

  theRobotStatus.sequenceNumbers[teamMessage.number - Settings::lowestValidPlayerNumber] = teamMessage.sequenceNumber;
}

void TeamMessageHandler::backup()
{
  lastSent.theBallModel = theBallModel;
  lastSent.theBehaviorStatus = theBehaviorStatus;
  lastSent.theFrameInfo = theFrameInfo;
  lastSent.theRobotPose = theRobotPose;
  lastSent.theRobotStatus = theRobotStatus;
  lastSent.theStrategyStatus = theStrategyStatus;
  lastSent.theWhistle = theWhistle;
}

bool TeamMessageHandler::globalBearingsChanged(const RobotPose& origin, const Vector2f& offset,
                                               const RobotPose& oldOrigin, const Vector2f& oldOffset,
                                               bool checkZero) const
{
  if(checkZero)
  {
    const bool offsetValid = offset != Vector2f::Zero();
    const bool oldOffsetValid = oldOffset != Vector2f::Zero();
    if(!offsetValid || !oldOffsetValid)
      return offsetValid; // Changed only if zero -> not zero
  }

  const Vector2f oldOffsetInCurrent = origin.inversePose * (oldOrigin * oldOffset);
  const Angle distanceAngle = Vector2f(offset.norm(), assumedObservationHeight).angle();
  const Angle oldDistanceAngle = Vector2f(oldOffsetInCurrent.norm(), assumedObservationHeight).angle();
  return ((offset - oldOffsetInCurrent).norm() > positionThreshold &&
          (offset.isZero() || oldOffsetInCurrent.isZero() ||
           offset.angleTo(oldOffsetInCurrent) > bearingThreshold ||
           std::abs(Angle::normalize(distanceAngle - oldDistanceAngle)) > bearingThreshold));
}

bool TeamMessageHandler::teammateBearingsChanged(const Vector2f& position, const Vector2f& oldPosition) const
{
  for(const Teammate& teammate : theTeamData.teammates)
  {
    const Vector2f estimatedPosition = Teammate::getEstimatedPosition(teammate.theRobotPose,
                                                                      teammate.theBehaviorStatus.walkingTo,
                                                                      teammate.theBehaviorStatus.speed,
                                                                      theFrameInfo.getTimeSince(teammate.theFrameInfo.time));
    const Vector2f offset = position - estimatedPosition;
    const Vector2f oldOffset = oldPosition - estimatedPosition;
    const Angle distanceAngle = Vector2f(offset.norm(), assumedObservationHeight).angle();
    const Angle oldDistanceAngle = Vector2f(oldOffset.norm(), assumedObservationHeight).angle();
    if ((offset - oldOffset).norm() > positionThreshold &&
        (offset.isZero() || oldOffset.isZero() ||
         offset.angleTo(oldOffset) > bearingThreshold ||
         std::abs(Angle::normalize(distanceAngle - oldDistanceAngle)) > bearingThreshold))
      return true;
  }
  return false;
}

bool TeamMessageHandler::enoughTimePassed() const
{
  return theFrameInfo.getTimeSince(timeWhenLastSent) >= minSendInterval || theFrameInfo.time < timeWhenLastSent;
}

bool TeamMessageHandler::notInPlayDead() const
{
#if !defined SITTING_TEST && defined TARGET_ROBOT
  return theMotionRequest.motion != MotionRequest::playDead &&
         theMotionInfo.executedPhase != MotionPhase::playDead;
#else
  return true;
#endif
}

bool TeamMessageHandler::withinNormalBudget() const
{
  const int timeRemainingInCurrentHalf = std::max(0, -theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds));
  const int timeInNextHalf = theGameState.phase == GameState::firstHalf ? durationOfHalf : 0;
  const int remainingTime = std::max(0, timeRemainingInCurrentHalf - lookahead) + timeInNextHalf;
  return (static_cast<int>(theGameState.ownTeam.messageBudget - normalMessageReserve) * durationOfHalf * 2 >
          static_cast<int>(overallMessageBudget - normalMessageReserve) * remainingTime);
}

bool TeamMessageHandler::withinPriorityBudget() const
{
  return theGameState.ownTeam.messageBudget > priorityMessageReserve;
}

bool TeamMessageHandler::whistleDetected() const
{
  const int timeRemainingInCurrentHalf = std::max(0, -theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds));
  return theWhistle.lastTimeWhistleDetected > lastSent.theWhistle.lastTimeWhistleDetected + minSendInterval &&
         timeRemainingInCurrentHalf >= ignoreWhistleBeforeEndOfHalf &&
         theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) <= maxWhistleSendDelay;
}

bool TeamMessageHandler::behaviorStatusChanged() const
{
  return (theBehaviorStatus.activity != lastSent.theBehaviorStatus.activity ||
          theBehaviorStatus.passTarget != lastSent.theBehaviorStatus.passTarget ||
          // theBehaviorStatus.walkingTo != lastSent.behaviorStatus.walkingTo || // included in robotPoseChanged
          // theBehaviorStatus.speed != lastSent.behaviorStatus.speed || // included in robotPoseChanged
          globalBearingsChanged(theRobotPose, theBehaviorStatus.shootingTo, lastSent.theRobotPose, lastSent.theBehaviorStatus.shootingTo, true));
}

bool TeamMessageHandler::robotStatusChanged() const
{
  return theRobotStatus.isUpright != lastSent.theRobotStatus.isUpright;
}

bool TeamMessageHandler::strategyStatusChanged() const
{
  return (theStrategyStatus.proposedTactic != lastSent.theStrategyStatus.proposedTactic ||
          // theStrategyStatus.acceptedTactic != lastSent.theStrategyStatus.acceptedTactic ||
          theStrategyStatus.proposedMirror != lastSent.theStrategyStatus.proposedMirror ||
          theStrategyStatus.acceptedMirror != lastSent.theStrategyStatus.acceptedMirror ||
          theStrategyStatus.proposedSetPlay != lastSent.theStrategyStatus.proposedSetPlay ||
          // theStrategyStatus.acceptedSetPlay != lastSent.theStrategyStatus.acceptedSetPlay ||
          // theStrategyStatus.setPlayStep != lastSent.theStrategyStatus.setPlayStep ||
          theStrategyStatus.position != lastSent.theStrategyStatus.position ||
          theStrategyStatus.role != lastSent.theStrategyStatus.role);
}

bool TeamMessageHandler::robotPoseValid() const
{
  return theRobotPose.quality != RobotPose::LocalizationQuality::poor;
}

bool TeamMessageHandler::robotPoseChanged() const
{
  const Vector2f estimatedPosition = Teammate::getEstimatedPosition(lastSent.theRobotPose,
                                                                    lastSent.theBehaviorStatus.walkingTo,
                                                                    lastSent.theBehaviorStatus.speed,
                                                                    theFrameInfo.getTimeSince(lastSent.theFrameInfo.time));
  return ((theRobotPose.translation - estimatedPosition).norm() > positionThreshold &&
          teammateBearingsChanged(theRobotPose.translation, estimatedPosition));
}

bool TeamMessageHandler::ballModelChanged() const
{
  if((theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) < disappearedThreshold) !=
     (lastSent.theFrameInfo.getTimeSince(lastSent.theBallModel.timeWhenDisappeared) < disappearedThreshold))
    return true;
  else if(theBallModel.timeWhenLastSeen == lastSent.theBallModel.timeWhenLastSeen)
    return false;
  else
  {
    const Vector2f ballEndPositon = BallPhysics::getEndPosition(theBallModel.estimate.position,
                                                                theBallModel.estimate.velocity,
                                                                theBallSpecification.friction);
    const Vector2f oldBallEndPositon = BallPhysics::getEndPosition(lastSent.theBallModel.estimate.position,
                                                                   lastSent.theBallModel.estimate.velocity,
                                                                   theBallSpecification.friction);
    const Vector2f teamBallEndPositon = theRobotPose.inversePose * BallPhysics::getEndPosition(theTeammatesBallModel.position,
                                                                                               theTeammatesBallModel.velocity,
                                                                                               theBallSpecification.friction);
    return globalBearingsChanged(theRobotPose, ballEndPositon, lastSent.theRobotPose, oldBallEndPositon) &&
           teammateBearingsChanged(theRobotPose * ballEndPositon, lastSent.theRobotPose * oldBallEndPositon) &&
           (!theTeammatesBallModel.isValid ||
            globalBearingsChanged(theRobotPose, ballEndPositon, theRobotPose, teamBallEndPositon));
  }
}

bool TeamMessageHandler::teamBallOld() const
{
  // Our ball is old, too
  if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > newBallThreshold)
    return false;

  // Determine the latest ball timestamp that was communicated
  const auto newest = std::max_element(theTeamData.teammates.begin(), theTeamData.teammates.end(),
                                       [](const Teammate& t1, const Teammate& t2)
                                       {return t1.theBallModel.timeWhenLastSeen < t2.theBallModel.timeWhenLastSeen;});
  const unsigned timeWhenLastSeen = std::max(newest == theTeamData.teammates.end() ? 0 : newest->theBallModel.timeWhenLastSeen,
                                             lastSent.theBallModel.timeWhenLastSeen);

  return theFrameInfo.getTimeSince(timeWhenLastSeen) > teamBallThresholdBase + theGameState.playerNumber * teamBallThresholdFactor;
}
