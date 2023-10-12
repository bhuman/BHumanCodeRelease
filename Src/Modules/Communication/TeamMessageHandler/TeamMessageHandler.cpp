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
#include "Debugging/Plot.h"
#include "Framework/Settings.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Streaming/Global.h"
#include <algorithm>

//#define SITTING_TEST
//#define SELF_TEST

MAKE_MODULE(TeamMessageHandler);

// GameControllerRBS and RobotPose cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(RobotStatus); \
  _(FrameInfo); \
  _(BallModel); \
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
#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) \
  TypeRegistry::addAttribute(name, (std::string(#x) == "Whistle" ? typeid(WhistleCompact) : typeid(x)).name(), "the" #x)

  TypeRegistry::addAttribute(name, typeid(RobotPoseCompact).name(), "theRobotPose");
  FOREACH_TEAM_MESSAGE_REPRESENTATION(REGISTER_TEAM_MESSAGE_REPRESENTATION);
}

TeamMessageHandler::TeamMessageHandler() :
  theTeamMessageChannel(inTeamMessages, outTeamMessage),
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
  theTeamMessageChannel.startLocal(Settings::getPortForTeam(Global::getSettings().teamNumber), static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  theTeamMessageChannel.start(Settings::getPortForTeam(Global::getSettings().teamNumber));
#endif
}

void TeamMessageHandler::update(BHumanMessageOutputGenerator& outputGenerator)
{
  DECLARE_PLOT("module:TeamMessageHandler:messageLength");
  DECLARE_DEBUG_RESPONSE("module:TeamMessageHandler:statistics");
  MODIFY("module:TeamMessageHandler:statistics", statistics);

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

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
    if(!writeMessage(outputGenerator, &outTeamMessage))
      return;
    theTeamMessageChannel.send();

    // Plot length of message:
    PLOT("module:TeamMessageHandler:messageLength", outTeamMessage.length);
  };
}

bool TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, TeamMessageChannel::Buffer::Container* const m)
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.playerNumber = static_cast<uint8_t>(theGameState.playerNumber);

  outputGenerator.timestamp = theFrameInfo.time;

  outputGenerator.compressedContainer.reserve(sizeof(m->data));
  CompressedTeamCommunicationOut stream(outputGenerator.compressedContainer, outputGenerator.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.out = &stream;

  SEND_PARTICLE(GameControllerRBS);

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

  outputGenerator.playerNumber |= theRobotHealth.maxJointTemperatureStatus << 4;

  outputGenerator.out = nullptr;

  if(outputGenerator.sizeOfBHumanMessage() > sizeof(m->data))
  {
    OUTPUT_ERROR("BHumanMessage too big (" <<
                 static_cast<unsigned>(outputGenerator.sizeOfBHumanMessage()) <<
                 " > " << static_cast<unsigned>(sizeof(m->data)) << ")");
    return false;
  }

  static_cast<const BHumanMessage&>(outputGenerator).write(reinterpret_cast<void*>(m->data));
  m->length = static_cast<uint8_t>(outputGenerator.sizeOfBHumanMessage());

  DEBUG_RESPONSE("module:TeamMessageHandler:statistics")
  {
    #define COUNT(name) \
      statistics.count(#name, the##name != lastSent.the##name)

    COUNT(RobotStatus.isUpright);
    // COUNT(BehaviorStatus.calibrationFinished);
    COUNT(BehaviorStatus.passTarget);
    statistics.count("BehaviorStatus.shootingTo",
                     globalBearingsChanged(theRobotPose, theBehaviorStatus.shootingTo,
                                           lastSent.theRobotPose, lastSent.theBehaviorStatus.shootingTo));
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

  outputGenerator.sentMessages++;
  if(theFrameInfo.getTimeSince(timeWhenLastSent) >= 2 * minSendInterval || theFrameInfo.time < timeWhenLastSent)
    timeWhenLastSent = theFrameInfo.time;
  else
    timeWhenLastSent += minSendInterval;
  backup(outputGenerator);

  return true;
}

void TeamMessageHandler::update(ReceivedTeamMessages& receivedTeamMessages)
{
  // read from team comm udp socket
  theTeamMessageChannel.receive();

  theGameControllerRBS.update();

  // push teammate data in our system
  receivedTeamMessages.messages.clear();
  receivedTeamMessages.unsynchronizedMessages = 0;
  while(!inTeamMessages.empty())
  {
    TeamMessageChannel::Buffer::Container* const m = inTeamMessages.takeBack();

    if(readTeamMessage(m))
    {
      theGameControllerRBS << receivedMessageContainer;

      // Don't accept messages from robots to which we do not know a time offset yet.
      if(dropUnsynchronizedMessages && !theGameControllerRBS[receivedMessageContainer.playerNumber]->isValid())
      {
        ANNOTATION("TeamMessageHandler", "Got unsynchronized message from " << receivedMessageContainer.playerNumber << ".");
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

    // the message had a parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError; return false; }
bool TeamMessageHandler::readTeamMessage(const TeamMessageChannel::Buffer::Container* const m)
{
  if(!receivedMessageContainer.read(m->data, m->length))
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

  receivedMessageContainer.playerNumber &= 15;

#ifndef SELF_TEST
  if(receivedMessageContainer.playerNumber == theGameState.playerNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.playerNumber < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.playerNumber > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received");

  return true;
}

#define RECEIVE_PARTICLE(particle) teamMessage.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessage(ReceivedTeamMessage& teamMessage)
{
  teamMessage.number = receivedMessageContainer.playerNumber;

  CompressedTeamCommunicationIn stream(receivedMessageContainer.compressedContainer,
                                       receivedMessageContainer.timestamp, teamMessageType,
                                       [smb = theGameControllerRBS[teamMessage.number]](unsigned u) { return smb->getRemoteTimeInLocalTime(u); });
  receivedMessageContainer.in = &stream;

  RECEIVE_PARTICLE(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(RECEIVE_PARTICLE);

  receivedMessageContainer.in = nullptr;
}

#define BACKUP_PARTICLE(particle) lastSent.the##particle << receivedMessageContainer
void TeamMessageHandler::backup(const BHumanMessageOutputGenerator& outputGenerator)
{
  CompressedTeamCommunicationIn stream(outputGenerator.compressedContainer, outputGenerator.timestamp, teamMessageType,
                                       [](unsigned u) { return u; });
  receivedMessageContainer.in = &stream;

  BACKUP_PARTICLE(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(BACKUP_PARTICLE);

  receivedMessageContainer.in = nullptr;
}

bool TeamMessageHandler::globalBearingsChanged(const RobotPose& origin, const std::optional<Vector2f>& offset,
                                               const RobotPose& oldOrigin, const std::optional<Vector2f>& oldOffset) const
{
  if(!offset.has_value() || !oldOffset.has_value())
    return offset.has_value(); // Changed only if zero -> not zero
  else
    return globalBearingsChanged(origin, offset.value(), oldOrigin, oldOffset.value());
}

bool TeamMessageHandler::globalBearingsChanged(const RobotPose& origin, const Vector2f& offset,
                                               const RobotPose& oldOrigin, const Vector2f& oldOffset) const
{
  const Vector2f oldOffsetInCurrent = origin.inverse() * (oldOrigin * oldOffset);
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
    if((offset - oldOffset).norm() > positionThreshold &&
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
  return (// theBehaviorStatus.calibrationFinished != lastSent.theBehaviorStatus.calibrationFinished || // not used
          theBehaviorStatus.passTarget != lastSent.theBehaviorStatus.passTarget ||
          // theBehaviorStatus.walkingTo != lastSent.behaviorStatus.walkingTo || // included in robotPoseChanged
          // theBehaviorStatus.speed != lastSent.behaviorStatus.speed || // included in robotPoseChanged
          globalBearingsChanged(theRobotPose, theBehaviorStatus.shootingTo, lastSent.theRobotPose, lastSent.theBehaviorStatus.shootingTo));
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
    const Vector2f teamBallEndPositon = theRobotPose.inverse() * BallPhysics::getEndPosition(theTeammatesBallModel.position,
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
