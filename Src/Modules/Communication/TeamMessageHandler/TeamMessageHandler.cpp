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
  _(StrategyStatus); \
  _(IndirectKick); \
  _(InitialToReady);

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
  theTeamMessageChannel(inTeamMessage, outTeamMessage),
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
  DECLARE_PLOT("module:TeamMessageHandler:budgetLimit");
  DECLARE_DEBUG_RESPONSE("module:TeamMessageHandler:statistics");
  MODIFY("module:TeamMessageHandler:statistics", statistics);

  DEBUG_RESPONSE("module:TeamMessageHandler:budgetLimit")
  {
    const int timeRemainingInCurrentHalf = std::max(0, -theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds));
    const int timeInNextHalf = theGameState.phase == GameState::firstHalf ? durationOfHalf : 0;
    const int remainingTime = std::max(0, timeRemainingInCurrentHalf - lookahead) + timeInNextHalf;
    const float ratio = Rangef::ZeroOneRange().limit(remainingTime / (durationOfHalf * 2.f));

    PLOT("module:TeamMessageHandler:budgetLimit", overallMessageBudget * ratio + normalMessageReserve * (1.f - ratio));
  }

  PLOT("module:TeamMessageHandler:previewMessageBudget", ownModeledBudget);

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  // Update ball constraint to send based on differences between the own ball and the team ball
  const Vector2f ballEndPosition = BallPhysics::getEndPosition(theBallModel.estimate.position,
                                   theBallModel.estimate.velocity,
                                   theBallSpecification.friction);
  const Vector2f teamBallEndPosition = theRobotPose.inverse() * BallPhysics::getEndPosition(theTeammatesBallModel.position,
                                       theTeammatesBallModel.velocity,
                                       theBallSpecification.friction);

  if(!globalBearingsChanged(theRobotPose, ballEndPosition, theRobotPose, teamBallEndPosition, mapToRange(ballEndPosition.norm(), teamBallDistanceInterpolationRange.min, teamBallDistanceInterpolationRange.max, positionThreshold, teamBallMaxPositionThreshold)))
    timeWhenBallWasNearTeamBall = theFrameInfo.time;

  outputGenerator.sendThisFrame = [this]
  {
    bool alwaysSend = this->alwaysSend;
    DEBUG_RESPONSE("module:TeamMessageHandler:alwaysSend")
      alwaysSend = true;
    const bool stateAllowsSending = notInPlayDead() && !theGameState.isPenaltyShootout()
                                    && (!theGameState.isPenalized() || Global::getSettings().scenario.starts_with("SharedAutonomy"));
    const bool alwaysSendAllowed = alwaysSend && enoughTimePassed();
    const bool alwaysSendPlaying = alwaysSendInPlaying && enoughTimePassed() && theGameState.isPlaying() && withinPriorityBudget();
    const bool gestureDetectedSend = (theGameState.state == GameState::standby || theGameState.isReady()) && theInitialToReady.gestureDetected && withinPriorityBudget();
    const bool whistleDetectedSend = (theGameState.isReady() || theGameState.isSet() || theGameState.isPlaying()) && withinPriorityBudget() && whistleDetected();
    const bool indirectKickChangedSend = theGameState.isPlaying() && withinPriorityBudget() && indirectKickChanged();
    const bool canSendPriorityMessage = stateAllowsSending && (alwaysSendAllowed || alwaysSendPlaying || gestureDetectedSend || whistleDetectedSend || indirectKickChangedSend);
    const bool normalChangeDetected = enoughTimePassed() && theGameState.isPlaying() && robotPoseValid() && withinNormalBudget() &&
    (behaviorStatusChanged() || robotStatusChanged() || strategyStatusChanged() || robotPoseChanged() || ballModelChanged() || teamBallOld());

    if(!canSendPriorityMessage && !normalChangeDetected)
      setTimeDelay();

    return canSendPriorityMessage || (normalChangeDetected && checkTimeDelay());
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
    setTimeDelay();
    ownModeledBudget -= std::min(ownModeledBudget, 1u);

    // Plot length of message:
    PLOT("module:TeamMessageHandler:messageLength", outTeamMessage.length);
  };
}

bool TeamMessageHandler::writeMessage(BHumanMessageOutputGenerator& outputGenerator, TeamMessageChannel::Container* const m)
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
  timeWhenLastSent = theFrameInfo.time;
  backup(outputGenerator);

  return true;
}

void TeamMessageHandler::update(ReceivedTeamMessages& receivedTeamMessages)
{
  // Reset representation (should contain only data from current frame).
  receivedTeamMessages.messages.clear();
  receivedTeamMessages.unsynchronizedMessages = 0;

  // Prepare timestamp conversion by updating the GameController packet buffer.
  theGameControllerRBS.update();

  while(theTeamMessageChannel.receive())
  {
    if(readTeamMessage(&inTeamMessage))
    {
      theGameControllerRBS << receivedMessageContainer;

      // Don't accept messages from robots to which we do not know a time offset yet.
      if(dropUnsynchronizedMessages && !theGameControllerRBS[receivedMessageContainer.playerNumber]->isValid())
      {
        ANNOTATION("TeamMessageHandler", "Got unsynchronized message from " << receivedMessageContainer.playerNumber << ".");
        ++receivedTeamMessages.unsynchronizedMessages;
        continue;
      }

      lastReceivedTimestamps[receivedMessageContainer.playerNumber - Settings::lowestValidPlayerNumber] = receivedMessageContainer.timestamp;

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

  handleBudgetPreview(receivedTeamMessages);
}

void TeamMessageHandler::handleBudgetPreview(ReceivedTeamMessages& receivedTeamMessages)
{
  // Budget update from GameController
  if(theGameState.ownTeam.messageBudget != lastReceivedBudget)
  {
    // Reset own model
    lastReceivedBudget = ownModeledBudget = theGameState.ownTeam.messageBudget;
    // All messages until this moment, including messages in this frame, are assumed to be received by the GC too
  }
  else
    ownModeledBudget -= std::min(ownModeledBudget, static_cast<unsigned>(receivedTeamMessages.messages.size()));
}

bool TeamMessageHandler::readTeamMessage(const TeamMessageChannel::Container* const m)
{
  if(!receivedMessageContainer.read(m->data, m->length))
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch;
    return false;
  }

  receivedMessageContainer.playerNumber &= 15;

#ifndef SELF_TEST
  if(receivedMessageContainer.playerNumber == theGameState.playerNumber)
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage;
    return false;
  }
#endif // !SELF_TEST

  if(receivedMessageContainer.playerNumber < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.playerNumber > Settings::highestValidPlayerNumber)
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::invalidPlayerNumber;
    return false;
  }

  // Duplicate messages actually exist (cf. RoboCup German Open 2024). In that case, they arrived
  // immediately after each other, but not necessarily in the same frame. It is unclear whether,
  // if multiple messages are sent within a short timespan, those can overtake each other (such that
  // at the receiving robot, the sequence looks like A B A B instead of A A B B).
  const unsigned lastTimestamp = lastReceivedTimestamps[receivedMessageContainer.playerNumber - Settings::lowestValidPlayerNumber];
  if(receivedMessageContainer.timestamp == lastTimestamp)
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::duplicate;
    return false;
  }

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
                                               const RobotPose& oldOrigin, const Vector2f& oldOffset,
                                               const std::optional<float>& positionalThreshold) const
{
  const float usedPositionThreshold = positionalThreshold.has_value() ? positionalThreshold.value() : positionThreshold;
  const Vector2f oldOffsetInCurrent = origin.inverse() * (oldOrigin * oldOffset);
  const Angle distanceAngle = Vector2f(offset.norm(), assumedObservationHeight).angle();
  const Angle oldDistanceAngle = Vector2f(oldOffsetInCurrent.norm(), assumedObservationHeight).angle();
  return ((offset - oldOffsetInCurrent).squaredNorm() > sqr(usedPositionThreshold) &&
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
    if((offset - oldOffset).squaredNorm() > sqr(positionThreshold) &&
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

bool TeamMessageHandler::checkTimeDelay() const
{
  // When switching to striker, sending is allowed without delay
  // Otherwise wait 0.6 to 1.2 seconds to allow other robots to send important information
  // TODO determine better parameters
  // TODO 600 ms min delay, because we currently do not have a preview of the message budget.
  // If we have -> could go down to 200 ms? But max should remain at 1200 ms?
  return theFrameInfo.getTimeSince(timeWhenLastSendTryStarted) >
         (Role::isActiveRole(theStrategyStatus.role) && !Role::isActiveRole(lastSent.theStrategyStatus.role) ?
          sendDelayPlayBall :
          mapToRange(static_cast<int>(theBallModel.estimate.position.norm()), ballDistanceRangeForDelay.min, ballDistanceRangeForDelay.max, sendDelayRange.min, sendDelayRange.max));
}

void TeamMessageHandler::setTimeDelay()
{
  timeWhenLastSendTryStarted = theFrameInfo.time;
}

bool TeamMessageHandler::withinNormalBudget() const
{
  const int timeRemainingInCurrentHalf = std::max(0, -theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds));
  const int timeInNextHalf = theGameState.phase == GameState::firstHalf ? durationOfHalf : 0;
  const int remainingTime = std::max(0, timeRemainingInCurrentHalf - lookahead) + timeInNextHalf;
  return (static_cast<int>(ownModeledBudget - normalMessageReserve) * durationOfHalf * 2 >
          static_cast<int>(overallMessageBudget - normalMessageReserve) * remainingTime);
}

bool TeamMessageHandler::withinPriorityBudget() const
{
  return ownModeledBudget > priorityMessageReserve;
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
  auto goalKeeperPositionSwitch = [](const Tactic::Position::Type position, const Tactic::Position::Type lastPosition) -> bool
  {
    return Tactic::Position::isGoalkeeper(position) && Tactic::Position::isGoalkeeper(lastPosition);
  };

  return (theStrategyStatus.proposedTactic != lastSent.theStrategyStatus.proposedTactic ||
          // theStrategyStatus.acceptedTactic != lastSent.theStrategyStatus.acceptedTactic ||
          theStrategyStatus.proposedMirror != lastSent.theStrategyStatus.proposedMirror ||
          theStrategyStatus.acceptedMirror != lastSent.theStrategyStatus.acceptedMirror ||
          theStrategyStatus.proposedSetPlay != lastSent.theStrategyStatus.proposedSetPlay ||
          // theStrategyStatus.acceptedSetPlay != lastSent.theStrategyStatus.acceptedSetPlay ||
          // theStrategyStatus.setPlayStep != lastSent.theStrategyStatus.setPlayStep ||
          (theStrategyStatus.position != lastSent.theStrategyStatus.position && !goalKeeperPositionSwitch(theStrategyStatus.position, lastSent.theStrategyStatus.position)) ||
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
    const Vector2f ballEndPosition = BallPhysics::getEndPosition(theBallModel.estimate.position,
                                                                 theBallModel.estimate.velocity,
                                                                 theBallSpecification.friction);
    const Vector2f oldBallEndPosition = BallPhysics::getEndPosition(lastSent.theBallModel.estimate.position,
                                                                    lastSent.theBallModel.estimate.velocity,
                                                                    theBallSpecification.friction);

    return globalBearingsChanged(theRobotPose, ballEndPosition, lastSent.theRobotPose, oldBallEndPosition) &&
           teammateBearingsChanged(theRobotPose * ballEndPosition, lastSent.theRobotPose * oldBallEndPosition) &&
           (!theTeammatesBallModel.isValid ||
            theFrameInfo.getTimeSince(timeWhenBallWasNearTeamBall) > minTimeBallIsNotNearTeamBall);
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

bool TeamMessageHandler::indirectKickChanged() const
{
  return theIndirectKick.lastKickTimestamp > lastSent.theIndirectKick.lastKickTimestamp && !theIndirectKick.allowDirectKick && lastSent.theIndirectKick.lastKickTimestamp < theIndirectKick.lastSetPlayTime; // lastSetPlayTime checks every GameState change
}
