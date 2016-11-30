/**
 * @file SPLStandardMessage.cpp
 * The file implements a struct that encapsulates the structure SPLStandardMessage
 * defined in the file SPLStandardMessage.h that is provided with the GameController.
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "SPLStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Tools/Debugging/Debugging.h"
#include <algorithm>

/**
 * This macro converts a timeStamp from DropIn Players into local time.
 */
#define PSEUDO_REMOTE_TIMESTAMP (Time::getCurrentSystemTime() - 500)

#define BHUMAN_AVG_WALK_SPEED 220
#define BHUMAN_MAX_KICK_DISTANCE 7000

SPLStandardMessage::SPLStandardMessage()
{
  teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  theOutMsgData.setSize(SPL_STANDARD_MESSAGE_DATA_SIZE);
}

SPLStandardMessage::SPLStandardMessage(const SPLStandardMessage& other) :
  RoboCup::SPLStandardMessage(other)
{
  // do not copy theOutMsgData, because it is not possible and not needed
}

SPLStandardMessage::SPLStandardMessage(MessageQueue& out) :
  SPLStandardMessage()
{
  out.handleAllMessages(*this);
  BHumanHeader& header = (BHumanHeader&) * data;
  header.magicNumber = Global::getSettings().magicNumber;
  header.timestamp = Time::getCurrentSystemTime();
  OutBinarySize sizeStream;
  sizeStream << theOutMsgData;
  numOfDataBytes = static_cast<uint16_t>(sizeStream.getSize() + bhumanHeaderSize);

  if(numOfDataBytes <= SPL_STANDARD_MESSAGE_DATA_SIZE)
  {
    OutBinaryMemory memory(data + bhumanHeaderSize);
    memory << theOutMsgData;
  }
  else
  {
    numOfDataBytes = static_cast<uint16_t>(bhumanHeaderSize);
    OUTPUT_ERROR("SPL_STANDARD_MESSAGE_DATA_SIZE exceeded!");
    ASSERT(false);
  }

  theOutMsgData.clear();
}

SPLStandardMessage& SPLStandardMessage::operator=(const SPLStandardMessage& other)
{
  static_cast<RoboCup::SPLStandardMessage&>(*this) = other;
  // do not copy theOutMsgData, because it is not possible and not needed
  return *this;
}

void SPLStandardMessage::toMessageQueue(MessageQueue& in, const unsigned remoteIp, const unsigned realNumOfDataBytes)
{
  if(header[0] != 'S' || header[1] != 'P' || header[2] != 'L' || header[3] != ' ')
  {
    OUTPUT_WARNING("Received package from ip " << remoteIp << " with Header '" << header[0] << header[1] << header[2] << header[3] << "' but should be 'SPL '. Ignoring package...");
    return;
  }

  if(version != SPL_STANDARD_MESSAGE_STRUCT_VERSION)
  {
    OUTPUT_WARNING("Received package from ip " << remoteIp << " with SPL_STANDARD_MESSAGE_STRUCT_VERSION '" << version << "' but should be '" << SPL_STANDARD_MESSAGE_STRUCT_VERSION << "'.Ignoring package...");
    return;
  }

  if(numOfDataBytes != static_cast<uint16_t>(realNumOfDataBytes))
  {
    OUTPUT_WARNING("SPL Message: numOfDataBytes is '" << numOfDataBytes << "' but realNumOfDataBytes is '" << realNumOfDataBytes << "'.");
    numOfDataBytes = std::min(numOfDataBytes, static_cast<uint16_t>(realNumOfDataBytes));

    if(!Global::getSettings().isDropInGame)
    {
      OUTPUT_WARNING("... ignoring package!");
      return;
    }
  }

  if(!Global::getSettings().isDropInGame && numOfDataBytes < bhumanHeaderSize)
  {
    OUTPUT_WARNING("Ignoring SPL Message because: 'numOfDataBytes < bhumanHeaderSize'.");
    return;
  }

  if(Global::getSettings().isDropInGame)
  {
    const unsigned pseudoRemoteTimestamp = PSEUDO_REMOTE_TIMESTAMP;

    in.out.bin << static_cast<int>(playerNum);
    in.out.finishMessage(idRobot);

    in.out.bin << ballAge;
    in.out.bin << fallen;
    in.out.bin << pseudoRemoteTimestamp;
    in.out.finishMessage(idDropInPlayer);

    RobotPoseCompressed robotPose;
    robotPose.translation.x() = pose[0];
    robotPose.translation.y() = pose[1];
    robotPose.rotation = pose[2];
    robotPose.deviation = 100.f + std::max(0.f, 1.f - static_cast<float>(currentPositionConfidence) / 100.f) * 3000.f;
    robotPose.validity = static_cast<unsigned char>(static_cast<int>(currentPositionConfidence) * 255 / 100);
    robotPose.covariance[0] = 50;
    robotPose.covariance[1] = 50;
    robotPose.covariance[2] = 1;
    robotPose.covariance[3] = 0;
    robotPose.covariance[4] = 0;
    robotPose.covariance[5] = 0;
    in.out.bin << robotPose;
    in.out.finishMessage(idRobotPose);

    SideConfidence sideConfidence;
    if(intention == DROPIN_INTENTION_LOST)
      sideConfidence.sideConfidence = 0.f;
    else
      sideConfidence.sideConfidence = static_cast<float>(currentSideConfidence) / 100.f;

    if(sideConfidence.sideConfidence >= 0.95f)
      sideConfidence.confidenceState = SideConfidence::ALMOST_CONFIDENT;
    else if(sideConfidence.sideConfidence > 0.f)
      sideConfidence.confidenceState = SideConfidence::UNSURE;
    else
      sideConfidence.confidenceState = SideConfidence::CONFUSED;
    in.out.bin << sideConfidence;
    in.out.finishMessage(idSideConfidence);

    BallModelCompressed ballModel;
    ballModel.position.x() = ball[0];
    ballModel.position.y() = ball[1];
    ballModel.velocity.x() = ballVel[0];
    ballModel.velocity.y() = ballVel[1];
    ballModel.lastPerception.x() = static_cast<short>(ball[0]);
    ballModel.lastPerception.y() = static_cast<short>(ball[1]);
    ballModel.covariance[0] = 500;
    ballModel.covariance[1] = 500;
    ballModel.covariance[2] = 0;
    if(ballAge >= 0.f && ballModel.position.squaredNorm() > 30.f * 30.f)
      ballModel.timeWhenLastSeen = ballModel.timeWhenDisappeared = std::max(0u, pseudoRemoteTimestamp - static_cast<unsigned>(ballAge * 1000.f));
    else
      ballModel.timeWhenLastSeen = ballModel.timeWhenDisappeared = 0u;
    in.out.bin << ballModel;
    in.out.finishMessage(idBallModel);

    bool upright = !fallen;
    in.out.bin << upright;
    in.out.finishMessage(idTeammateIsUpright);

    in.out.bin << upright;
    in.out.finishMessage(idTeammateHasGroundContact);
  }
  else
  {
    const BHumanHeader& header = (const BHumanHeader&) * data;
    if(header.magicNumber != Global::getSettings().magicNumber)
    {
      return;
    }
    in.out.bin << (remoteIp ? remoteIp : static_cast<int>(playerNum));
    in.out.bin << header.timestamp;
    in.out.bin << Time::getCurrentSystemTime();
    in.out.finishMessage(idNTPHeader);

    in.out.bin << static_cast<int>(playerNum);
    in.out.finishMessage(idRobot);

    InBinaryMemory memory(data + bhumanHeaderSize, numOfDataBytes - bhumanHeaderSize);
    memory >> in;

    RobotPoseCompressed robotPose;
    robotPose.translation.x() = pose[0];
    robotPose.translation.y() = pose[1];
    robotPose.rotation = pose[2];
    robotPose.deviation = header.robotPoseDeviation;
    robotPose.validity = header.robotPoseValidity;
    robotPose.covariance = header.robotPoseCovariance;
    in.out.bin << robotPose;
    in.out.finishMessage(idRobotPose);

    BallModelCompressed ballModel;
    ballModel.position.x() = ball[0];
    ballModel.position.y() = ball[1];
    ballModel.velocity.x() = ballVel[0];
    ballModel.velocity.y() = ballVel[1];
    ballModel.timeWhenLastSeen = header.ballTimeWhenLastSeen;
    ballModel.timeWhenDisappeared = header.ballTimeWhenDisappeared;
    ballModel.lastPerception.x() = header.ballLastPerceptX;
    ballModel.lastPerception.y() = header.ballLastPerceptY;
    ballModel.covariance = header.ballCovariance;
    in.out.bin << ballModel;
    in.out.finishMessage(idBallModel);
  }

  TeammateRoles roles;
  for(size_t i = 0; i < 5; ++i)
    switch(suggestion[i])
    {
    case DROPIN_SUGGESTION_KEEPER:
    case DROPIN_SUGGESTION_DEFENSIVE:
    case DROPIN_SUGGESTION_OFFENSIVE:
    case DROPIN_SUGGESTION_KICK:
      default:
        roles[i + 1] = Role::none;
    }
  in.out.bin << roles;
  in.out.finishMessage(idTeammateRoles);

  SPLStandardBehaviorStatus standardBehaviorStatus;
  standardBehaviorStatus.walkingTo  = Vector2f(walkingTo[0], walkingTo[1]);
  standardBehaviorStatus.shootingTo = Vector2f(shootingTo[0], shootingTo[1]);
  standardBehaviorStatus.intention  = intention;
  standardBehaviorStatus.averageWalkSpeed = averageWalkSpeed;
  standardBehaviorStatus.maxKickDistance  = maxKickDistance;
  in.out.bin << standardBehaviorStatus;
  in.out.finishMessage(idSPLStandardBehaviorStatus);
}

bool SPLStandardMessage::handleMessage(InMessage& message)
{
  BHumanHeader& header = (BHumanHeader&) * data;
  switch(message.getMessageID())
  {
    case idRobot:
    {
      message.bin >> playerNum;
      return true;
    }
    case idRobotPose:
    {
      RobotPoseCompressed robotPose;
      message.bin >> robotPose;
      pose[0] = robotPose.translation.x();
      pose[1] = robotPose.translation.y();
      pose[2] = robotPose.rotation;
      currentPositionConfidence = static_cast<int8_t>(static_cast<int>(robotPose.validity) * 100 / 255);
      header.robotPoseValidity = robotPose.validity;
      header.robotPoseDeviation = robotPose.deviation;
      header.robotPoseCovariance = robotPose.covariance;
      return true;
    }
    case idBallModel:
    {
      BallModelCompressed ballModel;
      message.bin >> ballModel;
      ball[0] = ballModel.position.x();
      ball[1] = ballModel.position.y();
      ballVel[0] = ballModel.velocity.x();
      ballVel[1] = ballModel.velocity.y();
      header.ballLastPerceptX = ballModel.lastPerception.x();
      header.ballLastPerceptY = ballModel.lastPerception.y();
      header.ballTimeWhenLastSeen = ballModel.timeWhenLastSeen;
      header.ballTimeWhenDisappeared = ballModel.timeWhenDisappeared;
      header.ballCovariance = ballModel.covariance;
      return true;
    }
    case idTeammateBallAge:
    {
      message.bin >> ballAge;
      return true;
    }
    case idSPLStandardBehaviorStatus:
    {
      SPLStandardBehaviorStatus standardBehaviorStatus;
      message.bin >> standardBehaviorStatus;
      walkingTo[0]     = standardBehaviorStatus.walkingTo.x();
      walkingTo[1]     = standardBehaviorStatus.walkingTo.y();
      shootingTo[0]    = standardBehaviorStatus.shootingTo.x();
      shootingTo[1]    = standardBehaviorStatus.shootingTo.y();
      intention        = standardBehaviorStatus.intention;
      averageWalkSpeed = BHUMAN_AVG_WALK_SPEED;
      maxKickDistance  = BHUMAN_MAX_KICK_DISTANCE;
      return true;
    }
    case idTeammateRoles:
    {
      TeammateRoles roles;
      message.bin >> roles;
      for(size_t i = 0; i < 5; ++i)
        switch(roles[i + 1])
        {
          case Role::none:
          default:
            suggestion[i] = DROPIN_SUGGESTION_DEFAULT;
        }
      return true;
    }
    case idSideConfidence:
    {
      SideConfidence sideConfidence;
      message.bin >> sideConfidence;
      currentSideConfidence = static_cast<int8_t>(sideConfidence.sideConfidence * 100);
      theOutMsgData.out.bin << sideConfidence;
      theOutMsgData.out.finishMessage(idSideConfidence);
      return true;
    }
    case idTeammateHasGroundContact:
    {
      bool hasContact;
      message.bin >> hasContact;
      fallen = !hasContact;
      theOutMsgData.out.bin << hasContact;
      theOutMsgData.out.finishMessage(idTeammateHasGroundContact);
      return true;
    }
    case idTeammateIsUpright:
    {
      bool isUpright;
      message.bin >> isUpright;
      fallen |= !isUpright;
      theOutMsgData.out.bin << isUpright;
      theOutMsgData.out.finishMessage(idTeammateIsUpright);
      return true;
    }
    default:
    {
      message >> theOutMsgData;
      return true;
    }
  }
}

void SPLStandardMessage::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(header);
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(walkingTo);
  STREAM(shootingTo);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(ballVel);
  STREAM(suggestion);
  STREAM(intention);
  STREAM(averageWalkSpeed);
  STREAM(maxKickDistance);
  STREAM(currentPositionConfidence);
  STREAM(currentSideConfidence);
  STREAM(numOfDataBytes);

  size_t safeNumOfDataBytes = std::min(static_cast<size_t>(numOfDataBytes), static_cast<size_t>(SPL_STANDARD_MESSAGE_DATA_SIZE));
  if(out)
    out->write(data, safeNumOfDataBytes);
  else
    in->read(data, safeNumOfDataBytes);

  STREAM_REGISTER_FINISH;
}

////
// SPLStandardMessageWithoutData
////

SPLStandardMessageWithoutData& SPLStandardMessageWithoutData::operator=(const SPLStandardMessage& other)
{
  static_cast<SPLStandardMessage&>(*this) = other;
  return *this;
}

void SPLStandardMessageWithoutData::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(header);
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamNum);
  STREAM(fallen);
  STREAM(pose);
  STREAM(walkingTo);
  STREAM(shootingTo);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(ballVel);
  STREAM(suggestion);
  STREAM(intention);
  STREAM(averageWalkSpeed);
  STREAM(maxKickDistance);
  STREAM(currentPositionConfidence);
  STREAM(currentSideConfidence);
  STREAM_REGISTER_FINISH;
}
