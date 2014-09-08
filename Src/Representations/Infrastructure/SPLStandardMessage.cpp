/**
 * @file SPLStandardMessage.cpp
 * The file implements a class that encapsulates the structure SPLStandardMessage
 * defined in the file SPLStandardMessage.h that is provided with the GameController.
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#include "SPLStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Tools/Debugging/Debugging.h"
#include <algorithm>

/** Sorry, offsetof cannot be used in constants with Microsoft's compiler */
#define bhumanHeaderSize offsetof(BHumanHeader, sizeMarker)

/**
* This macro converts a timeStamp from DropIn Players into local time.
*/
#define PSEUDO_REMOTE_TIMESTAMP (SystemCall::getCurrentSystemTime() - 500) \

SPLStandardMessage::SPLStandardMessage()
{
  theOutMsgData.setSize(SPL_STANDARD_MESSAGE_DATA_SIZE);
}

void SPLStandardMessage::toMessageQueue(MessageQueue& in, const unsigned remoteIp)
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

  if(Global::getSettings().isDropInGame)
  {
    const unsigned preudoRemoteTimestemp = PSEUDO_REMOTE_TIMESTAMP;

    in.out.bin << (int)playerNum;
    in.out.bin << teamColor; // for TeamComm3D
    in.out.finishMessage(idRobot);

    in.out.bin << fallen;
    in.out.bin << preudoRemoteTimestemp;
    in.out.bin << ballAge;
    in.out.finishMessage(idDropInPlayer);

    RobotPoseCompressed robotPose;
    robotPose.translation.x = pose[0];
    robotPose.translation.y = pose[1];
    robotPose.rotation = pose[2];
    robotPose.deviation = 300.f;
    robotPose.validity = 255; // 255 == 1 uncompressed
    in.out.bin << robotPose;
    in.out.finishMessage(idTeammateRobotPose);

    BallModelCompressed ballModel;
    ballModel.position.x = ball[0];
    ballModel.position.y = ball[1];
    ballModel.velocity.x = ballVel[0];
    ballModel.velocity.y = ballVel[1];
    if(ballAge >= 0)
    {
      ballModel.timeWhenLastSeen = preudoRemoteTimestemp - ballAge;
      ballModel.timeWhenDisappeared = preudoRemoteTimestemp - ballAge;
      if(ballAge == 0)
      {
        ballModel.lastPerception.x = (short)ball[0];
        ballModel.lastPerception.y = (short)ball[1];
      }
    }
    else
    {
      ballModel.timeWhenLastSeen = 0;
    }
    in.out.bin << ballModel;
    in.out.finishMessage(idTeammateBallModel);

    SideConfidence confidence;
    if(intention == DROPIN_INTENTION_LOST)
    {
      confidence.sideConfidence = 0.f;
      confidence.confidenceState = SideConfidence::CONFUSED;
    }
    else
    {
      confidence.sideConfidence = 0.3f;
      confidence.confidenceState = SideConfidence::UNSURE;
    }
    in.out.bin << confidence;
    in.out.finishMessage(idTeammateSideConfidence);

    bool upright = !fallen;
    in.out.bin << upright;
    in.out.finishMessage(idTeammateIsUpright);

    in.out.bin << upright;
    in.out.finishMessage(idTeammateHasGroundContact);
  }
  else
  {
    const BHumanHeader& header = (const BHumanHeader&) *data;

    in.out.bin << (remoteIp ? remoteIp : (int)playerNum);
    in.out.bin << header.timestamp;
    in.out.bin << SystemCall::getCurrentSystemTime();
    in.out.bin << header.messageSize;
    in.out.finishMessage(idNTPHeader);

    in.out.bin << (int)playerNum;
    in.out.finishMessage(idRobot);

    InBinaryMemory memory(data + bhumanHeaderSize, numOfDataBytes - bhumanHeaderSize);
    memory >> in;

    RobotPoseCompressed robotPose;
    robotPose.translation.x = pose[0];
    robotPose.translation.y = pose[1];
    robotPose.rotation = pose[2];
    robotPose.deviation = header.robotPoseDeviation;
    robotPose.validity = header.robotPoseValidity;
    in.out.bin << robotPose;
    in.out.finishMessage(idTeammateRobotPose);

    BallModelCompressed ballModel;
    ballModel.position.x = ball[0];
    ballModel.position.y = ball[1];
    ballModel.velocity.x = ballVel[0];
    ballModel.velocity.y = ballVel[1];
    ballModel.timeWhenLastSeen = header.ballTimeWhenLastSeen;
    ballModel.timeWhenDisappeared = header.ballTimeWhenDisappeared;
    ballModel.lastPerception.x = header.ballLastPerceptX;
    ballModel.lastPerception.y = header.ballLastPerceptY;
    in.out.bin << ballModel;
    in.out.finishMessage(idTeammateBallModel);
  }

  Vector2<> target;
  target.x = walkingTo[0];
  target.y = walkingTo[1];
  in.out.bin << target;
  in.out.finishMessage(idWalkTarget);

  Vector2<> kickTarget;
  kickTarget.x = shootingTo[0];
  kickTarget.y = shootingTo[1];
  in.out.bin << kickTarget;
  in.out.finishMessage(idKickTarget);

  in.out.bin << intention;
  in.out.finishMessage(idTeammateIntention);
}

unsigned SPLStandardMessage::fromMessageQueue(MessageQueue& out)
{
  intention = DROPIN_INTENTION_DEFAULT;

  out.handleAllMessages(*this);

  BHumanHeader& header = (BHumanHeader&) *data;
  header.timestamp = SystemCall::getCurrentSystemTime();
  header.messageSize = (unsigned short)(sizeof(RoboCup::SPLStandardMessage) - SPL_STANDARD_MESSAGE_DATA_SIZE);

  OutBinarySize sizeStream;
  sizeStream << theOutMsgData;
  numOfDataBytes = (uint16_t)(sizeStream.getSize() + bhumanHeaderSize);

  if(numOfDataBytes <= SPL_STANDARD_MESSAGE_DATA_SIZE)
  {
    header.messageSize += numOfDataBytes;

    OutBinaryMemory memory(data + bhumanHeaderSize);
    memory << theOutMsgData;
  }
  else
  {
    OUTPUT_ERROR("SPL_STANDARD_MESSAGE_DATA_SIZE exceeded!");
    ASSERT(false);
  }

  theOutMsgData.clear();
  return header.messageSize;
}

bool SPLStandardMessage::handleMessage(InMessage& message)
{
  BHumanHeader& header = (BHumanHeader&) *data;
  switch(message.getMessageID())
  {
    case idRobot:
    {
      message.bin >> playerNum;
      return true;
    }
    case idTeam:
    {
      message.bin >> teamColor;
      return true;
    }
    case idTeammateRobotPose:
    {
      RobotPoseCompressed robotPose;
      message.bin >> robotPose;
      pose[0] = robotPose.translation.x;
      pose[1] = robotPose.translation.y;
      pose[2] = robotPose.rotation;
      header.robotPoseDeviation = robotPose.deviation;
      header.robotPoseValidity = robotPose.validity;
      return true;
    }
    case idTeammateBallModel:
    {
      BallModelCompressed ballModel;
      message.bin >> ballModel;
      ball[0] = ballModel.position.x;
      ball[1] = ballModel.position.y;
      ballVel[0] = ballModel.velocity.x;
      ballVel[1] = ballModel.velocity.y;
      header.ballLastPerceptX = ballModel.lastPerception.x;
      header.ballLastPerceptY = ballModel.lastPerception.y;
      header.ballTimeWhenLastSeen = ballModel.timeWhenLastSeen;
      header.ballTimeWhenDisappeared = ballModel.timeWhenDisappeared;
      return true;
    }
    case idTeammateBallAge:
    {
      message.bin >> ballAge;
      return true;
    }
    case idWalkTarget:
    {
      Vector2<> target;
      message.bin >> target;
      walkingTo[0] = target.x;
      walkingTo[1] = target.y;
      return true;
    }
    case idKickTarget:
    {
      Vector2<> target;
      message.bin >> target;
      shootingTo[0] = target.x;
      shootingTo[1] = target.y;
      return true;
    }
    case idTeammateIntention:
    {
      message.bin >> intention;
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
  STREAM(teamColor);
  STREAM(fallen);
  STREAM(pose);
  STREAM(walkingTo);
  STREAM(shootingTo);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(ballVel);
  STREAM(intention);
  STREAM(numOfDataBytes);

  if(out)
    out->write(data, numOfDataBytes);
  else {
    in->read(data, std::min(static_cast<size_t>(numOfDataBytes), static_cast<size_t>(SPL_STANDARD_MESSAGE_DATA_SIZE)));
  }

  STREAM_REGISTER_FINISH;
}

////
// SPLStandardMessageWithoutData
////

SPLStandardMessageWithoutData::SPLStandardMessageWithoutData(const SPLStandardMessageWithoutData& other) : SPLStandardMessage()
{
  *this = other;
}

SPLStandardMessageWithoutData::SPLStandardMessageWithoutData(const SPLStandardMessage& other) : SPLStandardMessage()
{
  *this = other;
}

SPLStandardMessageWithoutData& SPLStandardMessageWithoutData::operator=(const SPLStandardMessageWithoutData& other)
{
  return *this = static_cast<const SPLStandardMessage&>(other);
}

SPLStandardMessageWithoutData& SPLStandardMessageWithoutData::operator=(const SPLStandardMessage& other)
{
  playerNum = other.playerNum;
  teamColor = other.teamColor;
  fallen = other.fallen;
  pose[0] = other.pose[0];
  pose[1] = other.pose[1];
  pose[2] = other.pose[2];
  walkingTo[0] = other.walkingTo[0];
  walkingTo[1] = other.walkingTo[1];
  shootingTo[0] = other.shootingTo[0];
  shootingTo[1] = other.shootingTo[1];
  ballAge = other.ballAge;
  ball[0] = other.ball[0];
  ball[1] = other.ball[1];
  ballVel[0] = other.ballVel[0];
  ballVel[1] = other.ballVel[1];
  intention = other.intention;

  return *this;
}

void SPLStandardMessageWithoutData::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(header);
  STREAM(version);
  STREAM(playerNum);
  STREAM(teamColor);
  STREAM(fallen);
  STREAM(pose);
  STREAM(walkingTo);
  STREAM(shootingTo);
  STREAM(ballAge);
  STREAM(ball);
  STREAM(ballVel);
  STREAM(intention);
  STREAM_REGISTER_FINISH;
}