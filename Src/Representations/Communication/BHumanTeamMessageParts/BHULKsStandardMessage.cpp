/**
 * @file BHULKsStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHULKsStandardMessage.h"
#include "Platform/BHAssert.h"

void BHULKsStandardMessage::serialize(In* in, Out* out)
{
  static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

  STREAM_REGISTER_BEGIN;
  std::string headerRef(header, 4);
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visble in a great way
  STREAM(version);
  STREAM(member);
  STREAM(timestamp);
  STREAM(isUpright);
  STREAM(hasGroundContact);
  STREAM(timeOfLastGroundContact);
  STREAM(isPenalized);
  STREAM(gameControlData)
  STREAM(headYawAngle);
  STREAM(currentlyPerfomingRole, B_HULKs);
  STREAM(roleAssignments, B_HULKs);
  STREAM(kingIsPlayingBall);
  STREAM(passTarget);
  STREAM(timeWhenReachBall);
  STREAM(timeWhenReachBallQueen);
  STREAM(ballTimeWhenLastSeen);
  STREAM(timestampLastJumped);
  STREAM(confidenceOfLastWhistleDetection, B_HULKs);
  STREAM(lastTimeWhistleDetected);
  STREAM(obstacles);
  STREAM(requestsNTPMessage);
  STREAM(ntpMessages);
  STREAM_REGISTER_FINISH;
}

namespace B_HULKs
{
  Out& operator<<(Out& stream, const Obstacle& bHSTMObstacle)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(bHSTMObstacle);
    STREAM_EXT(stream, bHSTMObstacle.center);
    STREAM_EXT(stream, bHSTMObstacle.timestampLastSeen);
    STREAM_EXT(stream, bHSTMObstacle.type, B_HULKs);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In& operator >> (In& stream, Obstacle& bHSTMObstacle)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(bHSTMObstacle);
    STREAM_EXT(stream, bHSTMObstacle.center);
    STREAM_EXT(stream, bHSTMObstacle.timestampLastSeen);
    STREAM_EXT(stream, bHSTMObstacle.type, B_HULKs);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  Out& operator<<(Out& stream, const BNTPMessage& ntpMessage)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ntpMessage);
    STREAM_EXT(stream, ntpMessage.receiver);
    STREAM_EXT(stream, ntpMessage.requestOrigination);
    STREAM_EXT(stream, ntpMessage.requestReceipt);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In& operator >> (In& stream, BNTPMessage& ntpMessage)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ntpMessage);
    STREAM_EXT(stream, ntpMessage.receiver);
    STREAM_EXT(stream, ntpMessage.requestOrigination);
    STREAM_EXT(stream, ntpMessage.requestReceipt);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  Out& operator<<(Out& stream, const OwnTeamInfo& ownTeamInfo)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ownTeamInfo);
    STREAM_EXT(stream, ownTeamInfo.timestampWhenReceived);
    STREAM_EXT(stream, ownTeamInfo.packetNumber);
    STREAM_EXT(stream, ownTeamInfo.gameType);
    STREAM_EXT(stream, ownTeamInfo.state);
    STREAM_EXT(stream, ownTeamInfo.firstHalf);
    STREAM_EXT(stream, ownTeamInfo.kickOffTeam);
    STREAM_EXT(stream, ownTeamInfo.secondaryState);
    STREAM_EXT(stream, ownTeamInfo.dropInTeam);
    STREAM_EXT(stream, ownTeamInfo.dropInTime);
    STREAM_EXT(stream, ownTeamInfo.secsRemaining);
    STREAM_EXT(stream, ownTeamInfo.secondaryTime);
    STREAM_EXT(stream, ownTeamInfo.score);
    STREAM_EXT(stream, ownTeamInfo.playersArePenalized);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In& operator >> (In& stream, OwnTeamInfo& ownTeamInfo)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ownTeamInfo);
    STREAM_EXT(stream, ownTeamInfo.timestampWhenReceived);
    STREAM_EXT(stream, ownTeamInfo.packetNumber);
    STREAM_EXT(stream, ownTeamInfo.gameType);
    STREAM_EXT(stream, ownTeamInfo.state);
    STREAM_EXT(stream, ownTeamInfo.firstHalf);
    STREAM_EXT(stream, ownTeamInfo.kickOffTeam);
    STREAM_EXT(stream, ownTeamInfo.secondaryState);
    STREAM_EXT(stream, ownTeamInfo.dropInTeam);
    STREAM_EXT(stream, ownTeamInfo.dropInTime);
    STREAM_EXT(stream, ownTeamInfo.secsRemaining);
    STREAM_EXT(stream, ownTeamInfo.secondaryTime);
    STREAM_EXT(stream, ownTeamInfo.score);
    STREAM_EXT(stream, ownTeamInfo.playersArePenalized);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  static const char* getName(Role e)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    if(e >= numOfRoles)
      return nullptr;

    static const char* names[static_cast<unsigned>(numOfRoles)] =
    {
      "King",
      "Rook",
      "Queen",
      "Knight",
      "Bishop",
      "beatenPieces"
    };
    return names[static_cast<unsigned>(e)];
  }

  static const char* getName(HearingConfidence e)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    if(e == HearingConfidence::iAmDeaf || e == HearingConfidence(0))
      return "iAmDeaf";

    if(e == HearingConfidence::heardOnOneEarButThinkingBothAreOk || e == HearingConfidence(1))
      return "heardOnOneEarButThinkingBothAreOk";

    if(e == HearingConfidence::oneEarIsBroken || e == HearingConfidence(2))
      return "oneEarIsBroken";

    if(e == HearingConfidence::allEarsAreOk || e == HearingConfidence(3))
      return "allEarsAreOk";

    return nullptr;
  }

  static const char* getName(ObstacleType e)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    if(e >= numOfObstacleTypes)
      return nullptr;

    static const char* names[static_cast<unsigned>(numOfObstacleTypes)] =
    {
      "goalpost",
      "unknown",
      "someRobot",
      "opponent",
      "teammate",
      "fallenSomeRobot",
      "fallenOpponent",
      "fallenTeammate"
    };
    return names[static_cast<unsigned>(e)];
  }
}
