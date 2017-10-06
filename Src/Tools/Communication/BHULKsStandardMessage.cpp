/**
 * @file BHULKsStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHULKsStandardMessage.h"
#include <cassert>
#include <algorithm>

namespace B_HULKs
{
  constexpr float pi = 3.1415926535897932384626433832795f;

  template<typename T> inline void writeVal(void*& data, T value)
  {
    *reinterpret_cast<T*&>(data)++ = value;
  }

  template<typename T> inline T readVal(const void*& data)
  {
    return *reinterpret_cast<T*&>(data)++;
  }

  int Obstacle::sizeOfObstacle()
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");
    return 5;
  }

  void Obstacle::write(void*& data, uint32_t timestamp) const
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");
#ifndef NDEBUG
    const void* const begin = data; //just for length check
#endif // NDEBUG

    static_assert(numOfObstacleTypes <= ObstacleType(8), "Following does not work for that many ObstacleTypes. Adjust it!");

    writeVal<int16_t>(data, static_cast<int16_t>(((static_cast<int16_t>(center[0]) >> 2) & 0x3FFF) | ((static_cast<int>(type) & 0xC) << 12)));
    writeVal<int16_t>(data, static_cast<int16_t>(((static_cast<int16_t>(center[1]) >> 2) & 0x3FFF) | ((static_cast<int>(type) & 0x3) << 14)));

    const uint32_t timestampLastSeenDiff64 = (timestamp - timestampLastSeen) >> 6;
    writeVal<uint8_t>(data, static_cast<uint8_t>(timestampLastSeenDiff64 > 0xFE ? 0xFF : timestampLastSeenDiff64));

    assert((reinterpret_cast<char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfObstacle());
  }

  void Obstacle::read(const void*& data, uint32_t timestamp)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");
    const int16_t center0Struct = readVal<const int16_t>(data);
    const int16_t center1Struct = readVal<const int16_t>(data);

    center[0] = static_cast<float>(static_cast<int16_t>(center0Struct << 2));
    center[1] = static_cast<float>(static_cast<int16_t>(center1Struct << 2));

    type = static_cast<ObstacleType>((static_cast<uint16_t>(center0Struct & 0xC000) >> 12) | (static_cast<uint16_t>(center1Struct & 0xC000) >> 14));

    const uint8_t timestampLastSeenDiff64 = readVal<const uint8_t>(data);
    timestampLastSeen = timestamp - (static_cast<uint32_t>(timestampLastSeenDiff64) << 6);
  }

  OwnTeamInfo::OwnTeamInfo() :
    timestampWhenReceived(0)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");
  }

  int OwnTeamInfo::sizeOfOwnTeamInfo() const
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");

    return 1 //timestampWhenReceived
           + sizeof packetNumber
           + 1 //gameType, state, firstHalf, secondaryState
           + 1 // kickOffTeam or dropInTeam
           + 3 // dropInTime (5), secsRemaining(10), secondaryTime(9)
           + 2 // score, players
           ;
  }

  void OwnTeamInfo::write(void*& data, uint32_t timestamp) const
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");

#ifndef NDEBUG
    const void* const begin = data; //just for length check
#endif // NDEBUG

    writeVal<uint8_t>(data, static_cast<uint8_t>((std::min(static_cast<unsigned>(0xFFFF), timestamp - timestampWhenReceived) >> 8) & 0xFF));
    writeVal<uint8_t>(data, packetNumber);

    writeVal<uint8_t>(data, static_cast<uint8_t>((gameType & 3) << 6 | (state & 7) << 3 | (firstHalf & 1) << 2 | (secondaryState & 3)));
    writeVal<uint8_t>(data, state == STATE_PLAYING ? dropInTeam : kickOffTeam);

    writeVal<uint16_t>(data, static_cast<uint16_t>(std::min(63u, 1u + dropInTime) >> 1 << 11 | (secsRemaining & 0x3FF) << 1 | (secondaryTime >> 8 & 1)));
    writeVal<uint8_t>(data, static_cast<uint8_t>(secondaryTime & 0xFF));

    uint16_t scorePlusIsPenalized = score;
    for(unsigned i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      (scorePlusIsPenalized <<= 1) |= playersArePenalized[i] ? 1 : 0;
    writeVal<uint16_t>(data, scorePlusIsPenalized);

    assert((reinterpret_cast<char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfOwnTeamInfo());
  }

  void OwnTeamInfo::read(const void*& data, uint32_t timestamp)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");

    timestampWhenReceived = timestamp - (readVal<const uint8_t>(data) << 8);
    if(timestamp - timestampWhenReceived == 0xFF00)
      timestampWhenReceived = 0;

    packetNumber = readVal<const int8_t>(data);

    const uint8_t stateStruct = readVal<const int8_t>(data);
    gameType = stateStruct >> 6;
    state = (stateStruct >> 3) & 7;
    firstHalf = (stateStruct >> 2) & 1;
    secondaryState = stateStruct & 3;

    if(state == STATE_PLAYING)
      dropInTeam = readVal<const int8_t>(data);
    else
      kickOffTeam = readVal<const int8_t>(data);

    const uint16_t timeStruct = readVal<const int16_t>(data);
    dropInTime = static_cast<uint16_t>(timeStruct >> 11 << 1);
    secsRemaining = static_cast<uint16_t>((timeStruct >> 1) & 0x3FF);
    secondaryTime = static_cast<uint16_t>((stateStruct & 1) << 8);
    secondaryTime |= readVal<const int8_t>(data);

    const uint16_t scorePenaltyStruct = readVal<const int16_t>(data);
    score = scorePenaltyStruct >> BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS;
    unsigned runner = 1 << (BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - 1);
    for(unsigned i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, runner >>= 1)
      playersArePenalized[i] = (runner & scorePenaltyStruct) != 0;
  }

  BHULKsStandardMessage::BHULKsStandardMessage() :
    version(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION),
    member(-1),
    timestamp(0),
    isUpright(true),
    hasGroundContact(true),
    timeOfLastGroundContact(0u),
    isPenalized(true),
    gameControlData(),
    headYawAngle(0.f),
    currentlyPerfomingRole(Role::beatenPieces),
    kingIsPlayingBall(false),
    passTarget(-1),
    timeWhenReachBall(std::numeric_limits<uint32_t>::max()),
    timeWhenReachBallQueen(std::numeric_limits<uint32_t>::max()),
    ballTimeWhenLastSeen(0),
    timestampLastJumped(0),
    confidenceOfLastWhistleDetection(HearingConfidence::iAmDeaf),
    lastTimeWhistleDetected(0),
    obstacles(),
    requestsNTPMessage(false),
    ntpMessages()
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "The constructor is not adjusted for the current message version");

    const char* init = BHULKS_STANDARD_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];

    roleAssignments[0] = Role::King;
    for(int i = 1; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      roleAssignments[i] = Role::Queen;
  }

  int BHULKsStandardMessage::sizeOfBHULKsMessage() const
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

    return sizeof header
           + sizeof version
           + sizeof timestamp
           + 1 // headYawAngle
           + 1 // timeOfLastGroundContact
           + 1 // timestampLastJumped
           + 2 // timeWhenReachBall
           + 2 // timeWhenReachBallQueen
           + 4 // ballTimeWhenLastSeen
           + 2 // whistle stuff
           + gameControlData.sizeOfOwnTeamInfo()
           + 4 // roleAssignments, currentlyPerfomingRole
           + 1 // /\, passTarget, \/
           + std::min(int(obstacles.size()), BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES) * Obstacle::sizeOfObstacle()
           + 2 // member, isUpright, hasGroundContact, isPenalized, kingIsPlayingBall, requestsNTPMessage, \/
           + static_cast<int>(ntpMessages.size()) * 5;
    ;
  }

  bool BHULKsStandardMessage::read(const void* data)
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

#ifndef NDEBUG
    const void* const begin = data; //just for length check
#endif // NDEBUG

    obstacles.clear();
    ntpMessages.clear();

    for(unsigned i = 0; i < sizeof(header); ++i)
      if(header[i] != readVal<const char>(data))
        return false;

    version = readVal<const uint8_t>(data);
    if(version != BHULKS_STANDARD_MESSAGE_STRUCT_VERSION)
      return false;

    timestamp = readVal<const uint32_t>(data);

    headYawAngle = static_cast<float>(readVal<const int8_t>(data)) / 180.f * pi;

    timeOfLastGroundContact = timestamp - (static_cast<uint32_t>(readVal<const uint8_t>(data)) << 6);
    timestampLastJumped = timestamp - (static_cast<uint32_t>(readVal<const uint8_t>(data)) << 7);

    timeWhenReachBall = timestamp + (static_cast<uint32_t>(readVal<const uint16_t>(data)) << 3);
    timeWhenReachBallQueen = timestamp + (static_cast<uint32_t>(readVal<const uint16_t>(data)) << 3);

    ballTimeWhenLastSeen = readVal<const uint32_t>(data);

    static_assert(numOfHearingConfidences == HearingConfidence(4), "Following does not work for that many HearingConfidences. Adjust it!");
    static const HearingConfidence hearingConfidences[4] = { HearingConfidence::iAmDeaf, HearingConfidence::heardOnOneEarButThinkingBothAreOk, HearingConfidence::oneEarIsBroken, HearingConfidence::allEarsAreOk };

    const uint16_t whistleDetectionContainer = readVal<const uint16_t>(data);
    confidenceOfLastWhistleDetection = hearingConfidences[whistleDetectionContainer >> 14];
    if((whistleDetectionContainer & 0x3FFF) > 0x3FFCu)
      lastTimeWhistleDetected = 0;
    else
      lastTimeWhistleDetected = timestamp - (whistleDetectionContainer & 0x3FFF);

    gameControlData.read(data, timestamp);

    static_assert(static_cast<unsigned>(numOfRoles) <= 8u, "Following does not work for that many roles. Adjust it!");
    const uint32_t roleContainer = readVal<const uint32_t>(data);

    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 10, "Following does not work for that many teammates. Adjust it!");
    unsigned runner = 0x7 << ((BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - 1) * 3);
    for(unsigned i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, runner >>= 3)
      roleAssignments[i] = static_cast<Role>((roleContainer & runner) >> ((BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - i - 1) * 3));

    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES < 8, "Following does not work for that many obstacles. Adjust it!");
    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS < 16, "Following does not work for that many teammates. Adjust it!");
    const uint8_t numObsCurrPerforRolePassTargContainer = readVal<const uint8_t>(data);
    passTarget = numObsCurrPerforRolePassTargContainer >> 4;
    currentlyPerfomingRole = static_cast<Role>(((numObsCurrPerforRolePassTargContainer & 8) >> 1) | (roleContainer >> (BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS * 3)));

    obstacles.resize(numObsCurrPerforRolePassTargContainer & 0x7u);
    for(auto& obstacle : obstacles)
      obstacle.read(data, timestamp);

    const uint16_t boolAndNTPReceiptContainer = readVal<const uint16_t>(data);
    runner = 1 << 15;
    member = (boolAndNTPReceiptContainer & runner) != 0 ? 1 : 0;
    isUpright = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
    hasGroundContact = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
    isPenalized = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
    kingIsPlayingBall = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
    requestsNTPMessage = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 10, "Following does not work for that many teammates. Adjust it!");
    for(uint8_t i = 1; runner != 0; ++i)
      if(boolAndNTPReceiptContainer & (runner >>= 1))
      {
        ntpMessages.emplace_back();
        BNTPMessage& message = ntpMessages.back();
        message.receiver = i;

        const uint32_t timeStruct32 = readVal<const uint32_t>(data);
        const uint8_t timeStruct8 = readVal<const uint8_t>(data);

        message.requestOrigination = timeStruct32 & 0xFFFFFFF;
        message.requestReceipt = timestamp - ((timeStruct32 >> 20) & 0xF00u) | static_cast<uint32_t>(timeStruct8);
      }

    assert((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHULKsMessage());
    return true;
  }

#define CUT_TO_PLUS_MINUS(value, positiveRange) std::max(std::min( (value), positiveRange), -positiveRange)
  void BHULKsStandardMessage::write(void* data) const
  {
    static_assert(BHULKS_STANDARD_MESSAGE_STRUCT_VERSION == 8, "This method is not adjusted for the current message version");

#ifndef NDEBUG
    const void* const begin = data; //just for length check
#endif // NDEBUG

    for(unsigned i = 0; i < sizeof(header); ++i)
      writeVal<char>(data, header[i]);

    writeVal<uint8_t>(data, version);

    writeVal<uint32_t>(data, timestamp);

    writeVal<int8_t>(data, static_cast<int8_t>(CUT_TO_PLUS_MINUS(headYawAngle * 180 / pi, 127.f)));

    const uint32_t timeOFLastGroundContactDiff64ms = (timestamp - timeOfLastGroundContact) >> 6;
    writeVal<uint8_t>(data, static_cast<uint8_t>(timeOFLastGroundContactDiff64ms > 200 ? 0xFF : timeOFLastGroundContactDiff64ms));

    const uint32_t timestampLastJumpedDiff128 = (timestamp - timestampLastJumped) >> 7;
    writeVal<uint8_t>(data, static_cast<uint8_t>(timestampLastJumpedDiff128 > 250 ? 0xFF : timestampLastJumpedDiff128));

    const uint32_t timeWhenReachBallDiff8 = (std::max(timestamp, timeWhenReachBall) - timestamp) >> 3;
    writeVal<uint16_t>(data, static_cast<uint16_t>(timeWhenReachBallDiff8 > 0xFFFE ? 0xFFFF : timeWhenReachBallDiff8));

    const uint32_t timeWhenReachBallQueenDiff8 = (std::max(timestamp, timeWhenReachBallQueen) - timestamp) >> 3;
    writeVal<uint16_t>(data, static_cast<uint16_t>(timeWhenReachBallQueenDiff8 > 0xFFFC ? 0xFFFD : timeWhenReachBallQueenDiff8));

    writeVal<uint32_t>(data, ballTimeWhenLastSeen);

    static_assert(numOfHearingConfidences == HearingConfidence(4), "Following does not work for that many HearingConfidences. Adjust it!");
    const uint16_t confidenceOfLastWhistleDetectionConverted =
      (confidenceOfLastWhistleDetection == HearingConfidence::iAmDeaf ? 0
       : (confidenceOfLastWhistleDetection == HearingConfidence::heardOnOneEarButThinkingBothAreOk ? 1
          : (confidenceOfLastWhistleDetection == HearingConfidence::oneEarIsBroken ? 2
             : 3)));

    const uint16_t lastTimeWhistleDetectedDiff = static_cast<uint16_t>(std::min(timestamp - lastTimeWhistleDetected, 0x3FFFu));
    writeVal<uint16_t>(data, static_cast<uint16_t>(confidenceOfLastWhistleDetectionConverted << 14) | lastTimeWhistleDetectedDiff);

    gameControlData.write(data, timestamp);

    static_assert(static_cast<unsigned>(numOfRoles) <= 8u, "Following does not work for that many roles. Adjust it!");
    uint32_t roleContainer = static_cast<uint32_t>(currentlyPerfomingRole);
    for(int i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      (roleContainer <<= 3) |= static_cast<uint32_t>(roleAssignments[i]);
    writeVal<uint32_t>(data, roleContainer); //losing 1 bit; taking it next

    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES < 8, "Following does not work for that many obstacles. Adjust it!");
    static_assert(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS < 16, "Following does not work for that many teammates. Adjust it!");
    const int numOfObstacles = std::min(BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES, static_cast<int>(obstacles.size()));
    writeVal<uint8_t>(data, static_cast<uint8_t>(numOfObstacles | ((static_cast<uint8_t>(currentlyPerfomingRole) & 4) << 1) | (passTarget << 4)));
    for(int i = 0; i < numOfObstacles; ++i)
      obstacles.at(i).write(data, timestamp);

    uint16_t boolContainer = 0;
    (boolContainer <<= 1) |= member ? 1 : 0;
    (boolContainer <<= 1) |= isUpright ? 1 : 0;
    (boolContainer <<= 1) |= hasGroundContact ? 1 : 0;
    (boolContainer <<= 1) |= isPenalized ? 1 : 0;
    (boolContainer <<= 1) |= kingIsPlayingBall ? 1 : 0;
    (boolContainer <<= 1) |= requestsNTPMessage ? 1 : 0;

    std::sort(const_cast<std::vector<BNTPMessage>&>(ntpMessages).begin(), const_cast<std::vector<BNTPMessage>&>(ntpMessages).end(), [&](const BNTPMessage& a, const BNTPMessage& b) {return a.receiver < b.receiver; });
    uint16_t ntpReceivers = 0;
    if(!ntpMessages.empty())
    {
      auto ntpMessagesItr = ntpMessages.cbegin();
      for(unsigned i = 0; i < BHULKS_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, ntpReceivers <<= 1)
        if(ntpMessagesItr == ntpMessages.cend())
          continue;
        else if(ntpMessagesItr->receiver == i + 1)
        {
          ntpReceivers |= 1;
          ntpMessagesItr++;
        }
    }
    writeVal<uint16_t>(data, static_cast<uint16_t>((boolContainer << 10) | (ntpReceivers >> 1)));
    for(const BNTPMessage& ntpMessage : ntpMessages)
    {
      uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
      writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00u) << 20));
      writeVal<uint8_t>(data, static_cast<uint8_t>(requestReceiptDiffCutted & 0xFF));
    }

    assert((reinterpret_cast<char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHULKsMessage());
  }
}