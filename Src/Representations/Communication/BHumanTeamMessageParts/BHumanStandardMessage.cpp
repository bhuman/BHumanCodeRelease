/**
 * @file BHumanStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

#include <algorithm>
#include <limits>

template<typename T>
inline void writeVal(void*& data, T value)
{
  *reinterpret_cast<T*&>(data)++ = value;
}

template<typename T>
inline T readVal(const void*& data)
{
  return *reinterpret_cast<T*&>(data)++;
}

BHumanStandardMessage::BHumanStandardMessage() :
  version(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION),
  magicNumber(0),
  timestamp(0),
  isPenalized(false),
  isUpright(true),
  hasGroundContact(true),
  timeOfLastGroundContact(0),
  robotPoseValidity(1.f),
  robotPoseDeviation(0.f),
  timestampLastJumped(0),
  ballTimeWhenLastSeen(0),
  ballTimeWhenDisappeared(0),
  ballSeenPercentage(0),
  ballVelocity(Vector2f::Zero()),
  ballLastPercept(Vector2f::Zero()),
  confidenceOfLastWhistleDetection(-1),
  lastTimeWhistleDetected(0),
  obstacles(),
  requestsNTPMessage(false),
  ntpMessages()
{
  const char* init = BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER;
  for(unsigned int i = 0; i < sizeof(header); ++i)
    header[i] = init[i];
}

int BHumanStandardMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

  return sizeof(header)
         + sizeof(version)
         + sizeof(magicNumber)
         + sizeof(timestamp)
         + 1 // timeOfLastGroundContact
         + 1 // robotPoseValidity
         + 4 // robotPoseDeviation
         + 6 * 4 // robotPoseCovariance
         + 1 // timestampLastJumped
         + 4 // ballTimeWhenLastSeen
         + 4 // ballTimeWhenDisappeared, ballSeenPercentage
         + 2 * 2 // ballVelocity
         + 2 * 2 // ballLastPercept
         + 3 * 4 // ballCovariance
         + 1 // confidenceOfLastWhistleDetection
         + 2 // lastTimeWhistleDetected
         + 2 // number of obstacles, isPenalized, isUpright, hasGroundContact, requestsNTPMessage, NTP reply bitset
         + static_cast<int>(obstacles.size()) * 25
         + static_cast<int>(ntpMessages.size()) * 5;
}

#define CLIP_AND_CAST_TO_INT16(x) static_cast<uint16_t>(std::min(std::max((x), -32768.f), 32767.f))

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

#ifndef NDEBUG
  const void* const begin = data; //just for length check
#endif // NDEBUG

  for(unsigned i = 0; i < sizeof(header); ++i)
    writeVal<char>(data, header[i]);
  writeVal<uint8_t>(data, version);
  writeVal<uint8_t>(data, magicNumber);
  writeVal<uint32_t>(data, timestamp);

  const uint32_t timeOfLastGroundContactDiff_64 = (timestamp - std::min(timestamp, timeOfLastGroundContact)) >> 6;
  writeVal<uint8_t>(data, static_cast<uint8_t>(timeOfLastGroundContactDiff_64 > 0xFE ? 0xFF : timeOfLastGroundContactDiff_64));

  const uint8_t robotPoseValidity255 = static_cast<uint8_t>(std::min(std::max(robotPoseValidity * 255.f, 0.f), 255.f));
  writeVal<uint8_t>(data, robotPoseValidity255);
  writeVal<float>(data, robotPoseDeviation);
  for(unsigned int i = 0; i < 6; ++i)
    writeVal<float>(data, robotPoseCovariance[i]);
  const uint32_t timestampLastJumpedDiff_128 = (timestamp - std::min(timestamp, timestampLastJumped)) >> 7;
  writeVal<uint8_t>(data, static_cast<uint8_t>(timestampLastJumpedDiff_128 > 0xFE ? 0xFF : timestampLastJumpedDiff_128));

  writeVal<uint32_t>(data, ballTimeWhenLastSeen);
  writeVal<uint32_t>(data, (ballTimeWhenDisappeared & 0xFFFFFF) | (ballSeenPercentage << 24));
  writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(ballVelocity.x()));
  writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(ballVelocity.y()));
  writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(ballLastPercept.x()));
  writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(ballLastPercept.y()));
  for(unsigned int i = 0; i < 3; ++i)
    writeVal<float>(data, ballCovariance[i]);

  writeVal<int8_t>(data, confidenceOfLastWhistleDetection);
  const uint32_t lastTimeWhistleDetectedDiff = timestamp - std::min(timestamp, lastTimeWhistleDetected);
  writeVal<uint16_t>(data, static_cast<uint16_t>(lastTimeWhistleDetectedDiff > 0xFFFEu ? 0xFFFFu : lastTimeWhistleDetectedDiff));

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team (but can be easily adjusted).");
  uint16_t boolContainer = 0;
  (boolContainer <<= 1) |= isPenalized ? 1 : 0;
  (boolContainer <<= 1) |= isUpright ? 1 : 0;
  (boolContainer <<= 1) |= hasGroundContact ? 1 : 0;
  (boolContainer <<= 1) |= requestsNTPMessage ? 1 : 0;
  std::sort(const_cast<std::vector<BNTPMessage>&>(ntpMessages).begin(), const_cast<std::vector<BNTPMessage>&>(ntpMessages).end(), [&](const BNTPMessage& a, const BNTPMessage& b) {return a.receiver < b.receiver; });
  uint16_t ntpReceivers = 0;
  if(!ntpMessages.empty())
  {
    auto ntpMessagesItr = ntpMessages.cbegin();
    for(unsigned int i = 0; i < BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, ntpReceivers <<= 1)
    {
      if(ntpMessagesItr == ntpMessages.cend())
        continue;
      else if(ntpMessagesItr->receiver == i + 1)
      {
        ntpReceivers |= 1;
        ++ntpMessagesItr;
      }
    }
  }
  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES < 64, "This code only works for up to 64 obstacles to send.");
  ASSERT(obstacles.size() <= BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES);
  writeVal<uint16_t>(data, static_cast<uint16_t>((obstacles.size() << 10) | (boolContainer << 6) | (ntpReceivers >> 1)));
  for(const Obstacle& obstacle : obstacles)
  {
    writeVal<float>(data, obstacle.covariance(0, 0));
    writeVal<float>(data, obstacle.covariance(1, 1));
    writeVal<float>(data, (obstacle.covariance(0, 1) + obstacle.covariance(1, 0)) / 2.f);
    writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(obstacle.center.x()));
    writeVal<int16_t>(data, CLIP_AND_CAST_TO_INT16(obstacle.center.y()));
    writeVal<uint16_t>(data, static_cast<uint16_t>(((CLIP_AND_CAST_TO_INT16(obstacle.left.x()) >> 2) & 0x3FFF) | ((obstacle.type & 0x3) << 14)));
    writeVal<uint16_t>(data, static_cast<uint16_t>(((CLIP_AND_CAST_TO_INT16(obstacle.left.y()) >> 2) & 0x3FFF) | ((obstacle.type & 0xC) << 12)));
    writeVal<uint16_t>(data, static_cast<uint16_t>(((CLIP_AND_CAST_TO_INT16(obstacle.right.x()) >> 2) & 0x3FFF) | ((obstacle.type & 0x30) << 10)));
    writeVal<uint16_t>(data, static_cast<uint16_t>(((CLIP_AND_CAST_TO_INT16(obstacle.right.y()) >> 2) & 0x3FFF) | ((obstacle.type & 0xC0) << 8)));
    const uint32_t lastSeenDiff_64 = (timestamp - std::min(timestamp, obstacle.lastSeen)) >> 6;
    writeVal<uint8_t>(data, static_cast<uint8_t>(lastSeenDiff_64 > 0xFE ? 0xFF : lastSeenDiff_64));
  }
  for(const BNTPMessage& ntpMessage : ntpMessages)
  {
    const uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
    writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00) << 20));
    writeVal<uint8_t>(data, requestReceiptDiffCutted & 0xFF);
  }

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

#ifndef NDEBUG
  const void* const begin = data; //just for length check
#endif // NDEBUG

  obstacles.clear();
  ntpMessages.clear();

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != readVal<const char>(data))
      return false;

  version = readVal<const uint8_t>(data);
  if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  magicNumber = readVal<const uint8_t>(data);
  if(!(Global::settingsExist() && Global::getSettings().magicNumber))
    return false;

  timestamp = readVal<const uint32_t>(data);

  timeOfLastGroundContact = timestamp - (static_cast<uint32_t>(readVal<const uint8_t>(data)) << 6);

  robotPoseValidity = readVal<const uint8_t>(data) / 255.f;
  robotPoseDeviation = readVal<const float>(data);
  for(unsigned int i = 0; i < 6; ++i)
    robotPoseCovariance[i] = readVal<const float>(data);
  timestampLastJumped = timestamp - (static_cast<uint32_t>(readVal<const uint8_t>(data)) << 7);

  ballTimeWhenLastSeen = readVal<const uint32_t>(data);
  const uint32_t ballTimeWhenDisappearedSeenPercentage = readVal<const uint32_t>(data);
  ballTimeWhenDisappeared = ballTimeWhenDisappearedSeenPercentage & 0xFFFFFF;
  ballSeenPercentage = ballTimeWhenDisappearedSeenPercentage >> 24;
  ballVelocity.x() = readVal<const int16_t>(data);
  ballVelocity.y() = readVal<const int16_t>(data);
  ballLastPercept.x() = readVal<const int16_t>(data);
  ballLastPercept.y() = readVal<const int16_t>(data);
  for(unsigned int i = 0; i < 3; ++i)
    ballCovariance[i] = readVal<const float>(data);

  confidenceOfLastWhistleDetection = readVal<const int8_t>(data);
  const uint32_t lastTimeWhistleDetectedDiff = readVal<const uint16_t>(data);
  if(lastTimeWhistleDetectedDiff > 0xFFFEu)
    lastTimeWhistleDetected = 0;
  else
    lastTimeWhistleDetected = timestamp - lastTimeWhistleDetectedDiff;

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team (but can be easily adjusted).");
  const uint16_t boolAndNTPReceiptContainer = readVal<const uint16_t>(data);
  const size_t numOfObstacles = boolAndNTPReceiptContainer >> 10;
  obstacles.resize(numOfObstacles);
  for(size_t i = 0; i < numOfObstacles; ++i)
  {
    Obstacle& o = obstacles[i];
    const float covXX = readVal<const float>(data);
    const float covXY = readVal<const float>(data);
    const float covYY = readVal<const float>(data);
    o.covariance << covXX, covXY, covXY, covYY;

    const int16_t centerX = readVal<const int16_t>(data);
    const int16_t centerY = readVal<const int16_t>(data);
    const uint16_t leftX_4Type = readVal<const uint16_t>(data);
    const uint16_t leftY_4Type = readVal<const uint16_t>(data);
    const uint16_t rightX_4Type = readVal<const uint16_t>(data);
    const uint16_t rightY_4Type = readVal<const uint16_t>(data);
    o.center = Vector2f(centerX, centerY);
    o.left = Vector2f(static_cast<int16_t>(leftX_4Type << 2), static_cast<int16_t>(leftY_4Type << 2));
    o.right = Vector2f(static_cast<int16_t>(rightX_4Type << 2), static_cast<int16_t>(rightY_4Type << 2));
    o.lastSeen = timestamp - (static_cast<uint32_t>(readVal<const uint8_t>(data)) << 6);
    o.type = static_cast<Obstacle::Type>((leftX_4Type >> 14) | ((leftY_4Type >> 12) & 0x0C) | ((rightX_4Type >> 10) & 0x30) | ((rightY_4Type >> 8) & 0xC0));
  }
  uint16_t runner = 1 << 9;
  isPenalized = (boolAndNTPReceiptContainer & runner) != 0;
  isUpright = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
  hasGroundContact = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
  requestsNTPMessage = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
  for(uint8_t i = 1; runner != 0; ++i)
  {
    if(boolAndNTPReceiptContainer & (runner >>= 1))
    {
      ntpMessages.emplace_back();
      BNTPMessage& message = ntpMessages.back();
      message.receiver = i;

      const uint32_t timeStruct32 = readVal<const uint32_t>(data);
      const uint8_t timeStruct8 = readVal<const uint8_t>(data);

      message.requestOrigination = timeStruct32 & 0xFFFFFFF;
      message.requestReceipt = timestamp - (((timeStruct32 >> 20) & 0xF00) | static_cast<uint32_t>(timeStruct8));
    }
  }

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
  return true;
}
