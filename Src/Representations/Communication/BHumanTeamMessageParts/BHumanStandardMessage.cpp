/**
 * @file BHumanStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

int BHumanStandardMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 3, "This method is not adjusted for the current message version");

  return sizeof(header)
         + sizeof(version)
         + sizeof(magicNumber)
         + sizeof(ballTimeWhenDisappearedSeenPercentage)
         + sizeof(ballLastPerceptX)
         + sizeof(ballLastPerceptY)
         + sizeof(ballCovariance)
         + sizeof(robotPoseDeviation)
         + sizeof(robotPoseCovariance)
         + sizeof(robotPoseValidity);
}

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 3, "This method is not adjusted for the current message version");

#ifndef NDEBUG
  const void* const begin = data; //just for length check
#endif // NDEBUG

  for(unsigned i = 0; i < sizeof(header); ++i)
    *reinterpret_cast<char*&>(data)++ = header[i];

  *reinterpret_cast<uint8_t*&>(data)++ = version;
  *reinterpret_cast<int8_t*&>(data)++ = magicNumber;

  *reinterpret_cast<uint32_t*&>(data)++ = ballTimeWhenDisappearedSeenPercentage;

  *reinterpret_cast<int16_t*&>(data)++ = ballLastPerceptX;
  *reinterpret_cast<int16_t*&>(data)++ = ballLastPerceptY;

  *reinterpret_cast<std::array<float, 3>*&>(data)++ = ballCovariance;
  *reinterpret_cast<float*&>(data)++ = robotPoseDeviation;

  *reinterpret_cast<std::array<float, 6>*&>(data)++ = robotPoseCovariance;
  *reinterpret_cast<uint8_t*&>(data)++ = robotPoseValidity;

  ASSERT((reinterpret_cast<char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 3, "This method is not adjusted for the current message version");

  for(size_t i = 0; i < sizeof(header); ++i)
    if(header[i] != *reinterpret_cast<const char*&>(data)++)
      return false;

  version = *reinterpret_cast<const uint8_t*&>(data)++;
  if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  magicNumber = *reinterpret_cast<const uint8_t*&>(data)++;
  if(!(Global::settingsExist() && Global::getSettings().magicNumber))
    return false;

  ballTimeWhenDisappearedSeenPercentage = *reinterpret_cast<const uint32_t*&>(data)++;

  ballLastPerceptX = *reinterpret_cast<const int16_t*&>(data)++;
  ballLastPerceptY = *reinterpret_cast<const int16_t*&>(data)++;

  ballCovariance = *reinterpret_cast<const std::array<float, 3>*&>(data)++;
  robotPoseDeviation = *reinterpret_cast<const float*&>(data)++;

  robotPoseCovariance = *reinterpret_cast<const std::array<float, 6>*&>(data)++;
  robotPoseValidity = *reinterpret_cast<const uint8_t*&>(data)++;

  return true;
}
