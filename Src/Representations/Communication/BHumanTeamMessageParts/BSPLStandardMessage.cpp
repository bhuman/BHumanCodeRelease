/**
 * @file BSPLStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */
#include "BSPLStandardMessage.h"
#include "Platform/BHAssert.h"
#include <cstddef>

size_t BSPLStandardMessage::sizeOfBSPLMessage() const
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 6, "Please adjust this file to the newer version.");

  return offsetof(RoboCup::SPLStandardMessage, data);
}

void BSPLStandardMessage::write(void* data) const
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 6, "Please adjust this file to the newer version.");

  memcpy(data, &header, sizeOfBSPLMessage());
}

bool BSPLStandardMessage::read(const void* data)
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 6, "Please adjust this file to the newer version.");

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != *reinterpret_cast<const char*&>(data)++)
      return false;

  version = *reinterpret_cast<const uint8_t*&>(data)++;

  if(version != SPL_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  memcpy(reinterpret_cast<void*>(&playerNum), data, sizeOfBSPLMessage() - (sizeof(header) + sizeof(version)));

  return true;
}

void BSPLStandardMessage::serialize(In* in, Out* out)
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 6, "Please adjust this file to the newer version.");

  STREAM_REGISTER_BEGIN;
  std::string headerRef(header, sizeof(header));
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visble in a greate way
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
  STREAM_REGISTER_FINISH;
}
