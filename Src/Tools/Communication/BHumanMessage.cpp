/**
 * @file BHumanMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanMessage.h"
#include "Framework/Settings.h"
#include "Platform/BHAssert.h"
#include "Streaming/Global.h"
#include <cstring>

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

size_t BHumanMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_MESSAGE_STRUCT_VERSION == 72, "This method is not adjusted for the current message version");

  return sizeof(uint8_t) // version
         + sizeof(playerNumber)
         + 3 // timestamp
         + sizeof(referenceGameControllerPacketTimestampOffset)
         + sizeof(referenceGameControllerPacketNumber)
         + compressedContainer.size();
}

void BHumanMessage::write(void* data) const
{
  static_assert(BHUMAN_MESSAGE_STRUCT_VERSION == 72, "This method is not adjusted for the current message version");

  [[maybe_unused]] const void* const begin = data; //just for length check

  writeVal<uint8_t>(data, BHUMAN_MESSAGE_STRUCT_VERSION ^ Global::getSettings().magicNumber);
  writeVal<uint8_t>(data, playerNumber);
  writeVal<uint16_t>(data, static_cast<uint16_t>(timestamp));
  writeVal<uint8_t>(data, static_cast<uint8_t>(timestamp >> 16));

  writeVal<uint16_t>(data, referenceGameControllerPacketTimestampOffset);
  writeVal<uint8_t>(data, referenceGameControllerPacketNumber);

  std::memcpy(data, compressedContainer.data(), compressedContainer.size());
  reinterpret_cast<char*&>(data) += compressedContainer.size();

  ASSERT(static_cast<size_t>(reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanMessage::read(const void* data, size_t size)
{
  static_assert(BHUMAN_MESSAGE_STRUCT_VERSION == 72, "This method is not adjusted for the current message version");

  compressedContainer.clear();
  if(size < sizeOfBHumanMessage())
    return false;

  [[maybe_unused]] const void* const begin = data; //just for length check

  if(readVal<const uint8_t>(data) != (BHUMAN_MESSAGE_STRUCT_VERSION ^ Global::getSettings().magicNumber))
    return false;

  playerNumber = readVal<const uint8_t>(data);

  timestamp = readVal<const uint16_t>(data);
  timestamp |= readVal<const uint8_t>(data) << 16;

  referenceGameControllerPacketTimestampOffset = readVal<const uint16_t>(data);
  referenceGameControllerPacketNumber = readVal<const uint8_t>(data);

  compressedContainer.resize(size - sizeOfBHumanMessage());
  std::memcpy(compressedContainer.data(), data, compressedContainer.size());
  reinterpret_cast<const char*&>(data) += compressedContainer.size();

  ASSERT(static_cast<size_t>(reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
  return true;
}
