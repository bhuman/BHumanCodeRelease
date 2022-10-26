/**
 * @file BHumanStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanStandardMessage.h"
#include "Platform/BHAssert.h"
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

BHumanStandardMessage::BHumanStandardMessage() :
  version(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
{
  const char* init = BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER;
  for(unsigned int i = 0; i < sizeof(header); ++i)
    header[i] = init[i];
}

int BHumanStandardMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 15, "This method is not adjusted for the current message version");

  return sizeof(header)
         + sizeof(version)
         + sizeof(magicNumber)
         + sizeof(timestamp)
         + sizeof(referenceGameControllerPacketTimestamp)
         + sizeof(referenceGameControllerPacketNumber)
         + sizeof(uint16_t) // size of compressedContainer
         + static_cast<int>(compressedContainer.size());
}

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 15, "This method is not adjusted for the current message version");

  [[maybe_unused]] const void* const begin = data; //just for length check

  for(unsigned i = 0; i < sizeof(header); ++i)
    writeVal<char>(data, header[i]);
  writeVal<uint8_t>(data, version);
  writeVal<uint8_t>(data, magicNumber);
  writeVal<uint32_t>(data, timestamp);

  writeVal<uint32_t>(data, referenceGameControllerPacketTimestamp);
  writeVal<uint8_t>(data, referenceGameControllerPacketNumber);

  ASSERT(compressedContainer.size() < (1ull << 16));
  writeVal<uint16_t>(data, static_cast<uint16_t>(compressedContainer.size()));
  std::memcpy(data, compressedContainer.data(), compressedContainer.size());
  reinterpret_cast<char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 15, "This method is not adjusted for the current message version");

  [[maybe_unused]] const void* const begin = data; //just for length check

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != readVal<const char>(data))
      return false;

  version = readVal<const uint8_t>(data);
  if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  magicNumber = readVal<const uint8_t>(data);
  timestamp = readVal<const uint32_t>(data);

  referenceGameControllerPacketTimestamp = readVal<const uint32_t>(data);
  referenceGameControllerPacketNumber = readVal<const uint8_t>(data);

  const uint16_t compressedContainerSize = readVal<const uint16_t>(data);
  compressedContainer.resize(compressedContainerSize);
  std::memcpy(compressedContainer.data(), data, compressedContainer.size());
  reinterpret_cast<const char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
  return true;
}
