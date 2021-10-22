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
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  return sizeof(header)
         + sizeof(version)
         + sizeof(magicNumber)
         + sizeof(timestamp)
         + sizeof(uint16_t) // size of compressedContainer (9 bits), requestsNTPMessage (1 bit), NTP reply bitset (6 bits)
         + static_cast<int>(ntpMessages.size()) * 5
         + static_cast<int>(compressedContainer.size());
}

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  for(unsigned i = 0; i < sizeof(header); ++i)
    writeVal<char>(data, header[i]);
  writeVal<uint8_t>(data, version);
  writeVal<uint8_t>(data, magicNumber);
  writeVal<uint32_t>(data, timestamp);

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team.");
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
        ASSERT(ntpMessagesItr == ntpMessages.cend() || ntpMessagesItr->receiver > i + 1);
      }
    }
  }
  ASSERT(compressedContainer.size() < (1ull << 9));
  writeVal<uint16_t>(data, static_cast<uint16_t>((compressedContainer.size() << 7) | (requestsNTPMessage ? (1u << 6) : 0u) | (ntpReceivers >> 1)));

  for(const BNTPMessage& ntpMessage : ntpMessages)
  {
    const uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
    writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00) << 20));
    writeVal<uint8_t>(data, requestReceiptDiffCutted & 0xFF);
  }

  std::memcpy(data, compressedContainer.data(), compressedContainer.size());
  reinterpret_cast<char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 13, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  ntpMessages.clear();

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != readVal<const char>(data))
      return false;

  version = readVal<const uint8_t>(data);
  if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
    return false;

  magicNumber = readVal<const uint8_t>(data);

  timestamp = readVal<const uint32_t>(data);

  static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team (but can be easily adjusted).");
  const uint16_t ntpAndSizeContainer = readVal<const uint16_t>(data);
  compressedContainer.resize(ntpAndSizeContainer >> 7);
  requestsNTPMessage = (ntpAndSizeContainer & (1u << 6)) != 0;
  uint16_t runner = 1u << 6;
  for(uint8_t i = 1; runner != 0; ++i)
  {
    if(ntpAndSizeContainer & (runner >>= 1))
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

  std::memcpy(compressedContainer.data(), data, compressedContainer.size());
  reinterpret_cast<const char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
  return true;
}
