/**
 * @file BHumanArbitraryMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanArbitraryMessage.h"
#include "Platform/BHAssert.h"

int BHumanArbitraryMessage::sizeOfArbitraryMessage() const
{
  static_assert(BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

  OutBinarySize sizeStream;
  sizeStream << queue;

  ASSERT(sizeStream.getSize() == queue.getStreamedSize());
  return static_cast<int>(sizeof header + sizeof version + sizeStream.getSize());
}

void BHumanArbitraryMessage::write(void* data)
{
  static_assert(BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

  for(unsigned i = 0; i < sizeof(header); ++i)
    *reinterpret_cast<char*&>(data)++ = header[i];

  *reinterpret_cast<uint8_t*&>(data)++ = version;

  OutBinaryMemory memory(data);
  memory << queue;
}

bool BHumanArbitraryMessage::read(const void* data)
{
  static_assert(BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");
  queue.clear();

  for(unsigned i = 0; i < sizeof(header); ++i)
    if(header[i] != *reinterpret_cast<const char*&>(data)++)
      return false;

  version = *reinterpret_cast<const uint8_t*&>(data)++;

  if(version != BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION)
    return false;

  InBinaryMemory memory(data);
  memory >> queue;

  return true;
}

void BHumanArbitraryMessage::serialize(In* in, Out* out)
{
  static_assert(BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION == 0, "This method is not adjusted for the current message version");

  if(queue.isEmpty())
  {
    queue.out.bin << 5;
    queue.out.finishMessage(idRobot);
  }

  STREAM_REGISTER_BEGIN;
  std::string headerRef(header, 4);
  STREAM(headerRef);// does not allow to change the header in any case, but makes it visble in a great way
  STREAM(version);

  //TODO steam messageque

  STREAM_REGISTER_FINISH;
}
