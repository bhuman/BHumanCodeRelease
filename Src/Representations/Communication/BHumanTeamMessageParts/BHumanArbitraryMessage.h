/**
 * @file BHumanArbitraryMessage.h
 *
 * The file representates the B-Human arbitrary team message part.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include <SPLStandardMessage.h>
#include <cstdint>

#pragma once

#define BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER  "BHUA"
#define BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION 0 // this should be incremented with each change

struct BHumanArbitraryMessage : public Streamable
{
  char header[4];
  uint8_t version;
  MessageQueue queue;

  // returns the size of this struct when it is written
  int sizeOfArbitraryMessage() const;

  // Method to convert this struct for communication usage
  // @param data point to dataspace,
  //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
  // -asserts: writing sizeOfArbitraryMessage() bytes
  void write(void* data);

  // Method to reads the message from data.
  // @param data the message
  // @return the header and the versions are convertible
  bool read(const void* data);

  BHumanArbitraryMessage()
  {
    const char* init = BHUMAN_ARBITRARY_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];

    version = BHUMAN_ARBITRARY_MESSAGE_STRUCT_VERSION;

    queue.setSize(SPL_STANDARD_MESSAGE_DATA_SIZE);
  };

protected:
  void serialize(In* in, Out* out) override;

private:
  static void reg();
};
