/**
 * @file BSPLStandardMessage.h
 *
 * The file integrate the SPLStandardMessage into the B-Human system.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Representations/Communication/RoboCupGameControlData.h"
#include "Tools/Streams/Streamable.h"

//THIS STRUCT IS INDENTED TO NOT USE RoboCup::SPLStandardMessage.data!
struct BSPLStandardMessage : public RoboCup::SPLStandardMessage, public Streamable
{
  static_assert(SPL_STANDARD_MESSAGE_STRUCT_VERSION == 7, "Please adjust this file to the newer version.");

  BSPLStandardMessage() : RoboCup::SPLStandardMessage(), Streamable()
  {
    numOfDataBytes = 0;
    // AlignedMemory::operator delete[](data); //FIXME make this correct
  }

  // returns the size of this struct when it is written
  size_t sizeOfBSPLMessage() const;

  // Method to convert this struct for communication usage
  // @param data point to dataspace,
  //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBSPLMessage()
  // -asserts: writing sizeOfBHMessage() bytes
  void write(void* data) const;

  // Method to reads the message from data.
  // @param data the message
  // @return the header and the versions are convertible
  bool read(const void* data);

protected:
  void serialize(In* in, Out* out);

private:
  static void reg();

  using RoboCup::SPLStandardMessage::data; //hide because it should not be used
};
