/**
 * @file BHumanStandardMessage.h
 *
 * The file declares the B-Human standard message.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include <stdint.h>

#define BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER  "BHUM"
#define BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION 3 // this should be incremented with each change

STREAMABLE(BHumanStandardMessage,
{
  // returns the size of this struct when it is written
  int sizeOfBHumanMessage() const;

  // Method to convert this struct for communication usage
  // @param data point to dataspace,
  //        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
  // -asserts: writing sizeOfBHMessage() bytes
  void write(void* data) const;

  // Method to reads the message from data.
  // @param data the message
  // @return the header and the versions are convertible
  bool read(const void* data);

  BHumanStandardMessage()
  {
    const char* init = BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER;
    for(unsigned int i = 0; i < sizeof(header); ++i)
      header[i] = init[i];
  },

  (char[4]) header,
  (uint8_t)(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION) version,
  (uint8_t) magicNumber,

  (uint32_t)(0u) ballTimeWhenDisappearedSeenPercentage,

  (int16_t) ballLastPerceptX,
  (int16_t) ballLastPerceptY,
  (std::array<float, 3>) ballCovariance,
  (float) robotPoseDeviation,
  (std::array<float, 6>) robotPoseCovariance,
  (uint8_t) robotPoseValidity,
});
