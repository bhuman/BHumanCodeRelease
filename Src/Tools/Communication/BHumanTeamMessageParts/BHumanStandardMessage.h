/**
 * @file BHumanStandardMessage.h
 *
 * The file declares the B-Human standard message.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/InOut.h"
#include <cstdint>
#include <vector>

#define BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER  "BHUM"
#define BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION 15      /**< This should be incremented with each change. */

STREAMABLE(BHumanStandardMessage,
{
  In* in = nullptr; /**< A stream where data can be read from the compressed part. */
  Out* out = nullptr; /**< A stream where data can be written to the compressed part. */

  /**
   * Returns the size of this struct when it is written.
   * @return The size of ...
   */
  int sizeOfBHumanMessage() const;

  /**
   * Converts this struct for communication usage.
   * @param data Pointer to dataspace,
   *        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
   * -asserts: writing sizeOfBHumanMessage() bytes
   */
  void write(void* data) const;

  /**
   * Reads the message from data.
   * @param data The message.
   * @return Whether the header and the versions are convertible.
   */
  bool read(const void* data);

  /** Constructor. */
  BHumanStandardMessage(),

  (char[4]) header,         /**< BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER */
  (uint8_t) version,        /**< BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION */
  (uint8_t)(0) magicNumber, /**< The magic number. */
  (unsigned)(0) timestamp,  /**< Timestamp when this message has been sent (relative to the clock frame of the sending robot). */

  (unsigned)(0) referenceGameControllerPacketTimestamp, /**< The timestamp when a specific GameController packet was received. */
  (uint8_t)(0) referenceGameControllerPacketNumber, /**< The packet number of the above GameController packet. */

  (std::vector<uint8_t>) compressedContainer, /**< The container for the compressed data. */
});
