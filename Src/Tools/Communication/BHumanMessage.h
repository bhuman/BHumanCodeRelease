/**
 * @file BHumanMessage.h
 *
 * The file declares the B-Human team message.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/InOut.h"
#include <cstdint>
#include <limits>
#include <vector>

#define BHUMAN_MESSAGE_STRUCT_VERSION 72      /**< This should be incremented with each change. */

/**
 * A representation of team message a B-Human robot would send.
 */
STREAMABLE(BHumanMessage,
{
  In* in = nullptr; /**< A stream where data can be read from the compressed part. */
  Out* out = nullptr; /**< A stream where data can be written to the compressed part. */

  /**
   * Returns the size of this struct when it is written.
   * @return The size of ...
   */
  size_t sizeOfBHumanMessage() const;

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
   * @param size The number of available bytes in the message.
   * @return Whether the header and the versions are convertible.
   */
  bool read(const void* data, size_t size),

  (uint8_t)(0) playerNumber, /**< The number of the sending player. */
  (unsigned)(0) timestamp, /**< Timestamp when this message has been sent (relative to the clock frame of the sending robot). */

  (uint16_t)(std::numeric_limits<uint16_t>::max()) referenceGameControllerPacketTimestampOffset, /**< The timestamp when a specific GameController packet was received, coded as offset to the timestamp above. */
  (uint8_t)(0) referenceGameControllerPacketNumber, /**< The packet number of the above GameController packet. */

  (std::vector<uint8_t>) compressedContainer, /**< The container for the compressed data. */
});
