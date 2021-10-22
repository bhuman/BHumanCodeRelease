/**
 * The file declares a specific ring buffer for buffering SPLStandardMessages
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Platform/Memory.h"
#include "Tools/Communication/RoboCupGameControlData.h"

#include <algorithm>

struct SPLStandardMessageBufferEntry
{
  unsigned timestamp; /**< The timestamp when the message was received. */
  RoboCup::SPLStandardMessage message; /**< The received message. */
};

template<std::size_t capacity = 0> class SPLStandardMessageBuffer
{
private:
  SPLStandardMessageBufferEntry* buffer; /**< Stores the elements of the buffer. */
  std::size_t head = 0; /**< The next entry that will be used for push_front(). */
  std::size_t entries = 0; /**< The number of entries in the buffer. */

public:
  SPLStandardMessageBuffer() :
    buffer(reinterpret_cast<SPLStandardMessageBufferEntry*>(Memory::alignedMalloc(capacity * sizeof(SPLStandardMessageBufferEntry))))
  { static_assert(capacity, "A capacity of zero is nonsense, pls fix it!"); }

  ~SPLStandardMessageBuffer() { Memory::alignedFree(reinterpret_cast<char*>(buffer)); }

  std::size_t size() const { return entries; }
  bool empty() const { return entries == 0; }

  /**
   * Sets the head forward and returns the pointer to this element.
   *
   * If the ringbuffer was full, this will indirectly remove the last element.
   */
  SPLStandardMessageBufferEntry* setForward()
  {
    entries = std::min(capacity, ++entries);
    (++head) %= capacity;
    return &buffer[head];
  }

  /**
   * Sets the head backward
   */
  void removeFront()
  {
    --entries;
    head = (head + capacity - 1) % capacity;
  }

  /** Reduces the entry count and returns the pointer to the message that "drops off" */
  SPLStandardMessageBufferEntry* takeBack()
  {
    ASSERT(!empty());
    return &buffer[(capacity + head - --entries) % capacity];
  }
};
