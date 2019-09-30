/**
 * @file FrameInfo.h
 * The file declares a struct that contains information on the current frame.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct FrameInfo
 * A struct that contains information on the current frame.
 */
STREAMABLE(FrameInfo,
{
  /**
   * The method returns the time difference between a given timestamp and the
   * current frame time.
   * @param timestamp A timestamp, usually in the past.
   * @return The number of ms passed since the given timestamp.
   */
  int getTimeSince(unsigned timestamp) const,

  (unsigned)(0) time, /**< The timestamp of the data processed in the current frame in ms. */
});

inline int FrameInfo::getTimeSince(unsigned timestamp) const
{
  return static_cast<int>(time - timestamp);
}

STREAMABLE_WITH_BASE(CognitionFrameInfo, FrameInfo, {,});
