/**
 * @file FrameInfo.h
 * The file declares a struct that contains information on the current frame.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
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
   * The method returns the time difference between a given time stamp and the
   * current frame time.
   * @param timeStamp A time stamp, usually in the past.
   * @return The number of ms passed since the given time stamp.
   */
  int getTimeSince(unsigned timeStamp) const;
  ,
  (unsigned)(0) time, /**< The time stamp of the data processed in the current frame in ms. */
  (float)(1) cycleTime, /**< Length of one cycle in seconds. */
});

inline int FrameInfo::getTimeSince(unsigned timeStamp) const
{
  return static_cast<int>(time - timeStamp);
}