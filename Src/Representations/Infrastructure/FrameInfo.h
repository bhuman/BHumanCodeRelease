/**
* @file FrameInfo.h
* The file declares a class that contains information on the current frame.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
* @class FrameInfo
* A class that contains information on the current frame.
*/
STREAMABLE(FrameInfo,
{
public:
  /**
  * The method returns the time difference between a given time stamp and the
  * current frame time.
  * @param timeStamp A time stamp, usually in the past.
  * @return The number of ms passed since the given time stamp.
  */
  int getTimeSince(unsigned timeStamp) const {return int(time - timeStamp);},

  (unsigned)(0) time, /**< The time stamp of the data processed in the current frame in ms. */
  (float)(1) cycleTime, /**< Length of one cycle in seconds. */
});

/**
* @class CognitionFrameInfo
* A class that contains information on the current Cognition frame.
* This representation is used to track whether camera images are
* received. In contrast to FrameInfo, it will be transfered to the
* Motion process.
*/
class CognitionFrameInfo : public FrameInfo {};
