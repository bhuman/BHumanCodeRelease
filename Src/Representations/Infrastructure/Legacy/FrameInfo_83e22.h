/**
* @file FrameInfo_83e22.h
* The file declares a class that contains information on the current frame.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

#include "Tools/Streams/Streamable.h"

/**
* @class FrameInfo_83e22
* A class that contains information on the current frame.
*/
class FrameInfo_83e22 : public Streamable
{
private:
  /**
  * The method makes the object streamable.
  * @param in The stream from which the object is read (if in != 0).
  * @param out The stream to which the object is written (if out != 0).
  */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(time);
    STREAM_REGISTER_FINISH;
  }

public:
  /**
  * Default constructor.
  */
  FrameInfo_83e22() : time(0) {}

  /**
  * The method returns the time difference between a given time stamp and the
  * current frame time.
  * @param timeStamp A time stamp, usually in the past.
  * @return The number of ms passed since the given time stamp.
  */
  int getTimeSince(unsigned timeStamp) const {return int(time - timeStamp);}

  unsigned time; /**< The time stamp of the data processed in the current frame in ms. */
};

/**
* @class CognitionFrameInfo_83e22
* A class that contains information on the current Cognition frame.
* This representation is used to track whether camera images are
* received. In contrast to FrameInfo_83e22, it will be transfered to the
* Motion process.
*/
class CognitionFrameInfo_83e22 : public FrameInfo_83e22 {};
