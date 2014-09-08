/**
 * @file Image.h
 *
 * Declaration of class Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Image.h"

class LowFrameRateImage : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
      STREAM(imageUpdated);
      if(imageUpdated)
      {
        STREAM(image);
      }
    STREAM_REGISTER_FINISH;
  }

  Image image;
  bool imageUpdated; /**< True if image was updated this frame */
};
