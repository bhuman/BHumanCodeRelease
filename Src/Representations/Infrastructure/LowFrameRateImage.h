/**
 * @file Image.h
 *
 * Declaration of struct Image
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Image.h"

struct LowFrameRateImage : public Streamable
{
  Image image;
  bool imageUpdated = false; /**< True if image was updated this frame */

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
};
