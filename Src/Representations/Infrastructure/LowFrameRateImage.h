/**
 * @file LowFrameRateImage.h
 *
 * Declaration of struct LowFrameRateImage
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "CameraImage.h"

struct LowFrameRateImage : public Streamable
{
  CameraImage image;
  bool imageUpdated = false; /**< True if image was updated this frame */

protected:
  void serialize(In* in, Out* out) override
  {
    STREAM(imageUpdated);
    if(imageUpdated)
      STREAM(image);
  }

private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(LowFrameRateImage);
    REG(imageUpdated);
    REG(image);
  }
};
