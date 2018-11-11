/**
 * @file Thumbnail.h
 *
 * Declares a representation containing a downscaled version of the camera image
 * to be stored in logs.
 *
 * @author Alexis Tsogias
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 */

#pragma once

#include "Image.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Tools/ImageProcessing/TImage.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Debugging/DebugImages.h"

struct Thumbnail : public Streamable
{
  ENUM(Mode,
  {,
    yuv,
    grayscale,
    grayscaleWithColorClasses,
  });

  /**
   * A grayscale thumbnail image, downscaled by the factor given by `scale`.
   */
  TImage<PixelTypes::GrayscaledPixel> imageY;

  /**
   * The U and V channels of a colored thumbnail image combined into one image, downscaled by `scale * 2`.
   * Only contains valid data if `grayscale` is false.
   */
  TImage<unsigned short> imageUV;
  TImage<PixelTypes::YUYVPixel>* imageYUYV = nullptr;

  unsigned int scale;
  Mode mode;

  void toYUYV(TImage<PixelTypes::YUYVPixel>& dest) const;
  void toECImage(ECImage& dest) const;
  void toImage(Image& dest) const;

  Thumbnail() = default;

  ~Thumbnail()
  {
    if(imageYUYV != nullptr)
      delete imageYUYV;
  }

  void draw()
  {
    SEND_DEBUG_IMAGE("ThumbnailGrayscale", imageY);
    COMPLEX_IMAGE("ThumbnailColored")
    {
      if(imageYUYV == nullptr)
        imageYUYV = new TImage<PixelTypes::YUYVPixel>();
      toYUYV(*imageYUYV);
      SEND_DEBUG_IMAGE("ThumbnailColored", *imageYUYV);
    }
  };

protected:
  void serialize(In* in, Out* out) override
  {
    STREAM(mode);
    STREAM(scale);
    STREAM(imageY);
    if(mode == yuv)
      STREAM(imageUV);
  }

private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(Thumbnail);
    REG(mode);
    REG(scale);
    REG(imageY);
    REG(imageUV);
  }
};
