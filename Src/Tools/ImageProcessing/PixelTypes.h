/**
 * @author Felix Thielke
 */

#pragma once

#include "Representations/Configuration/FieldColors.h"
#include "Tools/Streams/Enum.h"

namespace PixelTypes
{
  GLOBAL_ENUM(PixelType,
  {,
    RGB,       // useful for DebugImages?
    BGRA,      // the format that QImage uses
    YUYV,      // the format the NaoCamera supplies
    YUV,       // useful for DebugImages?
    Colored,   // format of the colored image in ECImage
    Grayscale, // format of the grayscaled image in ECImage
  });

  struct RGBPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char padding;
      };
    };
  };

  struct BGRAPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char b;
        unsigned char g;
        unsigned char r;
        unsigned char a;
      };
    };
  };

  struct YUYVPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char y0;
        unsigned char u;
        unsigned char y1;
        unsigned char v;
      };
    };

    inline unsigned char& y(const size_t x)
    {
      return (reinterpret_cast<unsigned char*>(&color)[(x & 1) << 1]);
    }

    inline unsigned char y(const size_t x) const
    {
      return (reinterpret_cast<const unsigned char*>(&color)[(x & 1) << 1]);
    }
  };

  struct YUVPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char padding;
        unsigned char u;
        unsigned char y;
        unsigned char v;
      };
    };
  };

  struct HSIPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char h;
        unsigned char s;
        unsigned char i;
        unsigned char padding;
      };
    };
  };

  using GrayscaledPixel = unsigned char;
  using ColoredPixel = FieldColors::Color;

  constexpr size_t pixelSize(const PixelType type)
  {
    return type == RGB ? sizeof(RGBPixel)
           : (type == BGRA ? sizeof(BGRAPixel)
              : (type == YUYV ? sizeof(YUYVPixel)
                 : (type == YUV ? sizeof(YUVPixel)
                    : (type == Grayscale ? sizeof(GrayscaledPixel)
                       : (type == Colored ? sizeof(ColoredPixel)
                          : 0)))));
  }
}
