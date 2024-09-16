/**
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "ImageProcessing/ColorModelConversions.h"
#include "Streaming/Enum.h"
#include <vector>

namespace PixelTypes
{
  ENUM(PixelType,
  {,
    RGB,              // useful for DebugImages?
    BGRA,             // the format that QImage uses
    YUYV,             // the format the NaoCamera supplies
    YUV,              // useful for DebugImages?
    Grayscale,        // format of the gray-scaled image in ECImage
    Hue,              // hue channel of a YHS image
    Binary,           //
    Edge2,            //
    Edge2MonoAvg,     //
    Edge2MonoAbsAvg,  //
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
    RGBPixel() = default;
    RGBPixel(unsigned int color) : color(color) {}
    RGBPixel(unsigned char r, unsigned char g, unsigned char b, unsigned char padding = 255) : r(r), g(g), b(b), padding(padding) {}
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

    BGRAPixel() = default;
    BGRAPixel(unsigned int color) : color(color) {}
    BGRAPixel(unsigned char b, unsigned char g, unsigned char r, unsigned char a = 255) : b(b), g(g), r(r), a(a) {}
    static unsigned numPixel() {return 1;}
    std::vector<unsigned char> raw() const {return std::vector<unsigned char>{b, g, r};}
    std::vector<unsigned char> rgb() const {return std::vector<unsigned char>{r, g, b};}
    std::vector<unsigned char> grayscale() const {return std::vector<unsigned char>{static_cast<unsigned char>((r + g + b) / 3)};}
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

    YUYVPixel() = default;
    YUYVPixel(unsigned int color) : color(color) {}
    YUYVPixel(unsigned char y0, unsigned char u, unsigned char y1, unsigned char v) : y0(y0), u(u), y1(y1), v(v) {}

    static unsigned numPixel() {return 2;}
    std::vector<unsigned char> raw() const {return std::vector<unsigned char>{y0, u, v, y1, u, v};}
    std::vector<unsigned char> rgb() const
    {
      std::vector<unsigned char> ret(6);
      ColorModelConversions::fromYUVToRGB(y0, u, v, ret[0], ret[1], ret[2]);
      ColorModelConversions::fromYUVToRGB(y1, u, v, ret[3], ret[4], ret[5]);
      return ret;
    }
    std::vector<unsigned char> grayscale() const {return std::vector<unsigned char>{y0, y1};}
    unsigned char& y(const size_t x) {return (reinterpret_cast<unsigned char*>(&color)[(x & 1) << 1]);}
    unsigned char y(const size_t x) const {return (reinterpret_cast<const unsigned char*>(&color)[(x & 1) << 1]);}
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

    YUVPixel() = default;
    YUVPixel(unsigned int color) : color(color) {}
    YUVPixel(unsigned char y, unsigned char u, unsigned char v, unsigned char padding = 0) : padding(padding), u(u), y(y), v(v) {}
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

    HSIPixel() = default;
    HSIPixel(unsigned int color) : color(color) {}
    HSIPixel(unsigned char h, unsigned char s, unsigned char i, unsigned char padding = 0) : h(h), s(s), i(i), padding(padding) {}
  };

  using GrayscaledPixel = unsigned char;

  class HuePixel
  {
  private:
    unsigned char value = 0;

  public:
    constexpr HuePixel() = default;
    constexpr HuePixel(unsigned char hue) : value(hue) {}

    operator unsigned char& () { return value; }
    constexpr operator const unsigned char& () const { return value; }

    constexpr HuePixel operator-() const { return -value; }
    HuePixel& operator+=(unsigned char hue) { value += hue; return *this; }
    HuePixel& operator-=(unsigned char hue) { value -= hue; return *this; }
    HuePixel& operator*=(unsigned char hue) { value *= hue; return *this; }
    HuePixel& operator/=(unsigned char hue) { value /= hue; return *this; }
  };

  using BinaryPixel = bool;

  struct Edge2Pixel
  {
    unsigned char filterX;
    unsigned char filterY;

    Edge2Pixel() = default;
    Edge2Pixel(unsigned char filterX, unsigned char filterY) : filterX(filterX), filterY(filterY) {}
  };

  constexpr size_t pixelSize(const PixelType type)
  {
    return type == RGB ? sizeof(RGBPixel)
           : (type == BGRA ? sizeof(BGRAPixel)
              : (type == YUYV ? sizeof(YUYVPixel)
                 : (type == YUV ? sizeof(YUVPixel)
                    : (type == Grayscale ? sizeof(GrayscaledPixel)
                        : (type == Hue ? sizeof(HuePixel)
                           : ((type == Edge2 || type == Edge2MonoAvg || type == Edge2MonoAbsAvg) ? sizeof(Edge2Pixel)
                              : (type == Binary ? sizeof(BinaryPixel)
                                 : 0)))))));
  }
}
