/**
 * @file ColorTable.h
 * The file declares a struct that represents the tabularized color calibration.
 * @author: marcel
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "ColorCalibration.h"

struct ColorTable : public Streamable
{
  struct Colors
  {
    unsigned char colors = 0;

    Colors() = default;
    Colors(unsigned char colors) : colors(colors) {}
    Colors(ColorClasses::Color color) : colors(color == ColorClasses::none ? 0 : static_cast<unsigned char>(1 << (color - 1))) {}

    bool is(ColorClasses::Color color) const
    {
      return (color == ColorClasses::none && !colors) ||
             (1 << (color - 1) & colors) != 0;
    }
  };

private:
  Colors colorTable[32][256][256];

public:
  void fromColorCalibration(const ColorCalibration& colorCalibration, ColorCalibration& prevCalibration);

  const Colors& operator[](const Image::Pixel& pixel) const
  {
    return colorTable[pixel.y >> 3][pixel.cb][pixel.cr];
  }

private:
  void update(const ColorCalibration::HSIRanges& ranges, unsigned char color);
  void update(const ColorCalibration::WhiteThresholds& thresholds, unsigned char color);

  virtual void serialize(In* in, Out* out);
};

inline Out& operator<<(Out& stream, const ColorTable::Colors& color)
{
  STREAM_REGISTER_BEGIN_EXT(color);
  STREAM_EXT(stream, color.colors);
  STREAM_REGISTER_FINISH;
  return stream;
}

inline In& operator>>(In& stream, ColorTable::Colors& color)
{
  STREAM_REGISTER_BEGIN_EXT(color);
  STREAM_EXT(stream, color.colors);
  STREAM_REGISTER_FINISH;
  return stream;
}
