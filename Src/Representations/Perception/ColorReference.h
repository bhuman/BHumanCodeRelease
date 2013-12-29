/*
 * @author: marcel
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Asserts.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/ColorClasses.h"
#include "Tools/Range.h"

class ColorReference : public Streamable
{
friend class ColorProvider;
friend class ImageWidget;
friend class RangeSelector;
friend class ThresholdSelector;
friend class ColorCalibrationWidget;

public:
  static const unsigned char shiftFactor = 1;
  static const unsigned char colorTableSize = 256 >> shiftFactor;

  /**
   * This class describes thresholds for a color.
   * first, second and third are the typical component of a color,
   * eg. first = r, second = g, third = b
   * or  first = y, second = cb,third = cr
   *
   * thresholds are all inclusive
   */
  template <class T=int>
  STREAMABLE(ColorThreshold,
  {,
    (T)(0) first,
    (T)(0) second,
    (T)(0) third,
  });

  /**
   * Defines a full configuration of a color. All values are inclusive.
   */
  STREAMABLE(HSVColorDefinition,
  {,
    (Range<>)(0) hue,
    (Range<>)(0) saturation,
    (Range<>)(0) value,
  });

  struct MultiColor
  {
    unsigned char colors;

    MultiColor() {}
    MultiColor(unsigned char colors) : colors(colors) {}

    bool operator==(ColorClasses::Color color) const
    {
      return (color == ColorClasses::none && colors == 0) ||
             (color != ColorClasses::none && (colors & 1 << (color - 1)) != 0);
    }

    bool operator!=(ColorClasses::Color color) const
    {
      return !(*this == color);
    }

    bool isNone() const
    {
      return colors == none;
    }

    bool isOrange() const
    {
      return (colors & orange) != 0;
    }

    bool isYellow() const
    {
      return (colors & yellow) != 0;
    }

    bool isBlue() const
    {
      return (colors & blue) != 0;
    }

    bool isWhite() const
    {
      return (colors & white) != 0;
    }

    bool isGreen() const
    {
      return (colors & green) != 0;
    }

    bool isBlack() const
    {
      return (colors & black) != 0;
    }

    bool isRed() const

    {
      return (colors & red) != 0;
    }
  };

  bool isNone(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isNone();
  }

  bool isOrange(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isOrange();
  }

  bool isYellow(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isYellow();
  }

  bool isBlue(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isBlue();
  }

  bool isWhite(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isWhite();
  }

  bool isGreen(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isGreen();
  }

  bool isBlack(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isBlack();
  }

  bool isRed(const Image::Pixel* pixel) const
  {
    return getColorClasses(pixel).isRed();
  }

  ColorReference::MultiColor getColorClasses(const Image::Pixel* pixel) const;

private:
  static const unsigned char none = 0;
  static const unsigned char orange = 1 << (ColorClasses::orange - 1);
  static const unsigned char yellow = 1 << (ColorClasses::yellow - 1);
  static const unsigned char blue = 1 << (ColorClasses::blue - 1);
  static const unsigned char white = 1 << (ColorClasses::white - 1);
  static const unsigned char green = 1 << (ColorClasses::green - 1);
  static const unsigned char black = 1 << (ColorClasses::black - 1);
  static const unsigned char red = 1 << (ColorClasses::red - 1);

  // color table
  unsigned char colorTable[colorTableSize][colorTableSize][colorTableSize];
  bool changed;

  // thresholds for color
  HSVColorDefinition thresholdGreen;
  HSVColorDefinition thresholdYellow;
  HSVColorDefinition thresholdOrange;
  HSVColorDefinition thresholdRed;
  HSVColorDefinition thresholdBlue;
  ColorThreshold<int> thresholdWhite; // minR, minB, minRB
  ColorThreshold<int> thresholdBlack; // cb, cr, maxY

  ColorReference::MultiColor getColorClassesFromHSI(const Image::Pixel& pixel) const;
  void update();
  virtual void serialize(In* in, Out* out);
};