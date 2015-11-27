/**
 * @file ColorTable.h
 * The file implements a struct that represents the tabularized color calibration.
 * @author: marcel
 */

#include "ColorTable.h"
#include "ColorCalibration.h"
#include "Platform/BHAssert.h"
#include "Tools/ColorModelConversions.h"
#include <snappy-c.h>

/**
 * Tables that map YCbCr color values to HSI or RB color values.
 * The resolution of this table is the one used by the ColorCalibration struct.
 */
static const struct ColorSpaceMapper
{
  struct RB
  {
    unsigned char r;
    unsigned char b;
    unsigned short rb;
  };

  Image::Pixel hsi[32][256][256];
  RB rb[32][256][256];

  ColorSpaceMapper()
  {
    Image::Pixel* p = &hsi[0][0][0];
    RB* q = &rb[0][0][0];
    unsigned char g;
    for(int y = 7; y < 256; y += 8)
      for(int cb = 0; cb < 256; ++cb)
        for(int cr = 0; cr < 256; ++cr, ++p, ++q)
        {
          ColorModelConversions::fromYCbCrToHSI(static_cast<unsigned char>(y),
                                                static_cast<unsigned char>(cb),
                                                static_cast<unsigned char>(cr),
                                                p->h, p->s, p->i);
          ColorModelConversions::fromYCbCrToRGB(static_cast<unsigned char>(y),
                                                static_cast<unsigned char>(cb),
                                                static_cast<unsigned char>(cr),
                                                q->r, g, q->b);
          q->rb = static_cast<unsigned short>(q->r) + q->b;
        }
  }
} colorSpaceMapper;

void ColorTable::fromColorCalibration(const ColorCalibration& colorCalibration, ColorCalibration& prevCalibration)
{
  bool greenChanged = colorCalibration.ranges[ColorClasses::green - 2] != prevCalibration.ranges[ColorClasses::green - 2];
  for(unsigned i = 0; i < ColorClasses::numOfColors - 2; ++i)
    if(colorCalibration.ranges[i] != prevCalibration.ranges[i])
    {
      update(colorCalibration.ranges[i], static_cast<unsigned char>(1 << (i + 1)));
      prevCalibration.ranges[i] = colorCalibration.ranges[i];
    }

  if(colorCalibration.white != prevCalibration.white || greenChanged)
  {
    update(colorCalibration.white, 1 << (ColorClasses::white - 1));
    prevCalibration.white = colorCalibration.white;
  }
}

void ColorTable::update(const ColorCalibration::HSIRanges& ranges, unsigned char color)
{
  Colors* dest = &colorTable[0][0][0];
  for(const Image::Pixel* src = &colorSpaceMapper.hsi[0][0][0],
                        * end = &colorSpaceMapper.hsi[32][0][0];
      src < end; ++src, ++dest)
    if(ranges.hue.isInside(src->h) &&
       ranges.saturation.isInside(src->s) &&
       ranges.intensity.isInside(src->i))
      dest->colors |= color;
    else
      dest->colors &= ~color;
}

void ColorTable::update(const ColorCalibration::WhiteThresholds& thresholds, unsigned char color)
{
  Colors* dest = &colorTable[0][0][0];
  for(const ColorSpaceMapper::RB* src = &colorSpaceMapper.rb[0][0][0],
                                * end = &colorSpaceMapper.rb[32][0][0];
      src < end; ++src, ++dest)
    if(src->r >= thresholds.minR &&
       src->b >= thresholds.minB &&
       src->rb >= thresholds.minRB &&
       !(dest->colors & 1 << (ColorClasses::green - 1)))
      dest->colors |= color;
    else
      dest->colors &= ~color;
}

void ColorTable::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN;
  size_t ctUncompressedSize = sizeof(colorTable);
  size_t ctCompressedSize = 0;
  std::vector<char> ctCompressed;

  if(out)
  {
    ctCompressedSize = snappy_max_compressed_length(ctUncompressedSize);
    ctCompressed.resize(ctCompressedSize);
    VERIFY(snappy_compress(reinterpret_cast<char*>(&colorTable[0][0][0]), ctUncompressedSize, ctCompressed.data(), &ctCompressedSize) == SNAPPY_OK);
    out->write(&ctCompressedSize, sizeof(int));
    out->write(ctCompressed.data(), ctCompressedSize);
  }
  else
  {
    in->read(&ctCompressedSize, sizeof(int));
    ctCompressed.resize(ctCompressedSize);
    in->read(ctCompressed.data(), ctCompressedSize);
    VERIFY(snappy_uncompress(ctCompressed.data(), ctCompressedSize, reinterpret_cast<char*>(&colorTable[0][0][0]), &ctUncompressedSize) == SNAPPY_OK);
    ASSERT(ctUncompressedSize == sizeof(colorTable));
  }
  STREAM_REGISTER_FINISH;
}
