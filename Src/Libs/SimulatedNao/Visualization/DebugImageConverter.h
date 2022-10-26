/**
 * @file DebugImageConverter.h
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Debugging/DebugImages.h"

class DebugImageConverter
{
private:
  using ConversionFunction = void (*)(unsigned int, const void*, void*);
  ConversionFunction converters[PixelTypes::numOfPixelTypes];

public:
  DebugImageConverter();
  ~DebugImageConverter();
  void convertToBGRA(const DebugImage& src, void* dest);
};
