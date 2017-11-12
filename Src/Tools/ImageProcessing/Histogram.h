/**
 * @file Histogram.h
 *
 * A namespace with methods for histogram calculations.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include <cstring>
#include <functional>
#include "Tools/Debugging/DebugDrawings.h"

template<typename PixelType>
class Histogram
{
public:
  //template<const std::function<unsigned char(const PixelType* const src)>& evalFunction>
  template<typename GetValueClass> //GetValueClass should habe a function getHistogramValue with header like above
  void calc(const PixelType* src, unsigned width, unsigned height, unsigned rowGabs, unsigned columnsGabs)
  {
    memset(histogram, 0, sizeof(unsigned int) * 256);

    const auto lineOverRun = columnsGabs - (width % columnsGabs) % columnsGabs;
    for(auto end = src + width * height; src < end; src += rowGabs * width - lineOverRun)
      for(auto endLine = src + width; src < endLine; src += columnsGabs)
        ++histogram[GetValueClass::getHistogramValue(src)];
  }

  int getOtsuThesh()
  {
    unsigned sb = 0; //sum pixels background
    unsigned cb = 0; //count (sum values) backgound)
    unsigned sf = 0; //sum foreground
    unsigned cf = 0; //count (sum values) foreground

    unsigned sum = 0; // sum / count of all pixels
    for(int i = 0; i < 256; ++i)
    {
      sum += histogram[i];
      cf += histogram[i] * i;
    }
    sf = sum;

    int bestPose = 0; // position of currently best between class variance
    float bestPoseBetweenClassVariance = 0; // currently best between class variance

    //try all seperation positions
    //a seperation is obligating (the iteration is desined so)
    for(int i = 0; i < 255; ++i)
    {
      const unsigned histI = histogram[i];
      sb += histI;         // shifting one position from foreground to background
      sf -= histI;         //   /
      cb += histI * i;     //  /
      cf -= histI * i;     // /

      // calculatie new between class variance
      const float betweenClassVariance = 1.f * sb / sum * sf / sum * sqr(1.f * cb / sb - 1.f * cf / sf);

      // memorize if better than last memorized
      if(betweenClassVariance > bestPoseBetweenClassVariance)
      {
        bestPoseBetweenClassVariance = betweenClassVariance;
        bestPose = i + 1; // the iteration i is pointing to the element we shifted to background
                          // what is one behind the threshold position
      }
    }
    return bestPose;
  }

  unsigned int histogram[256];
};

#define DRAW_HISTOGRAM(id, histo, xBase, yBase, penWidth, color, mirror, increment, chained, xScale, yScale) \
  do \
    COMPLEX_DRAWING(id) \
    { \
      if(chained) \
        for(int i = increment; i < 256; i += increment) \
          LINE(id, i * xScale + xBase, 1.f * yBase + (mirror ? -1. : 1.) * histo.histogram[i] * yScale, (i- increment) * xScale + xBase, 1.f *yBase + (mirror ? -1. : 1.) *histo.histogram[i - increment] * yScale, penWidth, Drawings::solidPen, color); \
      else \
        for(int i = 0; i < 256; i += increment) \
          LINE(id, i * xScale + xBase, 1.f *yBase + (mirror ? -1. : 1.) * histo.histogram[i] * yScale, i * xScale + xBase,  1.f *yBase +(mirror ? -1. : 1.) *histo.histogram[i] * yScale, penWidth, Drawings::solidPen, color); \
      const int otsu = histo.getOtsuThesh(); \
      LINE(id, otsu * xScale + xBase, 0, otsu * xScale + xBase, 1000, penWidth, Drawings::solidPen, color); \
    } \
  while(false)
