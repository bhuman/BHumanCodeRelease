/**
 * The file implements a class that creates colored regions on a single vertical scanline.
 * The class is based on Arne Böckmann's original implementation of this idea.
 * @author Thomas Röfer
 */

#include "LineScanner.h"
#include <algorithm>

void LineScanner::scan(const ScanGrid::Line& line, const int top, const float minColorRatio, ScanlineRegions& scanlineRegions) const
{
  const int x = line.x;
  scanlineRegions.scanlines.emplace_back(x);
  auto& regions = scanlineRegions.scanlines.back().regions;

  auto y = scanGrid.y.begin() + line.yMaxIndex;
  const auto yEnd = scanGrid.y.end();
  const int widthStep = image.widthStep;
  if(y != yEnd && *y > top && line.yMax - 1 > top)
  {
    int prevY = line.yMax - 1 > *y ? line.yMax - 1 : *y++;
    int currentY = prevY + 1;
    const Image::Pixel* pImg = &image[prevY][x];
    ColorTable::Colors currentColor = colorTable[*pImg];
    for(; y != yEnd && *y > top; ++y)
    {
      pImg += (*y - prevY) * widthStep;

      // If color changes, determine edge position between last and current scanpoint
      const ColorTable::Colors& color = colorTable[*pImg];
      if(color.colors != currentColor.colors)
      {
        const int otherColorThreshold = std::max(static_cast<int>((prevY - *y) * minColorRatio), 1);
        const int yMin = std::max(*y - otherColorThreshold + 1, 0);
        int counter = 0;
        int yy = std::min(prevY - 1, line.yMax - 1);
        for(const Image::Pixel* pImg2 = pImg + (yy - *y) * widthStep; yy >= yMin && counter < otherColorThreshold; --yy, pImg2 -= image.widthStep)
          if(colorTable[*pImg2].colors != currentColor.colors)
            ++counter;
          else
            counter = 0;

        // Enough pixels of different colors were found: end previous region and start a new one.
        if(counter == otherColorThreshold)
        {
          yy += otherColorThreshold + 1;
          ASSERT(currentY > yy);
          regions.emplace_back(currentY, yy, currentColor);
          currentColor = color;
          currentY = yy;
        }
      }
      prevY = *y;
    }
    ASSERT(currentY > top + 1);
    regions.emplace_back(currentY, top + 1, currentColor);
  }
}
