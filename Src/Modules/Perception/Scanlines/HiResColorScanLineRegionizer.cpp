/**
 * @author Felix Thielke
 */

#include "HiResColorScanLineRegionizer.h"

MAKE_MODULE(HiResColorScanLineRegionizer, perception)

void HiResColorScanLineRegionizer::update(ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped)
{
  colorScanLineRegionsVerticalClipped.scanLines.clear();
  colorScanLineRegionsVerticalClipped.scanLines.reserve(theScanGrid.lines.size());
  colorScanLineRegionsVerticalClipped.lowResStart = theScanGrid.lowResStart;
  colorScanLineRegionsVerticalClipped.lowResStep = theScanGrid.lowResStep;

  if(theScanGrid.lines.empty() || !theFieldBoundary.isValid)
    return;

  auto loRes = theColorScanLineRegionsVertical.scanLines.cbegin();
  for(const ScanGrid::Line& line : theScanGrid.lines)
  {
    colorScanLineRegionsVerticalClipped.scanLines.emplace_back(static_cast<unsigned short>(line.x));
    std::vector<ScanLineRegion>& regions = colorScanLineRegionsVerticalClipped.scanLines.back().regions;

    const int yBoundary = std::min<int>(std::max(0, theFieldBoundary.getBoundaryY(line.x)), theECImage.colored.height - 1);
    if(loRes != theColorScanLineRegionsVertical.scanLines.cend() && line.x == loRes->x)
    {
      auto k = loRes->regions.cbegin();
      for(; k != loRes->regions.cend() && k->range.from >= yBoundary; ++k)
        regions.emplace_back(*k);
      if(k != loRes->regions.cend() && k->range.to > yBoundary)
        regions.emplace_back(yBoundary, k->range.to, k->color);
      ++loRes;
    }
    else
    {
      scanVertical(line, yBoundary, regions);
    }
  }
}

void HiResColorScanLineRegionizer::scanVertical(const ScanGrid::Line& line, const int top, std::vector<ScanLineRegion>& regions) const
{
  auto y = theScanGrid.y.begin() + line.yMaxIndex;
  const auto yEnd = theScanGrid.y.end();
  const int width = theECImage.colored.width;
  if(y != yEnd && *y >= top && line.yMax - 1 > top)
  {
    int prevY = line.yMax - 1 > *y ? line.yMax - 1 : *y++;
    int currentY = prevY + 1;
    const FieldColors::Color* pImg = &theECImage.colored[prevY][line.x];
    FieldColors::Color currentColor = *pImg;
    for(; y != yEnd && *y > top; ++y)
    {
      pImg += (*y - prevY) * width;

      // If color changes, determine edge position between last and current scanpoint
      const FieldColors::Color& color = *pImg;
      if(color != currentColor)
      {
        const int otherColorThreshold = std::max(static_cast<int>((prevY - *y) * minColorRatio), 1);
        const int yMin = std::max(*y - otherColorThreshold + 1, 0);
        int counter = 0;
        int yy = std::min(prevY - 1, line.yMax - 1);
        for(const FieldColors::Color* pImg2 = pImg + (yy - *y) * width; yy >= yMin && counter < otherColorThreshold; --yy, pImg2 -= width)
          if(*pImg2 != currentColor)
            ++counter;
          else
            counter = 0;

        // Enough pixels of different colors were found: end previous region and start a new one.
        if(counter == otherColorThreshold)
        {
          yy += otherColorThreshold + 1;
          ASSERT(currentY > yy);
          regions.emplace_back(yy, currentY, currentColor);
          currentColor = color;
          currentY = yy;
        }
      }
      prevY = *y;
    }
    ASSERT(currentY > top + 1);
    regions.emplace_back(top + 1, currentY, currentColor);
  }
}
