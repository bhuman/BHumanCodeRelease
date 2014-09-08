#include "ScanlineRegions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

ScanlineRegions::Region::Region(const unsigned lower, const unsigned upper, const Color& c) :
  upper(upper), lower(lower), color(c)
{}

ScanlineRegions::Region::Region(const Region& other) :
  upper(other.upper), lower(other.lower), color(other.color)
{}

ScanlineRegions::Scanline::Scanline(const unsigned x) :
  x(x)
{
  regions.reserve(15);
}

void ScanlineRegions::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ScanlineRegions:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:ScanlineRegions:Image",
  {
    for(const Scanline& line : scanlines)
    {
      for(const Region& r : line.regions)
      {
        int x = line.x;
        ColorTable::Colors colors = r.color;
        for(int i = 0; i < ColorClasses::numOfColors; ++i)
          if(colors.is((ColorClasses::Color) i))
          {
            LINE("representation:ScanlineRegions:Image", x, r.lower - 1, x, r.upper, 2, Drawings::ps_solid, ColorClasses::getColorRGBA((ColorClasses::Color) i));
            ++x;
          }
      }
    }
  });
  
  COMPLEX_DEBUG_IMAGE(RegionsReconstructed,
  {
    if(Blackboard::getInstance().exists("CameraInfo")) //need CameraInfo to get width/height of image
    {
      const CameraInfo& cameraInfo = (const CameraInfo&)Blackboard::getInstance()["CameraInfo"];
      INIT_DEBUG_IMAGE_BLACK(RegionsReconstructed, cameraInfo.width, cameraInfo.height);
      for(int i = 0; i < int(scanlines.size()) - 1; ++i)
      {
        const Scanline& currentLine = scanlines[i];
        const Scanline& nextLine = scanlines[i + 1];
        for(const Region& currentRegion : currentLine.regions)
        {
          for(const Region& nextRegion : nextLine.regions)
          {
            if(currentRegion.color.colors == nextRegion.color.colors &&
               overlap(currentRegion, nextRegion))
            {
              drawParallelogram(currentRegion, nextRegion, currentLine.x, nextLine.x);
            }
          }
        }
      }
    }
    SEND_DEBUG_IMAGE(RegionsReconstructed);
  });
  
}

void ScanlineRegions::drawParallelogram(const Region& left, const Region& right,
                                        const int leftX, const int rightX) const
{
  //create biggest rectangle
  const int top = std::min(left.upper, right.upper);
  const int bottom = std::max(left.lower, right.lower); 
  Geometry::PixeledLine upperLine(leftX, rightX, left.upper, right.upper);
  Geometry::PixeledLine lowerLine(leftX, rightX, left.lower, right.lower);
  for(int x = leftX; x <= rightX; ++x)
  {
    const int lineIndex = x - leftX;
    //bottom index is exclusive
    for(int y = top; y < bottom; ++y) //bottom index is exclusive
    {      
      const int upperY = upperLine.getPixelY(lineIndex);
      const int lowerY = lowerLine.getPixelY(lineIndex);
      if(y >= upperY && y <= lowerY)
      {//if pixel is inside parallelogram
        ColorTable::Colors colors = left.color;
        for(int i = 0; i < ColorClasses::numOfColors; ++i)
          if(colors.is((ColorClasses::Color) i))
          {
            const ColorClasses::Color col = (ColorClasses::Color)i;
            const ColorRGBA colRgb = ColorClasses::getColorRGBA(col);
            DEBUG_IMAGE_SET_PIXEL_RGB(RegionsReconstructed, x, y, colRgb.r, colRgb.g, colRgb.b);
          }
      }
    }
  }
}

bool ScanlineRegions::overlap(const Region& r1, const Region& r2) const
{
  return r1.lower >= r2.upper && r2.lower >= r1.upper;
}

void ScanlineRegionsClipped::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ScanlineRegionsClipped:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:ScanlineRegionsClipped:Image",
  {
    for(const Scanline& line : scanlines)
    {
      for(const Region& r : line.regions)
      {
        int x = line.x;
        ColorTable::Colors colors = r.color;
        for(int i = 0; i < ColorClasses::numOfColors; ++i)
          if(colors.is((ColorClasses::Color) i))
          {
            LINE("representation:ScanlineRegionsClipped:Image", x, r.lower - 1, x, r.upper, 2, Drawings::ps_solid, ColorClasses::getColorRGBA((ColorClasses::Color) i));
            ++x;
          }
      }
    }
  });
}
