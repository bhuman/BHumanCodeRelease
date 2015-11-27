#include "ScanlineRegions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Module/Blackboard.h"
#include <algorithm>

ScanlineRegions::Scanline::Scanline(const unsigned x) :
  x(x)
{
  regions.reserve(100);
}

void ScanlineRegions::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ScanlineRegions:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:ScanlineRegions:Image")
  {
    for(const Scanline& line : scanlines)
    {
      for(const Region& r : line.regions)
      {
        int x = line.x;
        const ColorTable::Colors colors = r.color;
        for(int i = 0; i < ColorClasses::numOfColors; ++i)
          if(colors.is(static_cast<ColorClasses::Color>(i)))
          {
            LINE("representation:ScanlineRegions:Image", x, r.lower - 1, x, r.upper, 2, Drawings::solidPen, ColorClasses::getColorRGBA(static_cast<ColorClasses::Color>(i)));
            ++x;
          }
      }
    }
  }

  COMPLEX_IMAGE(RegionsReconstructed)
  {
    if(Blackboard::getInstance().exists("CameraInfo")) //need CameraInfo to get width/height of image
    {
      const CameraInfo& cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      INIT_DEBUG_IMAGE_BLACK(RegionsReconstructed, cameraInfo.width, cameraInfo.height);
      for(int i = lowResStart; i < int(scanlines.size() - lowResStep); i += lowResStep)
      {
        const Scanline& currentLine = scanlines[i];
        const Scanline& nextLine = scanlines[i + lowResStep];
        for(const Region& currentRegion : currentLine.regions)
        {
          for(const Region& nextRegion : nextLine.regions)
          {
            if(currentRegion.color.colors == nextRegion.color.colors && overlap(currentRegion, nextRegion))
            {
              //convert colortable value to rgb
              //this is a very simple conversion that does not mix colors,
              //instead it takes the last color it finds and uses it.
              const ColorTable::Colors colors = currentRegion.color;
              ColorRGBA colRgb;
              for(int i = 0; i < ColorClasses::numOfColors; ++i)
              {
                if(colors.is(static_cast<ColorClasses::Color>(i)))
                {
                  const ColorClasses::Color col = static_cast<ColorClasses::Color>(i);
                  colRgb = ColorClasses::getColorRGBA(col);
                }
              }
              //draw a parallelogram from two triangles
              //upper triangle:
              DEBUG_IMAGE_TRIANGLE_RGB(RegionsReconstructed,
                                       currentLine.x, currentRegion.upper, //top left
                                       nextLine.x, nextRegion.lower, //bottom right
                                       nextLine.x, nextRegion.upper,//top right
                                       colRgb.r, colRgb.g, colRgb.b);
              DEBUG_IMAGE_FILL_TRIANGLE_RGB(RegionsReconstructed,
                                            currentLine.x, currentRegion.upper, //top left
                                            nextLine.x, nextRegion.lower, //bottom right
                                            nextLine.x, nextRegion.upper,//top right
                                            colRgb.r, colRgb.g, colRgb.b);

              //lower triangle:
              DEBUG_IMAGE_TRIANGLE_RGB(RegionsReconstructed,
                                       currentLine.x, currentRegion.upper, //top left
                                       currentLine.x, currentRegion.lower, //bottom left
                                       nextLine.x, nextRegion.lower,//bottom right
                                       colRgb.r, colRgb.g, colRgb.b);
              DEBUG_IMAGE_FILL_TRIANGLE_RGB(RegionsReconstructed,
                                            currentLine.x, currentRegion.upper, //top left
                                            currentLine.x, currentRegion.lower, //bottom left
                                            nextLine.x, nextRegion.lower,//bottom right
                                            colRgb.r, colRgb.g, colRgb.b);
            }
          }
        }
      }
    }
    SEND_DEBUG_IMAGE(RegionsReconstructed);
  }
}

bool ScanlineRegions::overlap(const Region& r1, const Region& r2) const
{
  return r1.lower >= r2.upper && r2.lower >= r1.upper;
}

void ScanlineRegionsClipped::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ScanlineRegionsClipped:Image", "drawingOnImage");
  COMPLEX_DRAWING("representation:ScanlineRegionsClipped:Image")
  {
    for(const Scanline& line : scanlines)
    {
      for(const Region& r : line.regions)
      {
        int x = line.x;
        const ColorTable::Colors colors = r.color;
        for(int i = 0; i < ColorClasses::numOfColors; ++i)
          if(colors.is(static_cast<ColorClasses::Color>(i)))
          {
            LINE("representation:ScanlineRegionsClipped:Image", x, r.lower - 1, x, r.upper, 2, Drawings::solidPen, ColorClasses::getColorRGBA(static_cast<ColorClasses::Color>(i)));
            ++x;
          }
      }
    }
  }
}
