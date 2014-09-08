#include "BallSpotProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <algorithm>
#include <vector>

void BallSpotProvider::update(BallSpots& ballSpots)
{
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:additionalScanlines","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:orangeLines","drawingOnImage");
  
  ballSpots.ballSpots.clear();
  //first collect all orange regions from the scanlines
  //If no ballspot has been found on the scanlines, and this is the upper camera
  //search between the scanlines for ballspots
  searchScanLines(ballSpots); //FIXME if this is too slow only search between lines if necessary
  if(theCameraInfo.camera == CameraInfo::upper)
  {
    searchBetweenScanlines(ballSpots);
  }
}

bool BallSpotProvider::searchScanLines(BallSpots& ballSpots) const
{
  bool found = false;
  for(const Scanline& line : theScanlineRegionsClipped.scanlines)
  {
    for(const Region& region : line.regions)
    {
      ASSERT(region.upper >= 0 && region.upper < theImage.height);
      ASSERT(region.lower > 0 && region.lower <= theImage.height);
      //FIXME there is a bug somewhere inside the ScanlineRegionizer that creates
      //invalid regions
      if(region.upper < 0 || region.upper >= theImage.height || 
         region.lower <= 0 || region.lower > theImage.height)
      {
        TRACE("BallSpotProvider: Discarded Region");
        continue;
      }
      
      if(region.color.is(ColorClasses::orange))
      {
        const int y = static_cast<int>((region.upper + region.lower) / 2);        
        ASSERT(static_cast<int>(line.x) < theImage.width);
        ASSERT(y >= 0 && y < theImage.height);
        BallSpot bs(line.x, y);
        if(getBallSpot(bs))
        {
          found = true;
          ballSpots.ballSpots.emplace_back(bs.position.x, bs.position.y);
        }
      }
    }
  }
  return found;
}

void BallSpotProvider::searchBetweenScanlines(BallSpots& ballSpots) const
{
  if(theScanlineRegionsClipped.scanlines.size() <= 1)
    return;//this happens if the camera matrix is invalid
  //get distance between two scanlines
  ASSERT(theScanlineRegionsClipped.scanlines.size() > 1);
  const int a = theScanlineRegionsClipped.scanlines[0].x;
  const int b = theScanlineRegionsClipped.scanlines[1].x;
  const int thirdDist = static_cast<int>((abs(a-b)/3));
  ASSERT(thirdDist > 0); //this is the case if dist >= 3
  
  const int scanlineStartY = static_cast<int>(theImage.height * scanHeight);
  ASSERT(scanlineStartY >= 0 && scanlineStartY < theImage.height);
  std::vector<Scanline>::const_iterator line = theScanlineRegionsClipped.scanlines.begin();
  const std::vector<Scanline>::const_iterator end =  theScanlineRegionsClipped.scanlines.end() - 1; //-1 to avoid leaving the image
  Vector2<> lastBallSpot(std::numeric_limits<float>::min(), std::numeric_limits<float>::min()); //The initialization ensures that the first ball spot we find will be far enough away from this spot
  for(; line != end; ++line)
  {
    int x = line->x;
    for(int i = 0; i < 2; ++i)
    {
      x += thirdDist;
      ASSERT(x < theImage.width);
      ASSERT(x >= 0);
      const Image::Pixel* pPixel = &theImage[scanlineStartY][x];
      const int minY = std::max(0, theFieldBoundary.getBoundaryY(x));
      const int pixelStep = theImage.widthStep * step;
      int y = scanlineStartY;
      bool insideOrange = false;
      int orangeRegionStart = -1;
      int lastOrangeRegion = -1;
      for(; y >= minY; pPixel -= pixelStep, y -= step)
      {
        ASSERT(y < theImage.height);
        ASSERT(y >= 0);
        CROSS("module:BallSpotProvider:additionalScanlines", x, y, 1, 1, Drawings::ps_solid, ColorRGBA::blue);
        
        if(insideOrange)
        {
          //we are already inside an orange region, check if it continues
          if(!theColorTable[*pPixel].is(ColorClasses::orange))
          {
            //orange region ends here
            const int regionY = (orangeRegionStart + lastOrangeRegion) / 2;
            LINE("module:BallSpotProvider:orangeLines", x, orangeRegionStart, x, lastOrangeRegion, 1, Drawings::ps_solid, ColorRGBA::orange);     
            BallSpot bs(x, regionY);
            if(getBallSpot(bs))
            {
              const float dist = (lastBallSpot - Vector2<>((float) bs.position.x, (float) bs.position.y)).abs();
              if(dist > discardBallSpotDistance)
              {
                ballSpots.ballSpots.emplace_back(bs.position.x, bs.position.y);
                lastBallSpot.x = (float) bs.position.x;
                lastBallSpot.y = (float) bs.position.y;
              }
            }
            insideOrange = false;
          }
          else
          {
            //still inside orange region
            lastOrangeRegion = y;
          }
        }
        else
        {
          if(theColorTable[*pPixel].is(ColorClasses::orange))
          {//start of a new orange region
            orangeRegionStart = y;
            lastOrangeRegion = y;
            insideOrange = true;
          }
          else
          {
            //not inside an orange region and current pixel is not orange => do nothing
          }
        }
      }
    }
    
  }
}


bool BallSpotProvider::getBallSpot(BallSpot& ballSpot) const
{
  const int height = theImage.height - 3;
  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y);
  horizon = std::min(horizon, height);
  const int leftLimit = 2;
  const unsigned int orangeSkipping = 5;
  const int rightLimit = theImage.width - 3;

  unsigned int skipped = 0;
  int lower, upper;
  int left = 0;
  int right = 0;
  // find upper/lower => middle vertical
  lower = upper = ballSpot.position.y;
  skipped = 0;
  while(lower <= height && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[lower][ballSpot.position.x]].is(ColorClasses::orange))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    lower++;
  }
  lower -= skipped;

  skipped = 0;
  while(upper >= horizon && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[upper][ballSpot.position.x]].is(ColorClasses::orange))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    upper--;
  }
  upper += skipped + 1;
  ballSpot.position.y = (lower + upper) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines",
       ballSpot.position.x, lower,
       ballSpot.position.x, upper,
       1, Drawings::ps_solid, ColorRGBA::blue);
  // find left/right => middle horizontal
  left = right = ballSpot.position.x;
  skipped = 0;
  while(left >= leftLimit && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[ballSpot.position.y][left]].is(ColorClasses::orange))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    left--;
  }
  left += skipped + 1;
  skipped = 0;
  while(right <= rightLimit && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[ballSpot.position.y][right]].is(ColorClasses::orange))
    {
      skipped = 0;
    }
    else
    {
      skipped++;
    }
    right++;
  }
  right -= skipped;
  ballSpot.position.x = (left + right) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines",
       left, ballSpot.position.y,
       right, ballSpot.position.y,
       1, Drawings::ps_solid, ColorRGBA::blue);
  const int minBallSpotRadius = 3;
  return (right - left) >= minBallSpotRadius &&
         (lower - upper) >= minBallSpotRadius;
}


MAKE_MODULE(BallSpotProvider, Perception)
