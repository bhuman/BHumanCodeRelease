#include "BallSpotProvider.h"
#include <algorithm>

void BallSpotProvider::update(BallSpots& ballSpots)
{
  DECLARE_DEBUG_DRAWING("module:BallSpotProvider:ballSpotScanLines","drawingOnImage");
  ballSpots.ballSpots.clear();
  searchScanLines(ballSpots);
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

      if(region.color.is(ColorClasses::orange))
      {
        const int y = static_cast<int>((region.upper + region.lower) / 2);
        ASSERT(static_cast<int>(line.x) < theImage.width);
        ASSERT(y >= 0 && y < theImage.height);
        BallSpot bs(line.x, y);
        if(getBallSpot(bs))
        {
          found = true;
          ballSpots.ballSpots.emplace_back(bs.position);
        }
      }
    }
  }
  return found;
}

bool BallSpotProvider::getBallSpot(BallSpot& ballSpot) const
{
  const int height = theImage.height - 3;
  int horizon = std::max(2, (int) theImageCoordinateSystem.origin.y());
  horizon = std::min(horizon, height);
  const int leftLimit = 2;
  const unsigned int orangeSkipping = 5;
  const int rightLimit = theImage.width - 3;

  unsigned int skipped = 0;
  int lower, upper;
  int left = 0;
  int right = 0;
  // find upper/lower => middle vertical
  lower = upper = ballSpot.position.y();
  skipped = 0;
  while(lower <= height && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[lower][ballSpot.position.x()]].is(ColorClasses::orange))
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
    if(theColorTable[theImage[upper][ballSpot.position.x()]].is(ColorClasses::orange))
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
  ballSpot.position.y() = (lower + upper) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines",
       ballSpot.position.x(), lower,
       ballSpot.position.x(), upper,
       1, Drawings::solidPen, ColorRGBA::blue);
  // find left/right => middle horizontal
  left = right = ballSpot.position.x();
  skipped = 0;
  while(left >= leftLimit && skipped < orangeSkipping)
  {
    if(theColorTable[theImage[ballSpot.position.y()][left]].is(ColorClasses::orange))
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
    if(theColorTable[theImage[ballSpot.position.y()][right]].is(ColorClasses::orange))
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
  ballSpot.position.x() = (left + right) / 2;
  LINE("module:BallSpotProvider:ballSpotScanLines",
       left, ballSpot.position.y(),
       right, ballSpot.position.y(),
       1, Drawings::solidPen, ColorRGBA::blue);
  const int minBallSpotRadius = 3;
  return (right - left) >= minBallSpotRadius &&
         (lower - upper) >= minBallSpotRadius;
}


MAKE_MODULE(BallSpotProvider, perception)
