#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/ScanlineRegions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/FieldBoundary.h"

MODULE(BallSpotProvider,
{,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(ScanlineRegionsClipped),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(ColorTable),
  REQUIRES(FieldBoundary),
  PROVIDES_WITH_MODIFY_AND_DRAW(BallSpots),
  DEFINES_PARAMETERS(
  {,
    (int) (2) step, /**<vertical steps */
    (float) (0.45f) scanHeight, /**<The additional scan lines will start at theImage.height * scanHeight */
    (float) (5.f) discardBallSpotDistance, /**<If a ball spot is closer than this value to the ball spot that was discovered bevore, it is discarded */ 
  }),
});

/**
* @class BallSpotProvider
* A module that provides spots that might be inside balls
*/
class BallSpotProvider: public BallSpotProviderBase
{
private:
  void update(BallSpots& ballSpots);
  using Scanline = ScanlineRegionsClipped::Scanline;
  using Region = ScanlineRegionsClipped::Region;
  
  /**searches the scanlines for ball spots*/
  bool searchScanLines(BallSpots& ballSpots) const;
  void searchBetweenScanlines(BallSpots& ballSpots) const;
  
  /**Tries to find the center of a Ball*/
  bool getBallSpot(BallSpot& ballSpot) const;
};
