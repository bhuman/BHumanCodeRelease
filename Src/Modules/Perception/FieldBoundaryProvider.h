/**
* @author Alexis Tsogias
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/ScanlineRegions.h"

MODULE(FieldBoundaryProvider,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ColorTable),
  REQUIRES(Image),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(Odometer),
  REQUIRES(ScanlineRegions),
  PROVIDES_WITH_DRAW(FieldBoundary),
  DEFINES_PARAMETERS(
  {,
    (int)(2) upperBound,
    (int)(5) lowerBound,
    (int)(1) nonGreenPenalty,
    (int)(2) nonGreenPenaltyGreater,
    (int)(3500) nonGreenPenaltyDistance,
    (int)(5) minGreenCount,
  }),
});

/**
 *
 */
class FieldBoundaryProvider : public FieldBoundaryProviderBase
{
private:
  struct SpotAccumulator
  {
    int yStart;
    int yMax;
    int score;
    int maxScore;
  };

  using InImage = FieldBoundary::InImage;
  using InField = FieldBoundary::InField;

  bool validLowerCamSpots;
  InField lowerCamConvexHullOnField;
  InImage lowerCamSpotsInImage;
  InImage lowerCamSpostInterpol;

  void update(FieldBoundary& fieldBoundary);

  void handleLowerCamSpots();
  void findBundarySpots(FieldBoundary& fieldBoundary, int horizon);

  bool cleanupBoundarySpots(InImage& boundarySpots) const;
  std::vector<InImage> calcBoundaryCandidates(InImage boundarySpots) const;
  void findBestBoundary(const std::vector<InImage>& boundaryCandidates,
                        const InImage& boundarySpots, InImage& boundary) const;

  bool isLeftOf(Vector2<int>& a, Vector2<int>& b, Vector2<int>& c) const;
  InImage getUpperConvexHull(InImage& boundary) const;

  int clipToBoundary(const InImage& boundary, int x) const;

  int findGreaterPenaltyY(int horizon) const;
};
