/**
 * @author Alexis Tsogias
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ColorScanlineRegions.h"

MODULE(FieldBoundaryProvider,
{,
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(ECImage),
  REQUIRES(Odometer),
  REQUIRES(ColorScanlineRegionsVertical),
  PROVIDES(FieldBoundary),
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

  bool validLowerCamSpots = false;
  InField lowerCamConvexHullOnField;
  InImage lowerCamSpotsInImage;
  InImage lowerCamSpotsInterpol;
  std::vector<InImage> convexBoundaryCandidates; ///< Possible boundary candidates.

  void update(FieldBoundary& fieldBoundary);

  void handleLowerCamSpots();
  void findBoundarySpots(FieldBoundary& fieldBoundary);

  bool cleanupBoundarySpots(InImage& boundarySpots) const;
  void calcBoundaryCandidates(InImage boundarySpots);
  void findBestBoundary(const std::vector<InImage>& boundaryCandidates,
                        const InImage& boundarySpots, InImage& boundary) const;

  bool isLeftOf(const Vector2i& a, const Vector2i& b, const Vector2i& c) const;
  void getUpperConvexHull(const InImage& boundary, InImage& hull) const;

  int clipToBoundary(const InImage& boundary, int x) const;

  int findYInImageByDistance(int distance) const;

  void invalidateBoundary(FieldBoundary& fieldBoundary) const;

  void draw() const;
};
