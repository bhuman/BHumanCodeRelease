#pragma once

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/Transformation.h"
#include <vector>

STREAMABLE(PotentialScanlineSpot,
{
  PotentialScanlineSpot() = default;
  PotentialScanlineSpot(int xImg, int yImg, bool used, const Vector2f& inField, int height),

  (Vector2i) spotInImg,
  (Vector2f) spotInField,
  (bool) usedAsStartingPoint, /**<Whether this point has already been used as a starting point for line fitting */
  (int) heightInImg,
  (unsigned int) scanlineIdx,
});

inline PotentialScanlineSpot::PotentialScanlineSpot(int xImg, int yImg, bool used, const Vector2f& inField, int height) :
  spotInImg(xImg, yImg), spotInField(inField), usedAsStartingPoint(used), heightInImg(height)
{}

STREAMABLE(PotentialLineSpots,
{
  PotentialLineSpots() = default;

  void draw() const,

  (std::vector<PotentialScanlineSpot>) spots,
  (unsigned)(0) numOfScanlines,
});

inline void PotentialLineSpots::draw() const
{
  DEBUG_DRAWING("representation:PotentialLineSpots:image", "drawingOnImage")
    for(const auto& spot : spots)
    {
      CROSS("representation:PotentialLineSpots:image", spot.spotInImg.x(), spot.spotInImg.y(), 3, 1, Drawings::solidPen, ColorRGBA::red);
    }

  DEBUG_DRAWING("representation:PotentialLineSpots:field", "drawingOnField")
    for(const auto& spot : spots)
    {
      CROSS("representation:PotentialLineSpots:field", spot.spotInField.x(), spot.spotInField.y(), 15, 10, Drawings::solidPen, ColorRGBA::red);
    }
}
