/**
 * @file PenaltyMarkPercept.cpp
 * Implementation of a struct that represents a penalty mark.
 * @author Maik Schünemann
 */

#include "PenaltyMarkPercept.h"
#include "Debugging/DebugDrawings.h"

void PenaltyMarkPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:PenaltyMarkPercept:field", "drawingOnField");

  if(wasSeen)
  {
    CROSS("representation:PenaltyMarkPercept:image", positionInImage.x(), positionInImage.y(), 5, 5, Drawings::solidPen, ColorRGBA::blue);
    CROSS("representation:PenaltyMarkPercept:field", positionOnField.x(), positionOnField.y(), 40, 40, Drawings::solidPen, ColorRGBA::blue);
  }
}

void PenaltyMarkPercept::verify() const
{
  if(wasSeen)
  {
    ASSERT(std::isfinite(positionOnField.x()));
    ASSERT(std::isfinite(positionOnField.y()));
    ASSERT(std::isnormal(covarianceOnField(0, 0)));
    ASSERT(std::isnormal(covarianceOnField(1, 1)));
    ASSERT(std::isfinite(covarianceOnField(0, 1)));
    ASSERT(std::isfinite(covarianceOnField(1, 0)));
  }
}
