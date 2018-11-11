/**
 * @file FilteredBallPercepts.cpp
 *
 * Implementation of struct FilteredBallPercepts, a representation that
 * holds information about really seen balls and does (hopefully) not
 * contain any false positives ;-)
 *
 * @author Tim Laue
 */

#include "FilteredBallPercepts.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"

void FilteredBallPercepts::draw() const
{
  // drawing of the ball percepts in the field view
  DEBUG_DRAWING("representation:FilteredBallPercepts:field", "drawingOnField")
  {
    for(const auto& percept : percepts)
    {
      const Vector2f& p = percept.positionOnField;
      CROSS("representation:FilteredBallPercepts:field", p.x(), p.y(),
            40, 20, Drawings::solidPen, ColorRGBA::orange);
    }
  }
}

void FilteredBallPercepts::verify() const
{
#ifndef NDEBUG
  for(const auto& percept : percepts)
  {
    ASSERT(std::isfinite(percept.positionInImage.x()));
    ASSERT(std::isfinite(percept.positionInImage.y()));
    ASSERT(std::isfinite(percept.positionOnField.x()));
    ASSERT(std::isfinite(percept.positionOnField.y()));
    ASSERT(std::isnormal(percept.covOnField(0, 0)));
    ASSERT(std::isnormal(percept.covOnField(1, 1)));
    ASSERT(std::isfinite(percept.covOnField(0, 1)));
    ASSERT(std::isfinite(percept.covOnField(1, 0)));
    ASSERT(std::isfinite(percept.radiusOnField));
    ASSERT(percept.radiusOnField > 0.f);
    ASSERT(percept.timeWhenSeen > 0);
  }
#endif
}
