/**
 * @file FieldInterceptBall.cpp
 *
 * Declaration of a representation that contains additional information
 * about the intercepted position of the ball that is required by the behavior.
 *
 * @author Philip Reichenberg
 */

#include "FieldInterceptBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Blackboard.h"
#include "Tools/Modeling/BallPhysics.h"

void FieldInterceptBall::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:FieldBall:relative", "drawingOnField");

  if(timeUntilIntersectsOwnYAxis != std::numeric_limits<float>::max() && intersectionPositionWithOwnYAxis != Vector2f::Zero())
  {
    CROSS("representation:FieldBall:relative", intersectionPositionWithOwnYAxis.x(), intersectionPositionWithOwnYAxis.y(), 150, 20, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:FieldBall:relative", 0.f, 0.f, intersectionPositionWithOwnYAxis.x() * 1.5f, intersectionPositionWithOwnYAxis.y() * 1.5f,
         10, Drawings::dashedPen, ColorRGBA::red);
    DRAW_TEXT("representation:FieldBall:relative", intersectionPositionWithOwnYAxis.x() * 1.5f, intersectionPositionWithOwnYAxis.y() * 1.5f, 250, ColorRGBA::blue, timeUntilIntersectsOwnYAxis);
  }
}
