/**
 * @file ScoreDirections.h
 * Implementation of a representation that lists sectors of directions in which the ball
 * can be kicked and hit the goal. The directions are specified as the global positions
 * of obstacle borders that limit the free ranges to the left or right as seen from the
 * position of the ball, that is the global direction can be determined by
 * (limit - globalBall).angle().
 * @author Thomas Röfer
 */

#include "ScoreDirections.h"
#include "Tools/Debugging/DebugDrawings.h"

ScoreDirections::Sector::Sector(const Vector2<>& leftLimit, const Vector2<>& rightLimit)
: leftLimit(leftLimit),
  rightLimit(rightLimit) {}

void ScoreDirections::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ScoreDirections", "drawingOnField",
  {
    Drawings::PenStyle style = Drawings::ps_solid;
    for(const Sector& sector : sectors)
    {
      LINE("representation:ScoreDirections", sector.leftLimit.x, sector.leftLimit.y,
           sector.rightLimit.x, sector.rightLimit.y, 50, style, ColorRGBA::white);
      style = Drawings::ps_dot;
    }
  });
}
