/**
 * @file ScoreDirections.h
 * Declaration of a representation that lists sectors of directions in which the ball
 * can be kicked and hit the goal. The directions are specified as the global positions
 * of obstacle borders that limit the free ranges to the left or right as seen from the
 * position of the ball, that is the global direction can be determined by
 * (limit - globalBall).angle().
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ScoreDirections,
{
public:
  STREAMABLE(Sector,
  {
  public:
    Sector() = default;
    Sector(const Vector2<>& leftLimit, const Vector2<>& rightLimit),

    (Vector2<>) leftLimit, /** Global position of left limiting object border. */
    (Vector2<>) rightLimit, /** Global position of right limiting object border. */
  });

  void draw() const,

  (std::vector<Sector>) sectors, /**< All sectors sorted by their angular size (biggest first). */
});
