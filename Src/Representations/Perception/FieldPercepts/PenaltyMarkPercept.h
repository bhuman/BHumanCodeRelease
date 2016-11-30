/**
 * @file PenaltyMarkPercept.h
 * Declaration of a struct that represents a penalty mark.
 * @author Maik Schünemann
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct PenaltyMarkPercept
 * A struct that represents a detected penalty mark.
 */
STREAMABLE(PenaltyMarkPercept,
{
  void draw() const,

  (Vector2i)(Vector2i::Zero()) position, /**< Position in the image. */
  (Vector2f)(Vector2f::Zero()) positionOnField, /**< Position relative to robot on the field. */
  (unsigned)(0) timeLastSeen, /**< When was the percept last seen? */
});
