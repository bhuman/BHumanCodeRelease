/**
 * @file PenaltyMarkPercept.h
 * Declaration of a struct that represents a penalty mark.
 * @author Maik Schünemann
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"

/**
 * @struct PenaltyMarkPercept
 * A struct that represents a detected penalty mark.
 */
STREAMABLE(PenaltyMarkPercept,
{
  void draw() const,

  (Vector2i)(Vector2i::Zero()) positionInImage, /**< Position in the image. */
  (Vector2f)(Vector2f::Zero()) positionOnField, /**< Position relative to robot on the field. */
  (Angle)(pi) quarterRotationOnField, /**< The rotation on the field in the range [-pi/4 and pi/4[. Don't use if outside that range */
  (bool)(false) wasSeen, /**< Indicates whether the feature has been seen in the last frame */
});
