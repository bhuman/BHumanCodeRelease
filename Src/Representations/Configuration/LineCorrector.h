/**
 * @file LineCorrector.h
 *
 * This file declares a representation that has functionality to fit lines with higher precision than the normal line perception.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(LineCorrector,
{
  /** This struct represents a corrected line with its offset. */
  struct Line
  {
    Vector2f aInImage COMMA bInImage; /**< The first and last point of the line in the image. */
    Vector2f aOnField COMMA bOnField; /**< The first and last point of the line on the field. */
    float offset = 0.f; /**< The offset of the found line edge from the mid of the line (in mm). Usually +/- half of the fieldlines width */
  };

  FUNCTION(bool(Line&)) fitLine, /**< A function to fit a line. */
});
