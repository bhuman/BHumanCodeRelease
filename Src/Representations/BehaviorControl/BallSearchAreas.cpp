/**
 * @file BallSearchAreas.cpp

 *
 * @author Sina Schreiber
 */

#include "BallSearchAreas.h"
BallSearchAreas::Cell::Cell(unsigned id, unsigned priority, unsigned timestamp, const Vector2f& positionOnField)
  : id(id), priority(priority), timestamp(timestamp), positionOnField(positionOnField)
{
}
