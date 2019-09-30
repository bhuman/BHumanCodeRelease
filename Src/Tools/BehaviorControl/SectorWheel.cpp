/**
 * @file SectorWheel.cpp
 *
 * This file implements a class that represents a wheel of sectors around a center point.
 *
 * @author Arne Hasselbring
 */

#include "SectorWheel.h"
#include "Platform/BHAssert.h"
#include <algorithm>
#include <cmath>
#include <limits>

void SectorWheel::begin(const Vector2f& positionOnField)
{
  this->positionOnField = positionOnField;

  wheel.clear();
  wheel.emplace_back(Rangea(-pi, pi), std::numeric_limits<float>::max(), Sector::free);
}

const std::list<SectorWheel::Sector>& SectorWheel::finish()
{
  ASSERT(!wheel.empty());
  ASSERT(wheel.front().angleRange.min == -pi);
  ASSERT(wheel.back().angleRange.max == pi);
  if(wheel.size() > 1)
  {
    if(wheel.front().distance == wheel.back().distance && wheel.front().type == wheel.back().type)
    {
      wheel.back().angleRange.max = wheel.front().angleRange.max;
      wheel.pop_front();
    }
  }

#ifndef NDEBUG
  for(auto it = wheel.begin(), it2 = std::next(it); it2 != wheel.end(); ++it, ++it2)
    ASSERT(it->angleRange.max == it2->angleRange.min);
#endif

  return wheel;
}

void SectorWheel::addSector(const Vector2f& center, float width, Sector::Type type)
{
  return addSector(center, width, (center - positionOnField).norm(), type);
}

void SectorWheel::addSector(const Vector2f& center, float width, float distance, Sector::Type type)
{
  const float radius = std::atan2(width / 2.f, distance);
  const Angle direction = (center - positionOnField).angle();
  const Angle left = direction + radius;
  const Angle right = direction - radius;
  addSector(Rangea(right, left), distance, type);
}

void SectorWheel::addSector(const Rangea& angleRange, float distance, Sector::Type type)
{
  // TODO: This is only a workaround.
  if(angleRange.min == angleRange.max)
    return;
  // TODO: correct?
  if(angleRange.min > angleRange.max)
  {
    addSectorNormalized(Rangea(angleRange.min, pi), distance, type);
    addSectorNormalized(Rangea(-pi, angleRange.max), distance, type);
    return;
  }
  // If the sector has parts outside [-pi, pi[, split it up into two parts (they will be connected in the end).
  if(angleRange.max > pi)
  {
    ASSERT(angleRange.min < pi);
    addSectorNormalized(Rangea(angleRange.min, pi), distance, type);
    addSectorNormalized(Rangea(-pi, angleRange.max - pi2), distance, type);
    return;
  }
  else if(angleRange.min < -pi)
  {
    addSectorNormalized(Rangea(-pi, angleRange.max), distance, type);
    addSectorNormalized(Rangea(angleRange.min + pi2, pi), distance, type);
    return;
  }
  addSectorNormalized(angleRange, distance, type);
}


/**
* @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
*/
void SectorWheel::addSectorNormalized(const Rangea& angleRange, float distance, Sector::Type type)
{
  if(angleRange.max == angleRange.min)
    return;

  // There should always be the [-pi, pi[ sector.
  ASSERT(!wheel.empty());
  ASSERT(angleRange.max > angleRange.min);
  ASSERT(angleRange.min >= -pi);
  ASSERT(angleRange.max <= pi);

  // These variables are used to extend a new sector if it crosses multiple old sectors.
  bool contiguous = false;
  auto currentContiguousPart = wheel.end();
  for(auto it = wheel.begin(); it != wheel.end(); ++it)
  {
    if(it->angleRange.max <= angleRange.min) // The current sector ends before the new one starts -> try next.
      continue;

    if(it->angleRange.min >= angleRange.max) // The current sector begins after the new one ends -> done.
      break;

    if(distance > it->distance) // The current sector is closer than the new one -> look at next.
    {
      contiguous = false;
      continue;
    }

    // <---> below the new sector is closer than the current iterated one
    const Sector currentSector = *it;
    auto nextIt = std::next(it);

    // if the current sector begins before the new one, shrink the current to fit before it otherwise remove the current
    if(it->angleRange.min < angleRange.min)
      it->angleRange.max = (ASSERT(!contiguous), angleRange.min);
    else
      nextIt = wheel.erase(it);

    // add the new sector
    if(contiguous)
      currentContiguousPart->angleRange.max = std::min(currentSector.angleRange.max, angleRange.max), it = currentContiguousPart;
    else
      it = wheel.emplace(nextIt, Rangea(std::max(angleRange.min, currentSector.angleRange.min), std::min(currentSector.angleRange.max, angleRange.max)), distance, type);

    //if the current sector end after the new one: add some parts again or else set continues
    if((contiguous = currentSector.angleRange.max <= it->angleRange.max))
      currentContiguousPart = it;
    else
      it = wheel.emplace(nextIt, Rangea(it->angleRange.max, currentSector.angleRange.max), currentSector.distance, currentSector.type);
  }
}
