/**
 * @file ShellLimits.cpp
 * Implementation methods evaluating the shell limits at certain points.
 */

#include <algorithm>

#include "Tools/Debugging/Asserts.h"
#include "ShellLimits.h"

bool ShellLimits::Point::operator<(const ShellLimits::Point& other) const
{
  return x < other.x;
}

bool ShellLimits::evaluate(float x, float& min, float& max) const
{
  Point p;
  p.x = x;
  vector<Point>::const_iterator begin = points.begin();
  vector<Point>::const_iterator end   = points.end();

  if(begin->x > x)
    return false; // All x's are larger than the parameter x, so the parameter x is out of bounds.
  if((end - 1)->x < x)
    return false; // All x's are smaller than the paramter x, so the parameter x is out of bounds.

  vector<Point>::const_iterator it = lower_bound(begin, end, p);
  ASSERT(it < end); // ...since it >= end would mean that all x's are smaller than the parameter x.
  if(it == begin)
  {
    ASSERT(begin->x == x); // It is not smaller and not larger, so it must be equal (although this is a float).
    min = it->min;
    max = it->max;
  }
  else // begin < it < end
  {
    const vector<Point>::const_iterator it2 = it - 1;
    const float& xEnd     = it->x;
    const float& xStart   = it2->x;
    ASSERT(xStart < x);
    ASSERT(xEnd >= x);
    const float& minEnd   = it->min;
    const float& minStart = it2->min;
    const float& maxEnd   = it->max;
    const float& maxStart = it2->max;
    const float minSlope  = (minEnd - minStart) / (xEnd - xStart);
    const float maxSlope  = (maxEnd - maxStart) / (xEnd - xStart);
    min = minStart + (x - xStart) * minSlope;
    max = maxStart + (x - xStart) * maxSlope;
  }

  return true;
}
