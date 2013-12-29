/**
* @file Tools/Optimization/DownhillSimplex.cpp
* Implementation of a downhill simplex optimization algorithm
* @author Colin Graf
*/

#include <algorithm>
#include <limits>
#include <cmath>
#include <cstdlib>

#include "DownhillSimplex.h"
#include "Platform/BHAssert.h"

DownhillSimplex::DownhillSimplex() : bestRating(std::numeric_limits<float>::max()), state(computeReflection) {}

void DownhillSimplex::addDimension(float& variable, float min, float max, float minDelta)
{
  ASSERT(points.empty());
  ASSERT(max > min);
  ASSERT(minDelta > 0.f);
  ASSERT(minDelta <= max - min);
  dimensions.push_back(Dimension(&variable, min, max, minDelta));
}

void DownhillSimplex::start()
{
  ASSERT(points.empty());
  ASSERT(dimensions.size() > 0);

  points.reserve(dimensions.size() + 1);
  points.push_back(Point());
  Point& startPoint = points.back();

  for(const Dimension* dimension = &dimensions.front(), * end = dimension + dimensions.size(); dimension < end; ++dimension)
    startPoint.position.push_back(*dimension->variable);
  unratedPoints.push_back(&startPoint);

  for(int i = 0, count = dimensions.size(); i < count; ++i)
  {
    const Dimension& dimension = dimensions[i];

    points.push_back(Point());
    Point& point = points.back();
    point.position = startPoint.position;
    ASSERT(point.position.size() == dimensions.size());

    const float sign = rand() % 2 ? 1.f : -1.f;
    float pos = point.position[i] + dimension.minDelta * sign;
    //if((sign == 1.f && pos > dimension.max) || (sign == -1.f && pos < dimension.min))
    //  pos = point.position[i] - dimension.minDelta * sign;

    point.position[i] = pos;
    unratedPoints.push_back(&point);
  }

  ASSERT(points.size() == dimensions.size() + 1);
  ASSERT(unratedPoints.size() == dimensions.size() + 1);
  ASSERT(unratedPoints.front() == &startPoint);
}

void DownhillSimplex::setRating(float rating)
{
  ASSERT(unratedPoints.size() > 0);
  unratedPoints.front()->rating = rating;
  unratedPoints.pop_front();
  if(rating < bestRating)
    bestRating = rating;
}

void DownhillSimplex::next()
{
  if(unratedPoints.size() > 0)
  {
    const Point& point = *unratedPoints.front();
    for(int i = 0, count = dimensions.size(); i < count; ++i)
      *dimensions[i].variable = point.position[i];
    return;
  }

  switch(state)
  {
  case computeReflection:
    {
      // sort points by rating
      ASSERT(points.size() >= 2);
      std::sort(points.begin(), points.end());
      ASSERT(points.front().rating <= points.back().rating);

      // compute center of gravity of all points except point[n - 1]
      centerOfGravity = points.front().position;
      for(const Point* point = &points.front() + 1, * end = &points.front() + points.size() - 1; point < end; ++point)
      {
        for(int i = 0, count = point->position.size(); i < count; ++i)
          centerOfGravity[i] += point->position[i];
      }
      const float scale = 1.f / float(points.size() - 1);
      for(int i = 0, count = centerOfGravity.size(); i < count; ++i)
        centerOfGravity[i] *= scale;

      // compute position of the reflected point
      const Point& worstPoint = points.back();
      reflectionPoint.position = centerOfGravity;
      for(int i = 0, count = centerOfGravity.size(); i < count; ++i)
        reflectionPoint.position[i] += centerOfGravity[i] - worstPoint.position[i];

      // adopt reflected point
      unratedPoints.push_back(&reflectionPoint);
      state = evaluateReflection;
      return next();
    }
  case evaluateReflection:
    {
      const Point& bestPoint = points.front();
      const Point& secondWorstPoint = points[points.size() - 2];

      if(reflectionPoint.rating < secondWorstPoint.rating && reflectionPoint.rating >= bestPoint.rating)
      {
        Point& worstPoint = points.back();
        worstPoint = reflectionPoint;
        state = computeReflection;
        return next();
      }

      if(reflectionPoint.rating < bestPoint.rating)
      {
        state = computeExpansion;
        return next();
      }

      else // reflectionPoint.rating >= secondWorstPoint.rating
      {
        state = computeContraction;
        return next();
      }
      return;
    }
  case computeExpansion:
    {
      const Point& worstPoint = points.back();
      expansionPoint.position = centerOfGravity;
      for(int i = 0, count = centerOfGravity.size(); i < count; ++i)
        expansionPoint.position[i] += 2.f * (centerOfGravity[i] - worstPoint.position[i]);
      unratedPoints.push_back(&expansionPoint);
      state = evaluateExpansion;
      return next();
    }
  case evaluateExpansion:
    {
      Point& worstPoint = points.back();
      if(expansionPoint.rating < reflectionPoint.rating)
      {
        worstPoint = expansionPoint;
        state = computeReflection;
        return next();
      }
      else
      {
        worstPoint = reflectionPoint;
        state = computeReflection;
        return next();
      }
    }
  case computeContraction:
    {
      const Point& worstPoint = points.back();
      contractionPoint.position = centerOfGravity;
      for(int i = 0, count = centerOfGravity.size(); i < count; ++i)
        contractionPoint.position[i] += 0.5f * (centerOfGravity[i] - worstPoint.position[i]);
      unratedPoints.push_back(&contractionPoint);
      state = evaluateContraction;
      return next();
    }
  case evaluateContraction:
    {
      Point& worstPoint = points.back();
      if(contractionPoint.rating < worstPoint.rating)
      {
        worstPoint = contractionPoint;
        state = computeReflection;
        return next();
      }
      else
      {
        state = computeReduction;
        return next();
      }
    }
  case computeReduction:
    {
      Point& bestPoint = points.front();
      for(Point* point = &points.front() + 1, * end = &points.front() + points.size(); point < end; ++point)
      {
        for(int i = 0, count = point->position.size(); i < count; ++i)
        {
          float d = 0.5f * (point->position[i] - bestPoint.position[i]);
          if(fabs(d) < dimensions[i].minDelta)
          {
            d = d > 0.f ? dimensions[i].minDelta : -dimensions[i].minDelta;
            //const float sign = rand() % 2 ? 1.f : -1.f;
            //d = dimensions[i].minDelta * sign;
          }
          point->position[i] = bestPoint.position[i] + d;
        }
        unratedPoints.push_back(point);
      }
      unratedPoints.push_back(&bestPoint);
      state = computeReflection;
      return next();
    }
  }
}
