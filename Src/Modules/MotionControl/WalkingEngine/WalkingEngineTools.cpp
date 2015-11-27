/**
 * @file WalkingEngineTools.h
 * Implementation of tools utilized by the WalkingEngine
 * @author Colin Graf
 */

#include <cmath>

#include "WalkingEngineTools.h"
#include "Platform/BHAssert.h"

VectorYZ::VectorYZ(float y, float z) : y(y), z(z) {}

float FunctionMinimizer::minimize(float minPos, float maxPos, float startPos, float startPosDelta, float minVal, bool& clipped, const char* debugstr) const
{
  struct Point
  {
    float pos;
    float val;

    void setPos(float pos, float min, float max)
    {
      if(pos < min)
        this->pos = min;
      else if(pos > max)
        this->pos = max;
      else
        this->pos = pos;
    }
  };

  ASSERT(minPos <= maxPos);

  if(minPos == maxPos)
  {
    clipped = true;
    return minPos;
  }

  ASSERT(startPosDelta > 0.f);

  clipped = false;

  Point point[3];

  point[0].setPos(startPos, minPos, maxPos);
  startPos = point[0].pos;
  point[0].val = func(point[0].pos);
  if(point[0].val < minVal)
    return point[0].pos;

  point[1].setPos(startPos + startPosDelta, minPos, maxPos);
  if(point[0].pos == point[1].pos)
    point[1].setPos(startPos - startPosDelta, minPos, maxPos);
  point[1].val = func(point[1].pos);
  if(point[1].val < minVal)
    return point[1].pos;

  Point* smallest, *largest, *free = &point[2];
  if(point[0].val < point[1].val)
  {
    smallest = &point[0];
    largest = &point[1];
  }
  else
  {
    largest = &point[0];
    smallest = &point[1];
  }

  for(;;)
  {
    float delta = (smallest->pos - largest->pos);
    if(fabs(delta) <= 0.0001f)
    {
      clipped = true;
      return smallest->pos;
    }

    Point* reflection = free;
    reflection->setPos(smallest->pos + delta, minPos, maxPos);
    reflection->val = reflection->pos == smallest->pos ? largest->val : func(reflection->pos);

    if(reflection->val < smallest->val)
    {
      Point* expansion = largest;
      expansion->setPos(reflection->pos + delta, minPos, maxPos);
      expansion->val = expansion->pos == reflection->pos ? reflection->val : func(expansion->pos);

      if(expansion->val < reflection->val)
      {
        free = reflection;
        largest = smallest;
        smallest = expansion;
      }
      else
      {
        free = expansion;
        largest = smallest;
        smallest = reflection;
      }
    }
    else
    {
      Point* contraction = free;
      delta *= 0.5f;
      contraction->setPos(smallest->pos + delta, minPos, maxPos);
      contraction->val = contraction->pos == smallest->pos ? largest->val : func(contraction->pos);

      if(contraction->val < smallest->val)
      {
        free = largest;
        largest = smallest;
        smallest = contraction;
      }
      else if(contraction->val < largest->val)
      {
        free = largest;
        largest = contraction;
      }
      else
      {
        Point* reduction = free;
        reduction->setPos(smallest->pos - delta, minPos, maxPos);
        reduction->val = reduction->pos == smallest->pos ? largest->val : func(reduction->pos);

        if(reduction->val < smallest->val)
        {
          free = largest;
          largest = smallest;
          smallest = reduction;
        }
        else
        {
          free = largest;
          largest = reduction;
        }
      }
    }

    if(smallest->val < minVal)
      return smallest->pos;
  }
}
