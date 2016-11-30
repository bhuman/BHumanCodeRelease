/**
 * @file Tools/Optimization/FunctionMinimizer.cpp
 * Implementaiton of a one-dimensional downhill simplex optimizer
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#include "FunctionMinimizer.h"
#include "Platform/BHAssert.h"
#include <cmath>

float FunctionMinimizer::Function::evaluate(float val)
{
  ++counter;
  return std::abs(function(val));
}

float FunctionMinimizer::minimize(Function& function, float minVal, float maxVal, float startVal, float startDelta, float terminationCriterion, bool& clipped)
{
  struct Point
  {
    float val;
    float error;

    void setVal(float val, float min, float max)
    {
      if(val < min)
        this->val = min;
      else if(val > max)
        this->val = max;
      else
        this->val = val;
    }
  };

  ASSERT(minVal <= maxVal);

  if(minVal == maxVal)
  {
    clipped = true;
    return minVal;
  }

  ASSERT(startDelta > 0.f);

  clipped = false;

  Point point[3];

  point[0].setVal(startVal, minVal, maxVal);
  startVal = point[0].val;
  point[0].error = function.evaluate(point[0].val);
  if(point[0].error < terminationCriterion)
    return point[0].val;

  point[1].setVal(startVal + startDelta, minVal, maxVal);
  if(point[0].val == point[1].val)
    point[1].setVal(startVal - startDelta, minVal, maxVal);
  point[1].error = function.evaluate(point[1].val);
  if(point[1].error < terminationCriterion)
    return point[1].val;

  Point* smallest, *largest, *free = &point[2];
  if(point[0].error < point[1].error)
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
    float delta = (smallest->val - largest->val);
    if(fabs(delta) <= 0.0001f)
    {
      clipped = true;
      return smallest->val;
    }

    Point* reflection = free;
    reflection->setVal(smallest->val + delta, minVal, maxVal);
    reflection->error = reflection->val == smallest->val ? largest->error : function.evaluate(reflection->val);

    if(reflection->error < smallest->error)
    {
      Point* expansion = largest;
      expansion->setVal(reflection->val + delta, minVal, maxVal);
      expansion->error = expansion->val == reflection->val ? reflection->error : function.evaluate(expansion->val);

      if(expansion->error < reflection->error)
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
      contraction->setVal(smallest->val + delta, minVal, maxVal);
      contraction->error = contraction->val == smallest->val ? largest->error : function.evaluate(contraction->val);

      if(contraction->error < smallest->error)
      {
        free = largest;
        largest = smallest;
        smallest = contraction;
      }
      else if(contraction->error < largest->error)
      {
        free = largest;
        largest = contraction;
      }
      else
      {
        Point* reduction = free;
        reduction->setVal(smallest->val - delta, minVal, maxVal);
        reduction->error = reduction->val == smallest->val ? largest->error : function.evaluate(reduction->val);

        if(reduction->error < smallest->error)
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

    if(smallest->error < terminationCriterion)
      return smallest->val;
  }
}
