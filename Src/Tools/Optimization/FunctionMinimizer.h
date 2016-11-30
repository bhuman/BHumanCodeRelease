/**
 * @file Tools/Optimization/FunctionMinimizer.h
 * A one-dimensional downhill simplex optimizer
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#pragma once

class FunctionMinimizer
{
public:
  struct Function
  {
    unsigned counter = 0;

  public:
    float evaluate(float val);

  protected:
    /**
     * The function to be minimized in the form f(x) = 0
     */
    virtual float function(float val) const = 0;
  };

  static float minimize(Function& function, float minVal, float maxVal, float startVal, float startDelta, float terminationCriterion, bool& clipped);
};
