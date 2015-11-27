#pragma once

/**
 * An one-dimensional downhill simplex optimizer
 */
class FunctionMinimizer
{
public:
  class Function
  {
  private:
    unsigned counter = 0;

  public:
    float evaluate(float val);

  protected:
    /**
     * The function to be minimized in the form f(x) = 0
     */
    virtual float function(float val) const = 0;
  };

  static float minimize(Function& function, float minVal, float maxVal, float start, float startDelta, float terminationCriterion, bool& clipped);
};
