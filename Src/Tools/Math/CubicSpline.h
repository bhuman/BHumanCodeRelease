#pragma once

#include "Eigen.h"
#include <vector>

class CubicSpline
{
private:
  struct CubicFunction
  {
    float ai;
    float bi;
    float ci;
    float di;
    float xi;

    CubicFunction() = default;
    CubicFunction(float ai, float bi, float ci, float di, float xi) :
      ai(ai), bi(bi), ci(ci), di(di), xi(xi)
    {}
    CubicFunction(const CubicFunction& other) = default;

    float eval(float x) const;
  };

  std::vector<CubicFunction> functions; // n cubic functions interpolating between the n + 1 control points.

public:
  CubicSpline() = default;
  CubicSpline(const std::vector<Vector2f>& controllPoints) { initNatural(controllPoints); };
  CubicSpline(const std::vector<Vector2f>& controllPoints, float startVelocity, float endVelocity) { initClamped(controllPoints, startVelocity, endVelocity); };
  CubicSpline(const CubicSpline& other) = default;

  /**
   * Initialize a new clamped cubic spline.
   */
  void initClamped(const std::vector<Vector2f>& controllPoints, float startVelocity, float endVelocity);

  /**
   * Initialize a new natural cubic spline.
   */
  void initNatural(const std::vector<Vector2f>& controllPoints);

  float operator()(float);
};
