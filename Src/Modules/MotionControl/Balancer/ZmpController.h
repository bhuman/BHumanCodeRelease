#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(ZmpControllerParameters,
{
  bool operator==(const ZmpControllerParameters& other) const;
  bool operator!=(const ZmpControllerParameters& other) const,

  (unsigned)(0) numOfZmpPreviews,
  (float)(0) R,
  (float)(0) Qe,
  (Vector3f)(Vector3f::Zero()) Qx,
  (bool)(true) useIntegrator,
});

class ZmpController
{
protected:
  float comHeight;
  Matrix3f A;
  Vector3f B;
  RowVector3f C;

private:
  ZmpControllerParameters params;
  ZmpControllerParameters lastParams;
  bool operational = false;

  float GI;
  RowVector3f Gx;
  RowVectorXf Gd;

  float integrationError = 0.f;

public:
  ZmpController();
  void init(float cycleTime);
  void reset(float comHeight, float cycleTime, ZmpControllerParameters params);
  Vector3f control(const Vector3f& inputState, const VectorXf& Zmps, float comHeight, ZmpControllerParameters params);

  void plot() const;

private:
  void calculateGains();
};
