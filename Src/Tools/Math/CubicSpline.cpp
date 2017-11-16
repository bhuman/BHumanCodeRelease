#include "CubicSpline.h"
#include "BHMath.h"

#include <algorithm>

float CubicSpline::CubicFunction::eval(float x) const
{
  const float hi = x - xi;
  const float sqrhi = sqr(hi);
  return ai * sqrhi * hi + bi * sqrhi + ci * hi + di;
}

void CubicSpline::initClamped(const std::vector<Vector2f>& controllPoints, float startVelocity, float endVelocity)
{
  functions.clear();
  const size_t numOfPoints = controllPoints.size();
  ASSERT(numOfPoints > 1);

  if(numOfPoints == 2)
  {
    const float y0 = controllPoints[0].y();
    const float y1 = controllPoints[1].y();
    const float x0 = controllPoints[0].x();
    const float yDiff = y1 - y0;
    const float h0 = controllPoints[1].x() - x0;
    const float sqrh0 = h0 * h0;
    const float cubh0 = sqrh0 * h0;
    const float b0 = 3 * yDiff / (3 * sqrh0 + 2 * cubh0) - (2 * startVelocity - endVelocity) / (3 * h0 + 2 * sqrh0);
    const float a0 = yDiff / cubh0 - startVelocity / sqrh0 - b0 / h0;

    functions.emplace_back(a0, b0, startVelocity, y0, x0);
  }
  else
  {
    MatrixXf A = MatrixXf::Zero(numOfPoints, numOfPoints);
    VectorXf v(numOfPoints);

    const size_t n = numOfPoints - 1;
    {
      const float h0 = controllPoints[1].x() - controllPoints[0].x();
      A(0, 0) = 2 * h0;
      A(0, 1) = h0;
      v(0) = (controllPoints[1].y() - controllPoints[0].y()) / h0 - startVelocity;

      const float hn_1 = controllPoints[n].x() - controllPoints[n - 1].x();
      A(n, n - 1) = hn_1;
      A(n, n) = 2 * hn_1;
      v(n) = endVelocity - (controllPoints[n].y() - controllPoints[n - 1].y()) / hn_1;
    }

    for(size_t i = 1; i < n; ++i)
    {
      const float hi_1 = controllPoints[i].x() - controllPoints[i - 1].x();
      const float hi = controllPoints[i + 1].x() - controllPoints[i].x();
      A(i, i - 1) = hi_1;
      A(i, i) = 2 * (hi_1 + hi);
      A(i, i + 1) = hi;

      v(i) = (controllPoints[i + 1].y() - controllPoints[i].y()) / hi - (controllPoints[i].y() - controllPoints[i - 1].y()) / hi_1;
    }
    v *= 3.f;

    Eigen::LLT<MatrixXf> llt = A.llt();
    ASSERT(llt.info() == Eigen::ComputationInfo::Success);
    const VectorXf b = llt.solve(v);

    for(size_t i = 0; i < numOfPoints - 1; ++i)
    {
      const float xi = controllPoints[i].x();
      const float yi = controllPoints[i].y();
      const float hi = controllPoints[i + 1].x() - xi;
      const float ci = (controllPoints[i + 1].y() - yi) / hi - hi * (b[i + 1] + 2 * b[i]) / 3;
      const float ai = (b[i + 1] - b[i]) / (3 * hi);

      functions.emplace_back(ai, b[i], ci, yi, xi);
    }
  }

  ASSERT(functions.size() == numOfPoints - 1);
}

void CubicSpline::initNatural(const std::vector<Vector2f>& controllPoints)
{
  const size_t numOfPoints = controllPoints.size();
  ASSERT(numOfPoints > 1);

  if(numOfPoints == 2)
  {
    const float y0 = controllPoints[0].y();
    const float x0 = controllPoints[0].x();
    const float c0 = (controllPoints[1].y() - y0) / (controllPoints[1].x() - x0);

    functions.emplace_back(0.f, 0.f, c0, y0, x0);
  }
  else
  {
    MatrixXf A = MatrixXf::Zero(numOfPoints, numOfPoints);
    VectorXf v(numOfPoints);

    const size_t n = numOfPoints - 1;

    A(0, 0) = A(n, n) = 1;
    v(0) = v(n) = 0;

    for(size_t i = 1; i < n; ++i)
    {
      const float hi_1 = controllPoints[i].x() - controllPoints[i - 1].x();
      const float hi = controllPoints[i + 1].x() - controllPoints[i].x();
      A(i, i - 1) = hi_1;
      A(i, i) = 2 * (hi_1 + hi);
      A(i, i + 1) = hi;

      v(i) = (controllPoints[i + 1].y() - controllPoints[i].y()) / hi - (controllPoints[i].y() - controllPoints[i - 1].y()) / hi_1;
    }
    v *= 3.f;

    Eigen::LLT<MatrixXf> llt = A.llt();
    ASSERT(llt.info() == Eigen::ComputationInfo::Success);
    const VectorXf b = llt.solve(v);

    for(size_t i = 0; i < numOfPoints - 1; ++i)
    {
      const float xi = controllPoints[i].x();
      const float yi = controllPoints[i].y();
      const float hi = controllPoints[i + 1].x() - xi;
      const float ci = (controllPoints[i + 1].y() - yi) / hi - hi * (b[i + 1] + 2 * b[i]) / 3;
      const float ai = (b[i + 1] - b[i]) / (3 * hi);

      functions.emplace_back(ai, b[i], ci, yi, xi);
    }
  }

  ASSERT(functions.size() == numOfPoints - 1);
}

float CubicSpline::operator()(float x)
{
  ASSERT(!functions.empty());

  auto iter = functions.begin();
  auto nextIter = iter + 1;
  auto endIter = functions.end();
  while(nextIter < endIter && x >= nextIter->xi)
    nextIter = (++iter) + 1;

  return iter->eval(x);
}
