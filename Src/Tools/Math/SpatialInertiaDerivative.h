/**
 * @file SpatialInertiaDerivative.h
 * Contains class SpatialInertiaDerivative.
 *
 * If I is a SpatialInertia and v a motion vector, then the derivative is the following
 * expression: vx* I - I vx, where vx* is the cross product matrix of the cross product with a
 * force vector and vx is the cross product matrix for a cross product with a motion vector.
 *
 * TODO: This class probably has some room for performance improvements since it is implemented
 * as a 6x6 matrix whose 3x3 block on the lower right is always 0. (This reflects that the total
 * mass of the body is constant.)
 *
 * @author Felix Wenk
 */

#pragma once

#include "Platform/BHAssert.h"
#include "SpatialVector.h"
#include "Matrix3x3.h"

class SpatialInertiaDerivative : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(topLeft);
    STREAM(topRight);
    STREAM_REGISTER_FINISH;
  }

  SpatialInertiaDerivative() {}
  SpatialInertiaDerivative(const SpatialInertiaDerivative& other)
    : topLeft(other.topLeft), topRight(other.topRight)
  {}

  SpatialInertiaDerivative(const Matrix3x3<>& topLeft, const Matrix3x3<>& topRight)
    : topLeft(topLeft), topRight(topRight)
  {}

  SpatialInertiaDerivative& operator=(const SpatialInertiaDerivative& other)
  {
    topLeft = other.topLeft;
    topRight = other.topRight;
    return *this;
  }

  SpatialInertiaDerivative& operator+=(const SpatialInertiaDerivative& other)
  {
    topLeft += other.topLeft;
    topRight += other.topRight;
    return *this;
  }

  SpatialInertiaDerivative operator+(const SpatialInertiaDerivative& other) const
  {
    return SpatialInertiaDerivative(*this) += other;
  }

  SpatialVector<> operator*(const SpatialVector<>& vector) const
  {
    ASSERT(vector.motion);
    Matrix3x3<> bottomLeft = topRight * (-1.0f);
    return SpatialVector<>(topLeft * vector.top + topRight * vector.bottom,
                           bottomLeft * vector.top, false);
  }

  SpatialInertiaDerivative derive(const SpatialVector<>& vector) const
  {
    const Matrix3x3<> wx = SpatialVector<>::crossProductMatrix(vector.top);
    const Matrix3x3<> vx = SpatialVector<>::crossProductMatrix(vector.bottom);

    const Matrix3x3<> dTopLeft = wx * topLeft - vx * topRight - topLeft * wx - topRight * vx;
    const Matrix3x3<> dTopRight = wx * topRight - topRight * wx;

    return SpatialInertiaDerivative(dTopLeft, dTopRight);
  }

  bool isFinite() const
  {
#ifndef WIN32
    for(int c = 0; c < 3; ++c)
      for(int r = 0; r < 3; ++r)
      {
        ASSERT(std::isfinite(topLeft[c][r]));
        ASSERT(std::isfinite(topRight[c][r]));
      }
#endif
    return true;
  }

  Matrix3x3<> topLeft;
  Matrix3x3<> topRight;
  // Bottom left is -topRight
  // Bottom right is always 0.
};
