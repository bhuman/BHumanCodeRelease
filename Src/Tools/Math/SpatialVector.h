/**
 * @file Vector.h
 * Contains template class SpatialVector of type V.
 *
 * @author Felix Wenk
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Matrix3x3.h"
#include "Vector3.h"

template <class V = float> class SpatialVector : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(top);
    STREAM(bottom);
    STREAM(motion);
    STREAM_REGISTER_FINISH;
  }

  SpatialVector<V>() : top(0.0f, 0.0f, 0.0f), bottom(0.0f, 0.0f, 0.0f), motion(true) {}
  SpatialVector<V>(const SpatialVector<V>& other) : top(other.top), bottom(other.bottom), motion(other.motion) {}
  SpatialVector<V>(const Vector3<V>& top, const Vector3<V>& bottom, bool motion = true) : top(top), bottom(bottom), motion(motion) {}

  SpatialVector<V>& operator=(const SpatialVector<V>& other)
  {
    top = other.top;
    bottom = other.bottom;
    motion = other.motion;
    return *this;
  }

  SpatialVector<V>& operator*=(const V factor)
  {
    top *= factor;
    bottom *= factor;
    return *this;
  }

  SpatialVector<V>& operator/=(const V factor)
  {
    top /= factor;
    bottom /= factor;
    return *this;
  }

  SpatialVector<V> operator*(const V factor) const
  {
    return SpatialVector<V>(*this) *= factor;
  }

  SpatialVector<V> operator/(const V factor)
  {
    return SpatialVector<V>(*this) /= factor;
  }

  SpatialVector<V>& operator-=(const SpatialVector<V>& other)
  {
    ASSERT(motion == other.motion);
    top -= other.top;
    bottom -= other.bottom;
    return *this;
  }

  SpatialVector<V>& operator+=(const SpatialVector<V>& other)
  {
    ASSERT(motion == other.motion);
    top += other.top;
    bottom += other.bottom;
    return *this;
  }

  SpatialVector<V> operator-(const SpatialVector<V>& other)
  {
    return SpatialVector<V>(*this) -= other;
  }

  SpatialVector<V> operator+(const SpatialVector<V>& other) const
  {
    return SpatialVector<V>(*this) += other;
  }

  V operator*(const SpatialVector<V>& other) const
  {
    ASSERT(motion != other.motion);
    return top * other.top + bottom * other.bottom;
  }

  SpatialVector<V> operator^(const SpatialVector<V>& other) const
  {
    ASSERT(motion);
    if(other.motion)
      return SpatialVector<V>(top ^ other.top, (top ^ other.bottom) + (bottom ^ other.top), true);
    else
      return SpatialVector<V>((top ^ other.top) + (bottom ^ other.bottom), top ^ other.bottom, false);
  }

  SpatialVector<V> operator-() const
  {
    return SpatialVector<V>(-top, -bottom, motion);
  }

  bool isFinite() const
  {
#ifndef WIN32
    for(int c = 0; c < 3; ++c)
    {
      ASSERT(std::isfinite(top[c]));
      ASSERT(std::isfinite(bottom[c]));
    }
#endif
    return true;
  }

  Vector3<> top;    /*< The top block of this vector */
  Vector3<> bottom; /*< The bottom block of this vector */
  bool motion;      /*< True if this is a spatial motion vector. */

  /**
   * Computes a matrix V of a vector v such that Vp = v x p.
   */
  static Matrix3x3<> crossProductMatrix(const Vector3<>& v)
  {
    return Matrix3x3<>(0.0f, -v.z, v.y,
                       v.z,  0.0f, -v.x,
                       -v.y, v.x,  0.0f);
  }
};

