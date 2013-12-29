/**
* @file Matrix3x3.cpp
* Implementation of some Matrix3x3 function
* @author Colin Graf
*/

#include <cmath>

#include "Matrix3x3.h"

template <class V> Matrix3x3<V>::Matrix3x3(const Vector3<V>& a)
{
  const V angle = a.abs();
  Vector3<V> axis = a;
  if(angle != V())
    axis /= angle; // normalize a, rotation is only possible with unit vectors
  const V &x = axis.x, &y = axis.y, &z = axis.z;
  //compute sine and cosine of angle because it is needed quite often for complete matrix
  const V si = (V) sin(angle), co = (V) cos(angle);
  //compute all components needed more than once for complete matrix
  const V v = 1 - co;
  const V xyv = x * y * v;
  const V xzv = x * z * v;
  const V yzv = y * z * v;
  const V xs = x * si;
  const V ys = y * si;
  const V zs = z * si;
  //compute matrix
  c0.x = x * x * v + co; c1.x = xyv - zs;       c2.x = xzv + ys;
  c0.y = xyv + zs;       c1.y = y * y * v + co; c2.y = yzv - xs;
  c0.z = xzv - ys;       c1.z = yzv + xs;       c2.z = z * z * v + co;
}

template class Matrix3x3<float>;

