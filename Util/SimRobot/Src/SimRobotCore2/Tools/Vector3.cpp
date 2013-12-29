/**
* @file Vector3.cpp
* Implementation of some functions that use auxiliar functions defined in headers that should not be included globally
* @author Colin Graf
*/

#include <cmath>

#include "Vector3.h"

template <class V> V Vector3<V>::abs() const
{
  return (V) sqrt(x * x + y * y + z * z);
}

template class Vector3<float>;
