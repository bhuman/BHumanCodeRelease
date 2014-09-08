/**
 * @file Vector4.h
 * Contains template class Vector4 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 * @editor Florian Maaß
 */

#pragma once

#include <cmath>
#include "Tools/Streams/Streamable.h"

/** This class represents a 4-vector */
template <class V = float> class Vector4 : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(x);
    STREAM(y);
    STREAM(z);
    STREAM(w);
    STREAM_REGISTER_FINISH;
  }

  /** The vector values */
  V x, y, z, w;

  /** Default constructor. */
  Vector4<V>() : x(V()), y(V()), z(V()), w(V()) {}

  /** Constructor. */
  Vector4<V>(V x, V y, V z, V w) : x(x), y(y), z(z), w(w) {}

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  Vector4<V>(const Vector4<V>& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}

  /** Copy constructor from different element type
   *\param O The type of other's elements
   *\param other The other vector that is copied to this one
   */
  template<typename O> inline explicit Vector4<V>(const Vector4<O>& other) : x((V) other.x), y((V) other.y), z((V) other.z), w((V) other.w) {}

  /** Assignment operator
  *\param other The other vector that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  Vector4<V>& operator=(const Vector4<V>& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;
    w = other.w;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A reference to this object after the calculation.
  */
  Vector4<V>& operator+=(const Vector4<V>& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
    return *this;
  }

  /** Substraction of this vector from another one.
  *\param other The other vector this one will be substracted from
  *\return A reference to this object after the calculation.
  */
  Vector4<V>& operator-=(const Vector4<V>& other)
  {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
    return *this;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A reference to this object after the calculation.
  */
  Vector4<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    z *= factor;
    w *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
  *\param factor The factor this vector is divided by
  *\return A reference to this object after the calculation.
  */
  Vector4<V>& operator/=(const V& factor)
  {
    if(factor == V())
      return *this;
    x /= factor;
    y /= factor;
    z /= factor;
    w /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector4<V> operator+(const Vector4<V>& other) const
  {
    return Vector4<V>(*this) += other;
  }

  /** Subtraction of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  Vector4<V> operator-(const Vector4<V>& other) const
  {
    return Vector4<V>(*this) -= other;
  }

  /** Negation of this vector.
  *\return A new object that contains the result of the calculation.
  */
  Vector4<V> operator-() const
  {
    return Vector4<V>(-x, -y, -z, -w);
  }

  /** Inner product of this vector and another one.
  *\param other The other vector this one will be multiplied by
  *\return The inner product.
  */
  V operator*(const Vector4<V>& other) const
  {
    return x * other.x + y * other.y + z * other.z + w * other.w;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A new object that contains the result of the calculation.
  */
  Vector4<V> operator*(const V& factor) const
  {
    return Vector4<V>(*this) *= factor;
  }

  /** Division of this vector by a factor.
  *
  *\param factor The factor this vector is divided by
  *\return A new object that contains the result of the calculation.
  */
  Vector4<V> operator/(const V& factor) const
  {
    return Vector4<V>(*this) /= factor;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  bool operator==(const Vector4<V>& other) const
  {
    return x == other.x && y == other.y && z == other.z && w == other.w;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  bool operator!=(const Vector4<V>& other) const
  {
    return x != other.x || y != other.y || z != other.z || w != other.w;
  }

  /**
  * array-like member access.
  * \param i index of coordinate
  * \return reference to x, y or z
  */
  V& operator[](int i)
  {
    return (&x)[i];
  }

  /**
  * const array-like member access.
  * \param i index of coordinate
  * \return reference to x or y
  */
  const V& operator[](int i) const
  {
    return (&x)[i];
  }

  /** Calculation of the length of this vector.
  *\return The length.
  */
  V abs() const
  {
    return (V) sqrt(float((x * x) + (y * y) + (z * z) + (w * w)));
  }

  /** Calculation of the square length of this vector.
  *\return length*length.
  */
  V squareAbs() const
  {
    return x * x + y * y + z * z + w * w;
  }

  //TODO: crossproduct for Vector4<> need to be implemented, maybe
  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A new object that contains the result of the calculation.
  */
  /*Vector4<V> operator^(const Vector4<V>& other) const
  {
    return Vector4<V>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }*/

  //TODO: crossproduct for Vector4<> need to be implemented, maybe
  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A reference to this object after the calculation.
  */
  /*Vector4<V>& operator^=(const Vector4<V>& other)
  {
    return *this = *this ^ other;
  }*/

  /** normalize this vector.
  *\param len The length, the vector should be normalized to, default=1.
  *\return the normalized vector.
  */
  Vector4<V>& normalize(V len)
  {
    const V length = abs();
    if(length == V())
      return *this;
    *this *= len;
    return *this /= length;
  }

  /** normalize this vector.
   *\return the normalized vector.
   */
  Vector4<V>& normalize()
  {
    return *this /= abs();
  }
};
