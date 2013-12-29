/**
 * @file Vector3.h
 * Contains template class Vector3 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include "Common.h"
#include "Tools/Streams/Streamable.h"

/** This class represents a 3-vector */
template <class V = float> class Vector3 : public Streamable
{
public:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(x);
    STREAM(y);
    STREAM(z);
    STREAM_REGISTER_FINISH;
  }

  /** The vector values */
  V x, y, z;

  /** Default constructor. */
  inline Vector3<V>() : x(V()), y(V()), z(V()) {}

  /** Constructor. */
  inline Vector3<V>(V x, V y, V z) : x(x), y(y), z(z) {}

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  inline Vector3<V>(const Vector3<V>& other) : x(other.x), y(other.y), z(other.z) {}

  /** Copy constructor from different element type
   *\param O The type of other's elements
   *\param other The other vector that is copied to this one
   */
  template<typename O> inline explicit Vector3<V>(const Vector3<O>& other) : x((V) other.x), y((V) other.y), z((V) other.z) {}

  /** Assignment operator
  *\param other The other vector that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  inline Vector3<V>& operator=(const Vector3<V>& other)
  {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A reference to this object after the calculation.
  */
  inline Vector3<V>& operator+=(const Vector3<V>& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  /** Substraction of this vector from another one.
  *\param other The other vector this one will be substracted from
  *\return A reference to this object after the calculation.
  */
  inline Vector3<V>& operator-=(const Vector3<V>& other)
  {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A reference to this object after the calculation.
  */
  inline Vector3<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    z *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
  *\param factor The factor this vector is divided by
  *\return A reference to this object after the calculation.
  */
  inline Vector3<V>& operator/=(const V& factor)
  {
    if(factor == V()) return *this;
    x /= factor;
    y /= factor;
    z /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator+(const Vector3<V>& other) const
  {
    return Vector3<V>(*this) += other;
  }

  /** Subtraction of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator-(const Vector3<V>& other) const
  {
    return Vector3<V>(*this) -= other;
  }

  /** Negation of this vector.
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator-() const
  {
    return Vector3<V>(-x, -y, -z);
  }

  /** Inner product of this vector and another one.
  *\param other The other vector this one will be multiplied by
  *\return The inner product.
  */
  inline V operator*(const Vector3<V>& other) const
  {
    return x * other.x + y * other.y + z * other.z;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator*(const V& factor) const
  {
    return Vector3<V>(*this) *= factor;
  }

  /** Division of this vector by a factor.
  *
  *\param factor The factor this vector is divided by
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator/(const V& factor) const
  {
    return Vector3<V>(*this) /= factor;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  inline bool operator==(const Vector3<V>& other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are unequal.
  */
  inline bool operator!=(const Vector3<V>& other) const
  {
    return x != other.x || y != other.y || z != other.z;
  }

  /**
  * array-like member access.
  * \param i index of coordinate
  * \return reference to x, y or z
  */
  inline V& operator[](int i)
  {
    return (&x)[i];
  }

  /**
  * const array-like member access.
  * \param i index of coordinate
  * \return reference to x or y
  */
  inline const V& operator[](int i) const
  {
    return (&x)[i];
  }

  /** Calculation of the length of this vector.
  *\return The length.
  */
  inline V abs() const
  {
    return (V) sqrt(float((x * x) + (y * y) + (z * z)));
  }

  /** Calculation of the square length of this vector.
  *\return length*length.
  */
  inline V squareAbs() const
  {
    return x * x + y * y + z * z;
  }

  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A new object that contains the result of the calculation.
  */
  inline Vector3<V> operator^(const Vector3<V>& other) const
  {
    return Vector3<V>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }

  /** Crossproduct of this vector and another vector.
  *\param other The factor this vector is multiplied with.
  *\return A reference to this object after the calculation.
  */
  inline Vector3<V>& operator^=(const Vector3<V>& other)
  {
    return *this = *this ^ other;
  }

  /** normalize this vector.
  *\param len The length, the vector should be normalized to, default=1.
  *\return the normalized vector.
  */
  Vector3<V>& normalize(V len)
  {
    const V length = abs();
    if(length == V()) return *this;
    *this *= len;
    return *this /= length;
  }

  /** normalize this vector.
   *\return the normalized vector.
   */
  Vector3<V>& normalize()
  {
    return *this /= abs();
  }
};

/**
* Streaming operator that reads a Vector3<V> from a stream.
* @param stream The stream from which is read.
* @param vector3 The Vector3<V> object.
* @return The stream.
*/
/*
template <class V> In& operator>>(In& stream, Vector3<V>& vector3)
{
  stream >> vector3.x;
  stream >> vector3.y;
  stream >> vector3.z;
  return stream;
}
*/
/**
* Streaming operator that writes a Vector3<V> to a stream.
* @param stream The stream to write on.
* @param vector3 The Vector3<V> object.
* @return The stream.
*/
/*
template <class V> Out& operator<<(Out& stream, const Vector3<V>& vector3)
{
  stream << vector3.x;
  stream << vector3.y;
  stream << vector3.z;
  return stream;
}
*/
