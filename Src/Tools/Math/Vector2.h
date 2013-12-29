/**
 * @file Vector2.h
 * Contains template class Vector2 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include "Common.h"
#include "Tools/Streams/Streamable.h"

/** This class represents a 2-vector */
template <class V = float> class Vector2 : public ImplicitlyStreamable
{
public:
  /** The vector values */
  V x, y;

  /** Default constructor. */
  inline Vector2<V>() : x(V()) , y(V()) {}

  /** Constructor. */
  inline Vector2<V>(V x, V y) : x(x), y(y) {}

  /** Copy constructor
  *\param other The other vector that is copied to this one
  */
  inline Vector2<V>(const Vector2<V>& other) : x(other.x), y(other.y) {}

  /** Copy constructor from different element type
   *\param O The type of other's elements
   *\param other The other vector that is copied to this one
   */
  template<typename O> inline explicit Vector2<V>(const Vector2<O>& other) : x((V) other.x), y((V) other.y) {}

  /** Assignment operator
  *\param other The other vector that is assigned to this one
  *\return A reference to this object after the assignment.
  */
  inline Vector2<V>& operator=(const Vector2<V>& other)
  {
    x = other.x;
    y = other.y;
    return *this;
  }


  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A reference to this object after the calculation.
  */
  inline Vector2<V>& operator+=(const Vector2<V>& other)
  {
    x += other.x;
    y += other.y;
    return *this;
  }

  /** Substraction of this vector from another one.
  *\param other The other vector this one will be substracted from
  *\return A reference to this object after the calculation.
  */
  inline Vector2<V>& operator-=(const Vector2<V>& other)
  {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A reference to this object after the calculation.
  */
  inline Vector2<V>& operator*=(const V& factor)
  {
    x *= factor;
    y *= factor;
    return *this;
  }

  /** Division of this vector by a factor.
  *\param factor The factor this vector is divided by
  *\return A reference to this object after the calculation.
  */
  inline Vector2<V>& operator/=(const V& factor)
  {
    if(factor == V()) return *this;
    x /= factor;
    y /= factor;
    return *this;
  }

  /** Addition of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  inline Vector2<V> operator+(const Vector2<V>& other) const
  {
    return Vector2<V>(*this) += other;
  }

  /** Subtraction of another vector to this one.
  *\param other The other vector that will be added to this one
  *\return A new object that contains the result of the calculation.
  */
  inline Vector2<V> operator-(const Vector2<V>& other) const
  {
    return Vector2<V>(*this) -= other;
  }

  /** Negation of this vector.
  *\return A new object that contains the result of the calculation.
  */
  inline Vector2<V> operator-() const
  {
    return Vector2<V>(-x, -y);
  }

  /** Inner product of this vector and another one.
  *\param other The other vector this one will be multiplied by
  *\return The inner product.
  */
  inline V operator*(const Vector2<V>& other) const
  {
    return x * other.x + y * other.y;
  }

  /** Multiplication of this vector by a factor.
  *\param factor The factor this vector is multiplied by
  *\return A new object that contains the result of the calculation.
  */
  inline Vector2<V> operator*(const V& factor) const
  {
    return Vector2<V>(*this) *= factor;
  }

  /** Division of this vector by a factor.
  *
  *\param factor The factor this vector is divided by
  *\return A new object that contains the result of the calculation.
  */
  inline Vector2<V> operator/(const V& factor) const
  {
    return Vector2<V>(*this) /= factor;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one
  *\return Whether the two vectors are equal.
  */
  inline bool operator==(const Vector2<V>& other) const
  {
    return x == other.x && y == other.y;
  }

  /** Comparison of another vector with this one.
  *\param other The other vector that will be compared to this one.
  *\return Whether the two vectors are unequal.
  */
  inline bool operator!=(const Vector2<V>& other) const
  {
    return x != other.x || y != other.y;
  }

  /** Calculation of the length of this vector.
  *\return The length.
  */
  inline V abs() const
  {
    return (V) std::sqrt(((float)x) * x + ((float)y) * y);
  }

  /**
   * Calculation of the length of this vector
   * @return
   */
  inline float absFloat() const
  {
    return std::sqrt(((float)x) * x + ((float)y) * y);
  }

  /** Calculation of the square length of this vector.
  *\return length*length.
  */
  inline V squareAbs() const
  {
    return x * x + y * y;
  }

  /**
  * Calculation of the square of the vector.
  * \return The square
  */
  inline V sqr() const
  {
    return x * x + y * y;
  }

  /** normalize this vector.
  *\param len The length, the vector should be normalized to, default=1.
  *\return the normalized vector.
  */
  Vector2<V>& normalize(V len)
  {
    const V length = abs();
    if(length == V()) return *this;
    *this *= len;
    return *this /= length;
  }

  /** normalize this vector.
  *\return the normalized vector.
  */
  Vector2<V>& normalize()
  {
    return *this /= abs();
  }

  /** the vector is rotated left by 90 degrees.
  *\return the rotated vector.
  */
  Vector2<V>& rotateLeft()
  {
    V buffer = -y;
    y = x;
    x = buffer;
    return *this;
  }

  /** the vector is rotated right by 90 degrees.
  *\return the rotated vector.
  */
  Vector2<V>& rotateRight()
  {
    V buffer = -x;
    x = y;
    y = buffer;
    return *this;
  }

    /** the vector is rotated by 180 degrees.
  *\return the rotated vector.
  */
  Vector2<V>& mirror()
  {
    x = -x;
    y = -y;
    return *this;
  }

  /** the vector is rotated by alpha degrees.
  *\return the rotated vector.
  */
  Vector2<V>& rotate(float alpha)
  {
    float buffer = (float) x;
    float a = std::cos(alpha);
    float b = std::sin(alpha);
    x = (V)(a * (float)x - b * (float)y);
    y = (V)(b * buffer + a * (float)y);
    return *this;
  }

  /**
  * array-like member access.
  * \param i index of coordinate
  * \return reference to x or y
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

  /** Calculation of the angle of this vector */
  inline float angle() const
  {
    return std::atan2((float)y, (float)x);
  }
};

/**
* Streaming operator that writes a Vector2 to a stream.
* @param out The stream to write on.
* @param vector The Vector2 object.
* @return The stream.
*/
template <class T>
Out& operator<<(Out& out, const Vector2<T>& vector)
{
  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(out, vector.x);
  STREAM_EXT(out, vector.y);
  STREAM_REGISTER_FINISH;
  return out;
}

/**
* Streaming operator that reads a Vector2 from a stream.
* @param in The stream from which is read.
* @param vector The Vector2 object.
* @return The stream.
*/
template <class T>
In& operator>>(In& in, Vector2<T>& vector)
{
  STREAM_REGISTER_BEGIN_EXT(vector);
  STREAM_EXT(in, vector.x);
  STREAM_EXT(in, vector.y);
  STREAM_REGISTER_FINISH;
  return in;
}
