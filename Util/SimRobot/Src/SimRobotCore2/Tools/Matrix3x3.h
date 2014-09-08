/**
* @file Matrix3x3.h
* Contains template class Matrix3x3 of type V
* @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "Vector3.h"

/**
 * This class represents a 3x3-matrix
 *
 */
template <class V = float> class Matrix3x3
{
public:
  /**
   * The columns of the matrix
   */
  Vector3<V> c0;
  Vector3<V> c1;
  Vector3<V> c2;

  /**
   * Default constructor.
   */
  Matrix3x3<V>() : c0(1,0,0), c1(0,1,0), c2(0,0,1) {}

  /**
   * Constructor.
   *
   * \param  c0  the first column of the matrix.
   * \param  c1  the second column of the matrix.
   * \param  c2  the third column of the matrix.
  */
  Matrix3x3<V>(const Vector3<V>& c0, const Vector3<V>& c1,  const Vector3<V>& c2) : c0(c0), c1(c1), c2(c2) {}

  /**
   * Copy constructor.
   *
   * \param other The other matrix that is copied to this one
   */
  Matrix3x3<V>(const Matrix3x3<V>& other) : c0(other.c0), c1(other.c1), c2(other.c2) {}

  /**
   * RotationMatrix from rotation around any axis with an angle given as the length of the axis.
   * \param axis The axis.
   */
  Matrix3x3<V>(const Vector3<V>& axis);

  /**
   * Assignment operator.
   *
   * \param  other   The other matrix that is assigned to this one
   * \return         A reference to this object after the assignment.
  */
  Matrix3x3<V>& operator=(const Matrix3x3<V>& other)
  {
    c0 = other.c0;
    c1 = other.c1;
    c2 = other.c2;
    return *this;
  }

  /**
   * Adds this matrix with another matrix.
   *
   * \param  other  The matrix this one is added to
   * \return         A new matrix containing the result
   *                 of the calculation.
  */
  Matrix3x3<V> operator+(const Matrix3x3<V>& other) const
  {
    return Matrix3x3<V>(
      c0 + other.c0,
      c1 + other.c1,
      c2 + other.c2
    );
  }
   /**
   * Adds another matrix to this matrix.
   *
   * \param  other  The other matrix that is added to this one
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator+=(const Matrix3x3<V>& other)
  {
    c0 += other.c0;
    c1 += other.c1;
    c2 += other.c2;
    return *this;
  }

  /**
   * Compute difference of this matrix and another one
   *
   * \param  other  The matrix which is substracted from this one
   * \return         A new matrix containing the result
   *                 of the calculation.
  */
  Matrix3x3<V> operator-(const Matrix3x3<V>& other) const
  {
    return Matrix3x3<V>(
      c0 - other.c0,
      c1 - other.c1,
      c2 - other.c2
    );
  }
   /**
   * Substracts another matrix from this one
   *
   * \param  other  The other matrix that is substracted from this one
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator-=(const Matrix3x3<V>& other)
  {
    c0 -= other.c0;
    c1 -= other.c1;
    c2 -= other.c2;
    return *this;
  }

  /**
   * Multiplication of this matrix by vector.
   *
   * \param  vector  The vector this one is multiplied by
   * \return         A new vector containing the result
   *                 of the calculation.
  */
  Vector3<V> operator*(const Vector3<V>& vector) const
  {
    /*
    return c0 * vector.x + c1 * vector.y + c2 * vector.z;
    */
    return Vector3<V>(
      c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
      c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
      c0.z * vector.x + c1.z * vector.y + c2.z * vector.z);
  }

  /**
   * Multiplication of this matrix by another matrix.
   *
   * \param  other  The other matrix this one is multiplied by
   * \return        A new matrix containing the result
   *                of the calculation.
  */
  Matrix3x3<V> operator*(const Matrix3x3<V>& other) const
  {
    // this method is up to 2 times faster than "return Matrix3x3<V>((*this) * other.c0, (*this) * other.c1, (*this) * other.c2);"
    Matrix3x3<V> result;
    result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
    result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
    result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
    result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
    result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
    result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
    result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
    result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
    result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
    return result;
  }

  /**
   * Multiplication of this matrix by another matrix.
   *
   * \param  other  The other matrix this one is multiplied by
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator*=(const Matrix3x3<V>& other)
  {
    // this method is somehow faster than "return *this = *this * other;"
    Matrix3x3<V> result;
    result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
    result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
    result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
    result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
    result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
    result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
    result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
    result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
    result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
    *this = result;
    return *this;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by
   * \return         A reference to this object after the calculation.
  */
  Matrix3x3<V>& operator*=(const V& factor)
  {
    c0 *= factor;
    c1 *= factor;
    c2 *= factor;
    return *this;
  }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by
   * \return         A reference to this object after the calculation.
   */
  Matrix3x3<V>& operator/=(const V& factor)
  {
    const V invFactor = V(1) / factor;
    c0 *= invFactor;
    c1 *= invFactor;
    c2 *= invFactor;
    return *this;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator*(const V& factor) const
  {
    return Matrix3x3<V>(c0 * factor, c1 * factor, c2 * factor);
  }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator/(const V& factor) const
  {
    const V invFactor = V(1) / factor;
    return Matrix3x3<V>(c0 * invFactor, c1 * invFactor, c2 * invFactor);
  }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrices are equal.
   */
  bool operator==(const Matrix3x3<V>& other) const
  {
    return c0 == other.c0 && c1 == other.c1 && c2 == other.c2;
  }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrixs are unequal.
   */
  bool operator!=(const Matrix3x3<V>& other) const
  {
    return c0 != other.c0 || c1 != other.c1 || c2 != other.c2;
  }

  /**
   * Array-like member access.
   * \param  i index
   * \return reference to column
   */
  Vector3<V>& operator[](int i)
  {
    return (&c0)[i];
  }

  /**
   * const array-like member access.
   * \param  i index
   * \return reference to column
   */
  const Vector3<V>& operator[](int i) const
  {
    return (&c0)[i];
  }

  /**
   * Transpose the matrix
   *
   * \return  A new object containing transposed matrix
   */
  Matrix3x3<V> transpose() const
  {
    return Matrix3x3<V>(
      Vector3<V>(c0.x, c1.x, c2.x),
      Vector3<V>(c0.y, c1.y, c2.y),
      Vector3<V>(c0.z, c1.z, c2.z)
    );
  }
};
