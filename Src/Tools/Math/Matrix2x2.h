/**
 * @file Matrix2x2.h
 * Contains template class Matrix2x2 of type V
 *
 * @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Vector2.h"
#include <limits>

/** This class represents a 2x2-matrix */
template <class V = float> class Matrix2x2  : public Streamable
{
private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(c);
    STREAM_REGISTER_FINISH;
  }

public:
  /** The columns of the matrix */
  Vector2<V> c[2];

  /** Default constructor. */
  Matrix2x2<V>()
  {
    c[0] = Vector2<V>(1, 0);
    c[1] = Vector2<V>(0, 1);
  }

  /**
   * Anti-lateral thinking constructor.
   */
  Matrix2x2<V>(
    const V& a11, const V& a12,
    const V& a21, const V& a22)
  {
    c[0].x = a11;
    c[1].x = a12;
    c[0].y = a21;
    c[1].y = a22;
  }

  //! Constructor
  /*!
  \param c0 the first column of the matrix.
  \param c1 the second column of the matrix.
  */
  Matrix2x2<V>(const Vector2<V>& c0, const Vector2<V>& c1)
  {
    c[0] = c0;
    c[1] = c1;
  }

  //! Assignment operator
  /*!
  \param other The other matrix that is assigned to this one
  \return A reference to this object after the assignment.
  */
  Matrix2x2<V>& operator=(const Matrix2x2<V>& other)
  {
    c[0] = other.c[0];
    c[1] = other.c[1];
    return *this;
  }

  //! Copy constructor
  /*!
  \param other The other matrix that is copied to this one
   */
  Matrix2x2<V>(const Matrix2x2<V>& other)
  {
    *this = other;
  }

  /**
   * Array-like member access.
   * \param  i index
   * \return reference to column
   */
  Vector2<V>& operator[](int i)
  {
    return c[i];
  }

  /**
   * const array-like member access.
   * \param  i index
   * \return reference to column
   */
  const Vector2<V>& operator[](int i) const
  {
    return c[i];
  }

  //! Multiplication of this matrix by vector.
  /*!
  \param vector The vector this one is multiplied by
  \return A reference to a new vector containing the result
    of the calculation.
  */
  Vector2<V> operator*(const Vector2<V>& vector) const
  {
    return (c[0] * vector.x + c[1] * vector.y);
  }

  //! Multiplication of this matrix by another matrix.
  /*!
  \param other The other matrix this one is multiplied by
  \return An object containing the result of the calculation.
  */
  Matrix2x2<V> operator*(const Matrix2x2<V>& other) const
  {
    Matrix2x2<V> returnMatrix;
    returnMatrix.c[0].x = c[0].x * other.c[0].x + c[1].x * other.c[0].y;
    returnMatrix.c[0].y = c[0].y * other.c[0].x + c[1].y * other.c[0].y;
    returnMatrix.c[1].x = c[0].x * other.c[1].x + c[1].x * other.c[1].y;
    returnMatrix.c[1].y = c[0].y * other.c[1].x + c[1].y * other.c[1].y;
    return returnMatrix;
  }

  //! Multiplication of this matrix by another matrix.
  /*!
  \param other The other matrix this one is multiplied by
  \return A reference this object after the calculation.
  */
  Matrix2x2<V> operator*=(const Matrix2x2<V>& other)
  {
    return *this = *this * other;
  }

  //! Multiplication of this matrix by a factor.
  /*!
  \param factor The factor this matrix is multiplied by
  \return A reference to this object after the calculation.
  */
  Matrix2x2<V>& operator*=(const V& factor)
  {
    c[0] *= factor;
    c[1] *= factor;
    return *this;
  }

  //! Division of this matrix by a factor.
  /*!
  \param factor The factor this matrix is divided by
  \return A reference to this object after the calculation.
   */
  Matrix2x2<V>& operator/=(const V& factor)
  {
    c[0] /= factor;
    c[1] /= factor;
    return *this;
  }

  //! Multiplication of this matrix by a factor.
  /*!
  \param factor The factor this matrix is multiplied by
  \return A new object that contains the result of the calculation.
  */
  Matrix2x2<V> operator*(const V& factor) const
  {
    return Matrix2x2<V>(*this) *= factor;
  }

  //! Division of this matrix by a factor.
  /*!
  \param factor The factor this matrix is divided by
  \return A new object that contains the result of the calculation.
  */
  Matrix2x2<V> operator/(const V& factor) const
  {
    return Matrix2x2<V>(*this) /= factor;
  }

  //! Computes the sum of two matrices
  /*!
  \param other Another matrix
  \return The sum
  */
  Matrix2x2<V> operator+(const Matrix2x2<V>& other) const
  {
    return Matrix2x2<V>(Vector2<V>(c[0].x + other.c[0].x, c[0].y + other.c[0].y),
                        Vector2<V>(c[1].x + other.c[1].x, c[1].y + other.c[1].y));
  }

  //! Computes the difference of two matrices
  /*!
  \param other Another matrix
  \return The difference
  */
  Matrix2x2<V> operator-(const Matrix2x2<V>& other) const
  {
    return Matrix2x2<V>(Vector2<V>(c[0].x - other.c[0].x, c[0].y - other.c[0].y),
                        Vector2<V>(c[1].x - other.c[1].x, c[1].y - other.c[1].y));
  }

  /**
   * Adds another matrix.
   *
   * \param  other  The other matrix that is added to this one
   * \return        A reference to this object after the calculation.
  */
  Matrix2x2<V>& operator+=(const Matrix2x2<V>& other)
  {
    c[0] += other.c[0];
    c[1] += other.c[1];
    return *this;
  }

  /**
   * Subtracts another matrix.
   *
   * \param  other  The other matrix that is subtracted from this one
   * \return        A reference to this object after the calculation.
  */
  Matrix2x2<V>& operator-=(const Matrix2x2<V>& other)
  {
    c[0] -= other.c[0];
    c[1] -= other.c[1];
    return *this;
  }

  //! Computes an inverted matrix.
  /*!
  \return An inverted matrix.
  */
  Matrix2x2<V> invert() const
  {
    V factor(det());
    if(std::abs(factor) < std::numeric_limits<V>::min())
      factor = std::numeric_limits<V>::min();
    else
      factor = 1.f / factor;
    return Matrix2x2<V>(Vector2<V>(factor * c[1].y, -factor * c[0].y),
                        Vector2<V>(-factor * c[1].x, factor * c[0].x));
  }

  //! Comparison of another matrix with this one.
  /*!
  \param other The other matrix that will be compared to this one
  \return Whether the two matrices are equal.
  */
  bool operator==(const Matrix2x2<V>& other) const
  {
    return (c[0] == other.c[0] && c[1] == other.c[1]);
  }

  //! Comparison of another matrix with this one.
  /*!
  \param other The other matrix that will be compared to this one
  \return Whether the two matrixs are unequal.
  */
  bool operator!=(const Matrix2x2<V>& other) const
  {
    return !(*this == other);
  }

  /*! Transpose the matrix
  \return A new object containing transposed matrix
  */
  Matrix2x2<V> transpose() const
  {
    return Matrix2x2<V>(Vector2<V>(c[0].x, c[1].x),
                        Vector2<V>(c[0].y, c[1].y));
  }

  //! Calculation of the determinant of this matrix.
  /*!
  \return The determinant.
  */
  V det() const
  {
    return c[0].x * c[1].y - c[1].x * c[0].y;
  }

  /** Sums up the elements on the main diagonal
  * @return The sum
  */
  V trace() const
  {
    return c[0].x + c[1].y;
  }
};
