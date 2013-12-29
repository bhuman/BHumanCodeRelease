/**
* @file Matrix.h
* Contains template class Matrix of type V and size mxn
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
* @author Colin Graf
*/

#pragma once

#include "Vector.h"

/** This class represents a mxn-matrix */
template <int m = 2, int n = 2, class V = float> class Matrix
{
public:
  /** The columns of the matrix */
  Vector<m, V> c[n];

  /** Default constructor. */
  Matrix<m, n, V>() {}

  /**
  * Constructor that initializes the diagonal of nxn matrices with a value
  * @param v The value
  */
  Matrix<m, n, V>(V v)
  {
    ASSERT(m == n);
    const int mnm = n < m ? n : m;
    for(int i = 0; i < mnm; ++i)
      c[i][i] = v;
  }

  /** Constructor */
  Matrix<m, n, V>(const Vector<m, V>& c0)
  {
    ASSERT(n == 1);
    c[0] = c0;
  }

  /** Constructor */
  Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1)
  {
    ASSERT(n == 2);
    c[0] = c0;
    c[1] = c1;
  }

  /** Constructor */
  Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1, const Vector<m, V>& c2)
  {
    ASSERT(n == 3);
    c[0] = c0;
    c[1] = c1;
    c[2] = c2;
  }

  /** Constructor */
  Matrix<m, n, V>(const Vector<m, V>& c0, const Vector<m, V>& c1, const Vector<m, V>& c2, const Vector<m, V>& c3)
  {
    ASSERT(n == 4);
    c[0] = c0;
    c[1] = c1;
    c[2] = c2;
    c[3] = c3;
  }

  /**
  * Assignment operator
  * @param other The other matrix that is assigned to this one
  * @return A reference to this object after the assignment.
  */
  Matrix<m, n, V>& operator=(const Matrix<m, n, V>& other)
  {
    for(int i = 0; i < n; ++i)
      c[i] = other.c[i];
    return *this;
  }

  /**
  * Array-like member access.
  * @param i index
  * @return reference to column
  */
  inline Vector<m, V>& operator[](int i)
  {
    return c[i];
  }

  /**
  * const array-like member access.
  * @param i index
  * @return reference to column
  */
  inline const Vector<m, V>& operator[](int i) const
  {
    return c[i];
  }

  /**
  * Multiplication of this matrix by vector.
  * @param vector The vector this one is multiplied by
  * @return A reference to a new vector containing the result of the calculation.
  */
  Vector<m, V> operator*(const Vector<n, V>& vector) const
  {
    Vector<m, V> result = c[0] * vector[0];
    for(int i = 1; i < n; ++i)
      result += c[i] * vector[i];
    return result;
  }

  /**
  * Multiplication of this matrix by another matrix.
  * @param other The other matrix this one is multiplied by
  * @return An object containing the result of the calculation.
  */
  template<int o> Matrix<m, o, V> operator*(const Matrix<n, o, V>& other) const
  {
    Matrix<m, o, V> result;
    for(int i = 0; i < n; ++i)
      for(int j = 0; j < m; ++j)
        for(int k = 0; k < o; ++k)
          result.c[k][j] += c[i][j] * other.c[k][i];
    return result;
  }

  /**
  * Multiplication of this matrix by another matrix.
  * @param other The other matrix this one is multiplied by
  * @return A reference this object after the calculation.
  */
  Matrix<n, n, V> operator*=(const Matrix<n, n, V>& other)
  {
    return *this = *this * other;
  }

  /**
  * Multiplication of this matrix by a factor.
  * @param factor The factor this matrix is multiplied by
  * @return A reference to this object after the calculation.
  */
  Matrix<m, n, V>& operator*=(const V& factor)
  {
    for(int i = 0; i < n; ++i)
      c[i] *= factor;
    return *this;
  }

  /**
  * Multiplication of this matrix by a factor.
  * @param factor The factor this matrix is multiplied by
  * @return A new object that contains the result of the calculation.
  */
  Matrix<m, n, V> operator*(const V& factor) const
  {
    return Matrix<m, n, V>(*this) *= factor;
  }

  /**
  * Division of this matrix by a factor.
  * @param factor The factor this matrix is divided by
  * @return A reference to this object after the calculation.
  */
  Matrix<m, n, V>& operator/=(const V& factor)
  {
    for(int i = 0; i < n; ++i)
      c[i] /= factor;
    return *this;
  }

  /**
  * Division of this matrix by a factor.
  * @param factor The factor this matrix is divided by
  * @return A new object that contains the result of the calculation.
  */
  Matrix<m, n, V> operator/(const V& factor) const
  {
    return Matrix<m, n, V>(*this) /= factor;
  }

  /**
  * Adds another matrix.
  * @param other The other matrix that is added to this one
  * @return A reference to this object after the calculation.
  */
  Matrix<m, n, V>& operator+=(const Matrix<m, n, V>& other)
  {
    for(int i = 0; i < n; ++i)
      c[i] += other.c[i];
    return *this;
  }

  /**
  * Computes the sum of two matrices
  * @param other Another matrix
  * @return The sum
  */
  Matrix<m, n, V> operator+(const Matrix<m, n, V>& other) const
  {
    return Matrix<m, n, V>(*this) += other;
  }

  /**
  * Subtracts another matrix.
  * @param other The other matrix that is subtracted from this one
  * @return A reference to this object after the calculation.
  */
  Matrix<m, n, V>& operator-=(const Matrix<m, n, V>& other)
  {
    for(int i = 0; i < n; ++i)
      c[i] -= other.c[i];
    return *this;
  }

  /**
  * Computes the difference of two matrices
  * @param other Another matrix
  * @return The difference
  */
  Matrix<m, n, V> operator-(const Matrix<m, n, V>& other) const
  {
    return Matrix<m, n, V>(*this) -= other;
  }

  /**
  * Comparison of another matrix with this one.
  * @param other The other matrix that will be compared to this one
  * @return Whether the two matrices are equal.
  */
  bool operator==(const Matrix<m, n, V>& other) const
  {
    for(int i = 0; i < n; ++i)
      if(c[i] != other.c[i])
        return false;
    return true;
  }

  /**
  * Comparison of another matrix with this one.
  * @param other The other matrix that will be compared to this one
  * @return Whether the two matrixs are unequal.
  */
  bool operator!=(const Matrix<m, n, V>& other) const
  {
    for(int i = 0; i < n; ++i)
      if(c[i] != other.c[i])
        return true;
    return false;
  }

  /**
  * Transpose the matrix
  * @return A new object containing transposed matrix
  */
  Matrix<n, m, V> transpose() const
  {
    Matrix<n, m, V> result;
    for(int i = 0; i < n; ++i)
      for(int j = 0; j < m; ++j)
        result.c[j][i] = c[i][j];
    return result;
  }

  /**
  * Calculation of the determinant of this matrix.
  * @return The determinant.
  */
  V det() const;

  /**
  * Calculate the adjoint of this matrix.
  * @return the adjoint matrix.
  */
  Matrix<m, n, V> adjoint() const;

  /**
  * Calculate the inverse of this matrix.
  * @return The inverse matrix
  */
  Matrix<m, n, V> invert() const;

  /**
  * Solves the system A*x=b where A is the actual matrix
  * @param b Vector b
  * @param x Solution x
  * @return Whether solving was possible
  */
  bool solve(const Vector<n, V>& b, Vector<n, V>& x) const;
};

/**
* Streaming operator that reads a Matrix<m, n, V> from a stream.
* @param stream The stream from which is read.
* @param matrix The Matrix<m, n, V> object.
* @return The stream.
*/
template <int m, int n, class V> In& operator>>(In& stream, Matrix<m, n, V>& matrix)
{
  STREAM_REGISTER_BEGIN_EXT(matrix);
  STREAM_EXT(stream, matrix.c);
  STREAM_REGISTER_FINISH;
  return stream;
}

/**
* Streaming operator that writes a Matrix<m, n, V> to a stream.
* @param stream The stream to write on.
* @param matrix The Matrix<m, n, V> object.
* @return The stream.
*/
template <int m, int n, class V> Out& operator<<(Out& stream, const Matrix<m, n, V>& matrix)
{
  STREAM_REGISTER_BEGIN_EXT(matrix);
  STREAM_EXT(stream, matrix.c);
  STREAM_REGISTER_FINISH;
  return stream;
}

typedef Matrix<1, 1, float> Matrix1x1f;
typedef Matrix<2, 2, float> Matrix2x2f;
typedef Matrix<2, 1, float> Matrix2x1f;
typedef Matrix<1, 2, float> Matrix1x2f;
typedef Matrix<3, 3, float> Matrix3x3f;
typedef Matrix<3, 2, float> Matrix3x2f;
typedef Matrix<3, 1, float> Matrix3x1f;
typedef Matrix<2, 3, float> Matrix2x3f;
typedef Matrix<1, 3, float> Matrix1x3f;
typedef Matrix<4, 4, float> Matrix4x4f;
typedef Matrix<4, 3, float> Matrix4x3f;
typedef Matrix<4, 2, float> Matrix4x2f;
typedef Matrix<4, 1, float> Matrix4x1f;
typedef Matrix<3, 4, float> Matrix3x4f;
typedef Matrix<2, 4, float> Matrix2x4f;
typedef Matrix<1, 4, float> Matrix1x4f;
