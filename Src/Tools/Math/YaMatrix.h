/**
 * @file Matrix.h
 *
 * Yet another Matrix implementation.
 * A general dynamic Matrix implementation.
 *
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once
#include "MVTools.h"
#include <cstdarg>
#include <cmath>
#include <vector>

template<class V>
class YaVector : public std::vector<V>
{
  int N;
public:
  YaVector(size_t entries, const V& defaultValue)
    : std::vector<V>(entries, defaultValue), N(entries)
  {}

  YaVector<V>& operator=(const YaVector<V>& other)
  {
    (std::vector<V>&) *this = (std::vector<V>&) other;
    N = other.N;
    return *this;
  }

  const YaVector<V> operator+(const YaVector<V>& other)
  {
    YaVector<V> result(N, V());
    for(int i = 0; i < N; i++)
      result[i] = (*this)[i] + other[i];
    return result;
  }

  const YaVector<V> operator-(const YaVector<V>& other)
  {
    YaVector<V> result(N, V());
    for(int i = 0; i < N; i++)
      result[i] = (*this)[i] - other[i];
    return result;
  }

  V dotProduct(const YaVector<V>& other)
  {
    V result = V();
    for(int i = 0; i < N; i++)
      result += (*this)[i] * other[i];
    return result;
  }

  const YaVector<V> operator/(const V& scalar) const
  {
    YaVector<V> result(N, V());
    for(int i = 0; i < N; i++)
      result[i] = (*this)[i] / scalar;
    return result;
  }

  const YaVector<V> operator*(const V& scalar) const
  {
    YaVector<V> result(N, V());
    for(int i = 0; i < N; i++)
      result[i] = (*this)[i] * scalar;
    return result;
  }

  operator V() const
  {
    return (*this)[0];
  }
};

/**
 * @class YaMatrix
 * A general dynamic Matrix template class. A vector of row vectors.
 *
 * \tparam V The type of scalar values. The default is float.
 */
template<class V = float>
class YaMatrix
{
public:
  int M, N;
  /**
   * A two dimensional array containing the values of the YaMatrix.
   * Respectively a one dimensional array of the YaMatrix' rows.
   */
  std::vector<YaVector<V> > v;

  /**
   * Allocates and initializes the memory for a dynamic matrix.
   * \param M Number of rows.
   * \param N Number of columns.
   * \param diagonal Value of the main diagonal. All other values are
   *                 initialized as 0.
   */
  YaMatrix(int M, int N, V diagonal = V()) : M(M), N(N), v(M, YaVector<V>(N, V()))
  {
    if(diagonal != V())
      for(int i = 0; i < M && i < N; i++)
        v[i][i] = diagonal;
  }

  /**
   * \return The transposed YaMatrix.
   */
  YaMatrix<V> transpose() const
  {
    YaMatrix<V> t(N, M);
    for(int m = 0; m < M; m++)
    {
      const YaVector<V>& rowM = v[m];
      for(int n = 0; n < N; n++)
        t.v[n][m] = rowM[n];
    }
    return t;
  }

  /**
   * Complexity O(min(M,N))
   * \return The trace of the YaMatrix.
   */
  V trace() const
  {
    const int minimum = std::min(M, N);
    V trace = V(0);
    for(int i = 0; i < minimum; i++)
    {
      trace += v[i][i];
    }
    return trace;
  }

  /**
   * Inverts a diagonal matrix, i. e. sets diagonal elements to their reciprocal.
   */
  YaMatrix<V> diagonalInverse()
  {
    YaMatrix<V> inv(M, N);
    for(int i = 0; i < M; i++)
      inv[i][i] = V(1) / v[i][i];
    return inv;
  }

  /**
   * @author <a href="mailto:stefanuhrig@gmx.net">Stefan Uhrig</a>
   * @return The inverted matrix.
   */
  YaMatrix<V> inverse() const
  {
    YaMatrix<V> left(*this);
    YaMatrix<V> right(N, N, 1);
    YaMatrix<int> ranking(N, 1);
    int i;
    for(i = 0; i < N; ++i)
    {
      ranking[i][0] = i;
    }
    const V zero = V();
    int r, r2, maxrow;
    for(r = 0; r < (N - 1); ++r)
    {
      // find highest value
      V maxval = left[ranking[r][0]][r];
      maxrow = r;
      if(maxval < zero)
        maxval = -maxval;
      for(r2 = r + 1; r2 < N; ++r2)
      {
        V val = left[ranking[r2][0]][r];
        if(val < zero)
          val = -val;
        if(val > maxval)
        {
          maxval = val;
          maxrow = r2;
        }
      }
      // swap rows
      int temp = ranking[r][0];
      ranking[r][0] = ranking[maxrow][0];
      ranking[maxrow][0] = temp;
      if(MVTools::isNearZero(left[ranking[r][0]][r]))
      {
        if(MVTools::isNearNegZero(left[ranking[r][0]][r]))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      for(r2 = r + 1; r2 < (int)N; ++r2)
      {
        // calc factor for subtracting
        V factor = left[ranking[r2][0]][r] / left[ranking[r][0]][r];
        if(MVTools::isNearInf(factor))
        {
          if(MVTools::isNearPosInf(factor))
            throw MVException(MVException::PosInfValue);
          else
            throw MVException(MVException::NegInfValue);
        }
        // change left matrix
        YaMatrix<V> newRowVector = left.rowVector(ranking[r2][0]) - factor * left.rowVector(ranking[r][0]);
        left.setRowVector(ranking[r2][0], newRowVector);
        // change right matrix
        newRowVector = right.rowVector(ranking[r2][0]) - factor * right.rowVector(ranking[r][0]);
        right.setRowVector(ranking[r2][0], newRowVector);
      }
    }
    // matrix has triangle form
    // bring to diagonal form
    for(r = (int)(N - 1); r > 0; --r)
    {
      if(MVTools::isNearZero(left[ranking[r][0]][r]))
      {
        if(MVTools::isNearNegZero(left[ranking[r][0]][r]))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      for(r2 = r - 1; r2 >= 0; --r2)
      {
        V factor = left[ranking[r2][0]][r] / left[ranking[r][0]][r];
        if(MVTools::isNearInf(factor))
        {
          if(MVTools::isNearPosInf(factor))
            throw MVException(MVException::PosInfValue);
          else
            throw MVException(MVException::NegInfValue);
        }

        // change left matrix
        YaMatrix<V> newRowVector = left.rowVector(ranking[r2][0]) - factor * left.rowVector(ranking[r][0]);
        left.setRowVector(ranking[r2][0], newRowVector);

        // change right matrix
        newRowVector = right.rowVector(ranking[r2][0]) - factor * right.rowVector(ranking[r][0]);
        right.setRowVector(ranking[r2][0], newRowVector);
      }
    }
    // matrix has diagonal form
    // set entries of left matrix to 1 and apply multiplication to right
    YaMatrix<V> res(N, N);
    for(r = 0; r < N; ++r)
    {
      res[r] = right[ranking[r][0]];

      if(MVTools::isNearZero(left[ranking[r][0]][r]))
      {
        if(MVTools::isNearNegZero(left[ranking[r][0]][r]))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      YaMatrix newRowVector = res.rowVector(r) / left[ranking[r][0]][r];
      res.setRowVector(r, newRowVector);
    }
    return res;
  }

  /**
   * Calculates the matrix L, such that A = LL^T, where L is a lower triangular matrix.
   * Assumes the matrix to be symmetric and positive semi-definite.
   * Here, I implemented the Cholesky-Banachlewicz algorithm.
   * @see http://en.wikipedia.org/wiki/Cholesky_decomposition
   */
  YaMatrix<V> choleskyDecomposition() const
  {
    YaMatrix<V> L(N, N);
    for(int j = 0; j < N; j++)
    {
      // diagonal
      V sub = V();
      for(int k = 0; k < j; k++)
        sub += L[j][k] * L[j][k];
      L[j][j] = (V) sqrt(double(v[j][j] - sub));

      // below diagonal
      for(int i = j + 1; i < N; i++)
      {
        V sub = V();
        for(int k = 0; k < j; k++)
          sub += L[i][k] * L[j][k];
        L[i][j] = (v[i][j] - sub) / L[j][j];
      }
    }
    return L;
  }

  V squaredVectorLength()
  {
    V length(0);
    for(int m = 0; m < M; m++)
      length += v[m][0] * v[m][0];
    return length;
  }

  YaMatrix<V> columnVector(int n) const
  {
    YaMatrix<V> vector(M, 1);
    for(int m = 0; m < M; m++)
      vector[m][0] = v[m][n];
    return vector;
  }

  YaMatrix<V> rowVector(int m) const
  {
    YaMatrix<V> vector(1, N);
    const YaVector<V>& rowM = v[m];
    for(int n = 0; n < N; n++)
      vector[0][n] = rowM[n];
    return vector;
  }

  YaMatrix<V>& setRowVector(int m, const YaMatrix<V>& vector)
  {
    YaVector<V>& rowM = v[m];
    for(int n = 0; n < N; n++)
      rowM[n] = vector[0][n];
    return *this;
  }

  YaMatrix<V>& operator-()
  {
    return *this *= V(-1);
  }

  YaMatrix<V>& operator=(const YaMatrix<V>& o)
  {
    for(int i = 0; i < M; i++) for(int j = 0; j < N; j++)
        v[i][j] = o.v[i][j];
    return *this;
  }

  std::vector<V>& operator[](size_t row)
  {
    return v[row];
  }

  const std::vector<V>& operator[](size_t row) const
  {
    return v[row];
  }
};

template<class V>
bool operator==(const YaMatrix<V>& a, const YaMatrix<V>& b)
{
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      if(a.v[m][n] != b.v[m][n]) return false;
  return true;
}

template<class V>
bool operator!=(const YaMatrix<V>& a, const YaMatrix<V>& b)
{
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      if(a.v[m][n] != b.v[m][n]) return true;
  return false;
}

/**
 * Multiplies two matrices.
 * \tparam V scalar type
 * \param a the first factor
 * \param b the second factor
 * \return the resulting MxO product
 */
template<class V>
YaMatrix<V> operator*(const YaMatrix<V>& a, const YaMatrix<V>& b)
{
  YaMatrix<V> c(a.M, b.N);
  for(int m = 0; m < a.M; m++) for(int o = 0; o < b.N; o++)
      for(int n = 0; n < a.N; n++)
        c.v[m][o] += a.v[m][n] * b.v[n][o];
  return c;
}

/**
 * Adds two matrices of the same dimensions.
 * \tparam V scalar type
 * \param a the first summand
 * \param b the second summand
 * \return sum
 */
template<class V>
YaMatrix<V> operator+(const YaMatrix<V>& a, const YaMatrix<V>& b)
{
  YaMatrix<V> c(a.M, a.N);
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      c.v[m][n] = a.v[m][n] + b.v[m][n];
  return c;
}

template<class V>
YaMatrix<V> operator+=(YaMatrix<V>& a, const YaMatrix<V>& b)
{
  return a = a + b;
}

/**
 * Subtracts two matrices of the same dimensions.
 * \tparam V scalar type
 * \param a minuend
 * \param b subtrahend
 * \return difference
 */
template<class V>
YaMatrix<V> operator-(const YaMatrix<V>& a, const YaMatrix<V>& b)
{
  YaMatrix<V> c(a.M, a.N);
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      c.v[m][n] = a.v[m][n] - b.v[m][n];
  return c;
}

template<class V>
YaMatrix<V>& operator-=(YaMatrix<V>& a, const YaMatrix<V>& b)
{
  return a = a - b;
}

/**
 * Multiplies a YaMatrix with a scalar factor.
 * \tparam V scalar type
 * \param f The scalar factor.
 * \param a The YaMatrix.
 * \return product
 */
template<class V>
YaMatrix<V> operator*(const V f, const YaMatrix<V>& a)
{
  YaMatrix<V> b(a.M, a.N);
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      b.v[m][n] = f * a.v[m][n];
  return b;
}

/**
 * Multiplies a YaMatrix with a scalar factor.
 * \tparam V scalar type
 * \param f The scalar factor.
 * \param a The YaMatrix.
 * \return product
 */
template<class V>
YaMatrix<V> operator*(const YaMatrix<V>& a, const V f)
{
  return f * a;
}

template<class V>
YaMatrix<V>& operator*=(YaMatrix<V>& a, const V f)
{
  return a = f * a;
}

/**
 * Multiplies a YaMatrix with the reciprocal of a scalar.
 * \tparam V scalar type
 * \param d scalar
 * \param a YaMatrix
 * \return division
 */
template<class V>
YaMatrix<V> operator/(const YaMatrix<V>& a, const V d)
{
  YaMatrix<V> b(a.M, a.N);
  for(int m = 0; m < a.M; m++) for(int n = 0; n < a.N; n++)
      b.v[m][n] = a.v[m][n] / d;
  return b;
}

template<class V>
YaMatrix<V>& operator/=(YaMatrix<V>& a, const V f)
{
  return a = a / f;
}

template<class T>
bool isNaN(const YaMatrix<T>& value)
{
  for(int m = 0; m < value.M; m++)
    for(int n = 0; n < value.N; n++)
      if(isNaN(value[m][n]))
        return true;

  return false;
}

template<class T>
bool isInf(const YaMatrix<T>& value)
{
  for(int m = 0; m < value.M; m++)
    for(int n = 0; n < value.N; n++)
      if(isInf(value[m][n]))
        return true;

  return false;
}

template<class T>
bool equals(const YaMatrix<T>& a, const YaMatrix<T>& b, T delta)
{
  if(a.M != b.M || a.N != b.N)
    return false;

  for(int m = 0; m < a.M; m++)
    for(int n = 0; n < a.N; n++)
      if(T(fabs(a[m][n] - b[m][n])) >= delta)
        return false;

  return true;
}

template<class T>
YaMatrix<T> yaMatrix(int M, int N, T first, ...)
{
  YaMatrix<T> matrix(M, N);
  matrix[0][0] = first;
  va_list vl;
  va_start(vl, first);
  for(int i = 0; i < M; i++)
    for(int j = 0; j < N; j++)
      if(i != 0 || j != 0)
        matrix[i][j] = (T) va_arg(vl, T);
  va_end(vl);
  return matrix;
}

template<class T>
YaMatrix<T> yaVector(int N, T first, ...)
{
  YaMatrix<T> vector(N, 1);
  vector[0][0] = first;
  va_list vl;
  va_start(vl, first);
  for(int i = 1; i < N; i++)
    vector[i][0] = (T) va_arg(vl, T);
  va_end(vl);
  return vector;
}
