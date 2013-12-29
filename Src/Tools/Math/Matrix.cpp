/**
* @file Matrix.cpp
* Implementation of some functions of the Matrix class for various types.
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
* @author Colin Graf
*/

#include "Platform/BHAssert.h"
#include "Matrix.h"

template <> Matrix<1, 1, float> Matrix<1, 1, float>::invert() const
{
  return Matrix<1, 1, float>(1.f / c[0][0]);
}

template <> float Matrix<2, 2, float>::det() const
{
  return c[0].x * c[1].y - c[1].x * c[0].y;
}

template <> Matrix<2, 2, float> Matrix<2, 2, float>::invert() const
{
  float factor(1.f / det());
  return Matrix<2, 2, float>(Vector<2, float>(factor * c[1].y, -factor * c[0].y), Vector<2, float>(-factor * c[1].x, factor * c[0].x));
}

template <> float Matrix<3, 3, float>::det() const
{
  return
    c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) +
    c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
    c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
}

template <class V> static V det2(V a, V b, V c, V d)
{
  return a * d - b * c;
}

template <> Matrix<3, 3, float> Matrix<3, 3, float>::adjoint() const
{
  return Matrix<3, 3, float>(
           Vector<3, float>(
             det2(c[1].y, c[2].y, c[1].z, c[2].z),
             det2(c[2].x, c[1].x, c[2].z, c[1].z),
             det2(c[1].x, c[2].x, c[1].y, c[2].y)
           ),
           Vector<3, float>(
             det2(c[2].y, c[0].y, c[2].z, c[0].z),
             det2(c[0].x, c[2].x, c[0].z, c[2].z),
             det2(c[2].x, c[0].x, c[2].y, c[0].y)
           ),
           Vector<3, float>(
             det2(c[0].y, c[1].y, c[0].z, c[1].z),
             det2(c[1].x, c[0].x, c[1].z, c[0].z),
             det2(c[0].x, c[1].x, c[0].y, c[1].y)
           )
         );
}

template <> Matrix<3, 3, float> Matrix<3, 3, float>::invert() const
{
  return adjoint().transpose() / det();
}


template <int m, int n, class V> V Matrix<m, n, V>::det() const
{
  ASSERT(false); // not implemented
  /*
    // create copy of actual matrix
    Matrix_nxn<T, N> m(*this);

    // initialize ranking vector
    Vector_n<int, N> ranking;
    size_t i;
    for (i = 0; i < N; ++i)
      ranking[i] = i;

    T z = T();
    bool bPositive = true;
    int c;
    int r;
    for (c = 0; c < (int)(N-1); ++c)
    {
      // find row containing highest value
      int maxRow = c;
      T maxValue = m[ranking[maxRow]][c];
      if (maxValue < z)
        maxValue = -maxValue;
      for (r = c+1; r < (int)N; ++r)
      {
        T value = m[ranking[r]][c];
        if (value < z)
          value = -value;
        if (value > maxValue)
        {
          maxRow = r;
          maxValue = value;
        }
      }

      // if maximum value zero --> determinant is zero
      if (MVTools::isNearZero(maxValue))
        return z;
        / *if (maxValue == z)
      return z;* /

      // swap rows in ranking
      if (c != maxRow)
      {
        int temp = ranking[c];
        ranking[c] = ranking[maxRow];
        ranking[maxRow] = temp;
        bPositive = !bPositive;
      }

      // process all following rows
      for (r = c+1; r < (int)N; ++r)
      {
        // calc factor for subtracting
        T factor = m[ranking[r]][c] / m[ranking[c]][c];
        if (MVTools::isNearInf(factor))
        {
          if (MVTools::isNearPosInf(factor))
            throw MVException(MVException::PosInfValue);
          else
            throw MVException(MVException::NegInfValue);
        }

        // change matrix
        m[ranking[r]][c] = T();
        for (int c2 = c+1; c2 < (int)N; ++c2)
        {
          m[ranking[r]][c2] -= factor*m[ranking[c]][c2];
          T sub;
          sub = factor*m[ranking[c]][c2];
          if (MVTools::isNearInf(sub))
          {
            if (MVTools::isNearPosInf(sub))
              throw MVException(MVException::PosInfValue);
            else
              throw MVException(MVException::NegInfValue);
          }
        }
      }
    }
    // if last entry of matrix zero --> determinant is zero
    if (MVTools::isNearZero(m[ranking[N-1]][N-1]))
      return z;
      / *if (m[ranking[N-1]][N-1] == z)
    return z;* /
    // matrix has triangle form
    // calculate determinant
    T res = m[ranking[0]][0];
    for (r = 1; r < (int)N; ++r)
    {
      res *= m[ranking[r]][r];
      if (MVTools::isNearInf(res))
      {
        if (MVTools::isNearPosInf(res))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }
    }
    if (!bPositive)
      res = -res;
    return res;

  */
  return V(0);
}

template <int m, int n, class V> Matrix<m, n, V> Matrix<m, n, V>::adjoint() const
{
  ASSERT(false); // not implemented
  return Matrix<m, n, V>();
}

template <int m, int n, class V> Matrix<m, n, V> Matrix<m, n, V>::invert() const
{
  Matrix<n, n, V> left(*this);
  Matrix<n, n, V> right(V(1)); // I
  Vector<n, int> ranking;

  for(int i = 0; i < n; ++i)
    ranking[i] = i;

  int r, r2, maxrow;
  for(r = 0; r < n - 1; ++r)
  {
    // find highest value
    V maxval = left[ranking[r]][r];
    maxrow = r;
    if(maxval < V(0))
      maxval = -maxval;
    for(r2 = r + 1; r2 < n; ++r2)
    {
      float val = left[ranking[r2]][r];
      if(val < V(0))
        val = -val;
      if(val > maxval)
      {
        maxval = val;
        maxrow = r2;
      }
    }
    // swap rows
    int temp = ranking[r];
    ranking[r] = ranking[maxrow];
    ranking[maxrow] = temp;
    /*
    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }
    */

    for(r2 = r + 1; r2 < n; ++r2)
    {
      // calc factor for subtracting
      ASSERT(left[ranking[r]][r] != 0.f);
      V factor = left[ranking[r2]][r] / left[ranking[r]][r];
      /*
      if (MVTools::isNearInf(factor))
      {
        if (MVTools::isNearPosInf(factor))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }
      */

      // change left matrix
      left[ranking[r2]] -= left[ranking[r]] * factor;

      // change right matrix
      right[ranking[r2]] -= right[ranking[r]] * factor;
    }
  }
  // matrix has triangle form
  // bring to diagonal form
  for(r = n - 1; r > 0; --r)
  {
    /*
    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }
    */
    for(r2 = r - 1; r2 >= 0; --r2)
    {
      ASSERT(left[ranking[r]][r] != 0.f);
      V factor = left[ranking[r2]][r] / left[ranking[r]][r];
      /*
      if (MVTools::isNearInf(factor))
      {
        if (MVTools::isNearPosInf(factor))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }
      */

      // change left matrix
      left[ranking[r2]] -= left[ranking[r]] * factor;

      // change right matrix
      right[ranking[r2]] -= right[ranking[r]] * factor;
    }
  }
  // matrix has diagonal form
  // set entries of left matrix to 1 and apply multiplication to right
  Matrix<n, n, V> res;
  for(r = 0; r < n; ++r)
  {
    res[r] = right[ranking[r]];
    ASSERT(left[ranking[r]][r] != 0.f);
    /*
    if (MVTools::isNearZero(left[ranking[r]][r]))
    {
      if (MVTools::isNearNegZero(left[ranking[r]][r]))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }
    */
    res[r] /= left[ranking[r]][r];
  }
  return res;
}

template <int m, int n, class V> bool Matrix<m, n, V>::solve(const Vector<n, V>& b2, Vector<n, V>& x) const
{
  // create copy of actual matrix
  Vector<n, V> b = b2;
  Matrix<n, n, V> matrix = *this;

  // initialize ranking vector
  Vector<n, int> ranking;
  for(int i = 0; i < n; ++i)
    ranking[i] = i;

  for(int c = 0; c < n - 1; ++c)
  {
    // find row containing highest value
    int maxRow = c;
    V maxValue = matrix[c][ranking[maxRow]];
    if(maxValue < V())
      maxValue = -maxValue;
    for(int r = c + 1; r < n; ++r)
    {
      V value = matrix[c][ranking[r]];
      if(value < V())
        value = -value;
      if(value > maxValue)
      {
        maxRow = r;
        maxValue = value;
      }
    }

    // if maximum value zero --> matrix is singular
    if(maxValue == V())
      return false;

    // swap rows in ranking
    int temp = ranking[c];
    ranking[c] = ranking[maxRow];
    ranking[maxRow] = temp;

    // process all following rows
    for(int r = c + 1; r < n; ++r)
    {
      // calc factor for subtracting
      V factor = matrix[c][ranking[r]] / matrix[c][ranking[c]];
      V sub = factor * b[ranking[c]];

      // change vector b
      b[ranking[r]] -= sub;

      // change matrix
      matrix[c][ranking[r]] = V();
      for(int c2 = c + 1; c2 < n; ++c2)
      {
        sub = factor * matrix[c2][ranking[c]];
        matrix[c2][ranking[r]] -= sub;
      }
    }
  }

  // if last entry of matrix zero --> matrix is singular
  if(matrix[n - 1][ranking[n - 1]] == V())
    return false;

  // matrix has triangle form
  // calculate solutions
  b[ranking[n - 1]] /= matrix[n - 1][ranking[n - 1]];
  for(int r = n - 2; r >= 0; --r)
  {
    V sum = V();
    for(int c = r + 1; c < n; ++c)
      sum += matrix[c][ranking[r]] * b[ranking[c]];
    b[ranking[r]] = (b[ranking[r]] - sum) / matrix[r][ranking[r]];
  }

  // create vector with correct order
  for(int r = 0; r < n; ++r)
    x[r] = b[ranking[r]];
  return true;
}

// explicit template instantiation
template class Matrix<1, 1, float>;
template class Matrix<2, 2, float>;
template class Matrix<3, 3, float>;
template class Matrix<4, 4, float>;

