//Do NOT directly include this file, instead include "Tools/Math/Eigen.h"

/**
 * Calculates the gradient function for each row, e.g. the derivation.
 * The gradient is computed using central differences in the interior
 * and first differences at the boundaries.
 * The returned gradient hence has the same shape as the input array.
 *
 * @param dt distance between two points.
 */
Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> gradient(const _Scalar dt) const
{
  Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> ret(Base::rows(), Base::cols());
  ASSERT(Base::cols() > 0);
  ASSERT(Base::rows() > 0);
  //forward difference quotient (special case for first element)
  ret.col(0) = (Base::col(1) - Base::col(0)) / dt;

  const int end = static_cast<int>(Base::cols()) - 1;
  const _Scalar dt2 = 2 * dt;
  for(int i = 1; i < end; ++i)
  {
    //central difference quotient for each inner element
    ret.col(i) = (Base::col(i + 1) - Base::col(i - 1)) / dt2;
  }
  //backward difference quotient (special case for last element)
  ret.col(end) = (Base::col(end) - Base::col(end - 1)) / dt;
  return ret;
}
