#include "FifthOrderPolynomial.h"

FifthOrderPolynomial::FifthOrderPolynomial(const float x0, const float fx0,
                                           const float fx0d, const float fx0dd,
                                           const float x1, const float fx1,
                                           const float fx1d, const float fx1dd)
{

  const float x02 = x0 * x0;
  const float x03 = x02 * x0;
  const float x04 = x03 * x0;
  const float x05 = x04 * x0;
  const float x12 = x1 * x1;
  const float x13 = x12 * x1;
  const float x14 = x13 * x1;
  const float x15 = x14 * x1;
  Matrix6f A;
  A << 1, x0, x02   , x03    , x04     , x05     ,
       0, 1 , 2 * x0, 3 * x02, 4 * x03 , 5 * x04 ,
       0, 0 , 2     , 6 * x0 , 12 * x02, 20 * x03,
       1, x1, x12   , x13    , x14     , x15     ,
       0, 1 , 2 * x1, 3 * x12, 4 * x13 , 5 * x14 ,
       0, 0 , 2     , 6 * x1 , 12 * x12, 20 * x13;

  Vector6f b;
  b << fx0, fx0d, fx0dd, fx1, fx1d, fx1dd;
  c = A.colPivHouseholderQr().solve(b);
}

Vector3f FifthOrderPolynomial::evaluate(const float x) const
{
  const float x2 = x * x;
  const float x3 = x2 * x;
  const float x4 = x3 * x;
  const float x5 = x4 * x;
  return Vector3f(c(0) + c(1) * x + c(2) * x2 + c(3) * x3 + c(4) * x4 + c(5) * x5,
                  c(1) + 2 * c(2) * x + 3 * c(3) * x2 + 4 * c(4) * x3 + 5 * c(5) * x4,
                  2 * c(2) + 6 * c(3) * x + 12 * c(4) * x2 + 20 * c(5) * x3);
}
