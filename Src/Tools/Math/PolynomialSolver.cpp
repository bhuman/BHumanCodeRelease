/**
 * File:   QuarticPolynomial.cpp
 * Author: arne
 *
 * Created on May 1, 2014, 10:34 AM
 */

#include "PolynomialSolver.h"
#include <cmath>
#include "BHMath.h"

int PolynomialSolver::isZero(float x)
{
  const float EQN_EPS = 1e-7f;
  return x > -EQN_EPS && x < EQN_EPS;
}

int PolynomialSolver::solveLinear(float c[2], float s[1])
{
  if(isZero(c[1]))
    return 0;
  s[0] = -c[0] / c[1];
  return 1;
}

int PolynomialSolver::solveQuadric(float c[3], float s[2])
{
  float p, q, D;

  // make sure we have a d2 equation
  if(isZero(c[2]))
    return solveLinear(c, s);

  // normal for: x^2 + px + q
  p = c[1] / (2.0f * c[2]);
  q = c[0] / c[2];
  D = p * p - q;

  if(isZero(D))
  {
    // one float root
    s[0] = s[1] = -p;
    return 1;
  }

  if(D < 0.0)// no real root
    return 0;
  else
  {
    // two real roots
    float sqrt_D = std::sqrt(D);
    s[0] = sqrt_D - p;
    s[1] = -sqrt_D - p;
    return 2;
  }
}

int PolynomialSolver::solveCubic(float c[4], float s[3])
{
  // normalize the equation:x ^ 3 + Ax ^ 2 + Bx  + C = 0
  const float A = c[2] / c[3];
  const float B = c[1] / c[3];
  const float C = c[0] / c[3];

  // substitute x = y - A / 3 to eliminate the quadric term: x^3 + px + q = 0
  const float sq_A = A * A;
  const float p = 1.0f / 3.0f * (-1.0f / 3.0f * sq_A + B);
  const float q = 1.0f / 2.0f * (2.0f / 27.0f * A * sq_A - 1.0f / 3.0f * A * B + C);

  // use Cardano's formula
  const float cb_p = p * p * p;
  const float D = q * q + cb_p;

  int num;
  if(isZero(D))
  {
    if(isZero(q))
    {
      // one triple solution
      s[0] = 0.0f;
      num = 1;
    }
    else
    {
      // one single and one float solution
      const float u = cbrtf(-q);
      s[0] = 2.0f * u;
      s[1] = -u;
      num = 2;
    }
  }
  else if(D < 0.0)
  {
    // casus irreductibilis: three real solutions
    const float phi = 1.0f / 3.0f * std::acos(-q / std::sqrt(-cb_p));
    const float t = 2.0f * std::sqrt(-p);
    s[0] = t * std::cos(phi);
    s[1] = -t * std::cos(phi + pi / 3.0f);
    s[2] = -t * std::cos(phi - pi / 3.0f);
    num = 3;
  }
  else
  {
    // one real solution
    const float sqrt_D = std::sqrt(D);
    const float u = cbrtf(sqrt_D + std::abs(q));
    if(q > 0.0f)
      s[0] = -u + p / u;
    else
      s[0] = u - p / u;
    num = 1;
  }

  // resubstitute
  const float sub = 1.0f / 3.0f * A;
  for(int i = 0; i < num; i++)
    s[i] -= sub;
  return num;
}

int PolynomialSolver::solveQuartic(float c[5], float s[4])
{
  float coeffs[4],
        z, u, v, sub,
        A, B, C, D,
        sq_A, p, q, r;

  // normalize the equation:x ^ 4 + Ax ^ 3 + Bx ^ 2 + Cx + D = 0
  A = c[3] / c[4];
  B = c[2] / c[4];
  C = c[1] / c[4];
  D = c[0] / c[4];

  // subsitute x = y - A / 4 to eliminate the cubic term: x^4 + px^2 + qx + r = 0
  sq_A = A * A;
  p = -3.0f / 8.0f * sq_A + B;
  q = 1.0f / 8.0f * sq_A * A - 1.0f / 2.0f * A * B + C;
  r = -3.0f / 256.0f * sq_A * sq_A + 1.0f / 16.0f * sq_A * B - 1.0f / 4.0f * A * C + D;

  int num;
  if(isZero(r))
  {
    // no absolute term:y(y ^ 3 + py + q) = 0
    coeffs[0] = q;
    coeffs[1] = p;
    coeffs[2] = 0.0;
    coeffs[3] = 1.0;

    num = solveCubic(coeffs, s);
    s[num++] = 0;
  }
  else
  {
    // solve the resolvent cubic...
    coeffs[0] = 1.0f / 2.0f * r * p - 1.0f / 8.0f * q * q;
    coeffs[1] = -r;
    coeffs[2] = -1.0f / 2.0f * p;
    coeffs[3] = 1.0f;
    (void)solveCubic(coeffs, s);

    // ...and take the one real solution...
    z = s[0];

    // ...to build two quadratic equations
    u = z * z - r;
    v = 2.0f * z - p;

    if(isZero(u))
      u = 0.0;
    else if(u > 0.0f)
      u = std::sqrt(u);
    else
      return 0;

    if(isZero(v))
      v = 0;
    else if(v > 0.0f)
      v = std::sqrt(v);
    else
      return 0;

    coeffs[0] = z - u;
    coeffs[1] = q < 0 ? -v : v;
    coeffs[2] = 1.0f;

    num = solveQuadric(coeffs, s);

    coeffs[0] = z + u;
    coeffs[1] = q < 0 ? v : -v;
    coeffs[2] = 1.0f;

    num += solveQuadric(coeffs, s + num);
  }

  // resubstitute
  sub = 1.0f / 4 * A;
  for(int i = 0; i < num; i++)
    s[i] -= sub;

  return num;
}
