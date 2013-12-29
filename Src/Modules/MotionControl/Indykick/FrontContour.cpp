/**
 * @file FrontContour.h
 * Implementation of the FrontContour methods.
 * @author Felix Wenk
 */

#include "Tools/Debugging/Asserts.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/Vector.h"
#include "FrontContour.h"

void FrontContour::approximate()
{
  // Bezier curve passes through end points of curve.
  p0 = points.front();  // First control point
  p3 = points.back();   // Last control point
  // Contour is vertical at the end points, and other control points are on the tangents of the contour.
  p1 = Vector2<>(-1.0f, p0.y); // -1 denotes an invalid x coordinate.
  p2 = Vector2<>(-1.0f, p3.y);

  // Now we can calculate the t's using the y component of the curve.
  const unsigned numOfCPoints = 9 + 2; // Actually this depends on the contour file, but the Microsoft compiler complains if I use points.size() directly.
  ASSERT(numOfCPoints == points.size());
  float t[numOfCPoints];
  for(unsigned i = 0; i < numOfCPoints; ++i)
    t[i] = calcTforCoordinate(points[i].y);

  // Now use least squares to approxmiate x coordinate of p1 and p2
  // First we assemble the vector of the expected results from the contour points.
  // We don't use the first and the last point on the contour since the parameters we're
  // optimizing (i.e. the inner control points) have no effect on the value function for those points.
  Vector<9, float> target;
  for(size_t i = 1; i < points.size() - 1; ++i)
  {
    const float p0c = (1 - t[i]) * (1 - t[i]) * (1 - t[i]);
    const float p3c = t[i] * t[i] * t[i];
    target[i - 1] = points[i].x - p0c * p0.x - p3c * p3.x;
  }

  // Next is the design matrix, whose rows are the coefficients of the parameters we want to optimize
  // in the bezier curve function. Again, each row correspsonds to one training example.
  Matrix<9, 2, float> X;
  for(int i = 0; i < 9; ++i)
  {
    const float time = t[i + 1];
    X[0][i] = 3 * (1 - time) * (1 - time) * time;
    X[1][i] = 3 * (1 - time) * time * time;
  }

  // ...and now we solve for the parameter vector, which contains the x coordinates of p1 and p2.
  const Matrix<2, 9, float> Xtrans = X.transpose();
  const Matrix<2, 2, float> XtransX = Xtrans * X;
  const Vector<2, float> param = XtransX.invert() * Xtrans * target;

  p1.x = param.x;
  p2.x = param.y;
}

float FrontContour::calcTforCoordinate(float y) const
{
  const float p0y = p0.y;
  const float p3y = p3.y;
  const Vector<3, float> coefficients(p0y - y,
                                      3.0f * p3y - 3.0f * p0y,
                                      2.0f * p0y - 2.0f * p3y);
  // For all points on a bezier curve: 0 <= t <= 1
  float a = 0.0f;
  float b = 1.0f;
  float c;
  while(true)
  {
    const Vector<3, float> aVec(1.0f, a * a, a * a * a);
    const Vector<3, float> bVec(1.0f, b * b, b * b * b);

    const float ya = coefficients * aVec;
    const float yb = coefficients * bVec;
    c = a - (b - a) / (yb - ya) * ya;

    const Vector<3, float> cVec(1.0f, c * c, c * c * c);
    const float yc = coefficients * cVec;

    if(fabs(yc) < 1e-3f)
      break; // converged

    if((ya < 0.0f) == (yc < 0.0f))
      a = c;
    else if((yb < 0.0f) == (yc < 0.0f))
      b = c;
    else
      ASSERT(false);
  }

  return c;
}

Vector2<> FrontContour::calcContactPoint(const Vector2<>& r, const bool mirror, Vector2<>& tangent) const
{
  // Use the correct contour for the foot. Since the control points are for the right
  // foot, we mirror the control points with the x-axis, because the feet are x-axially symmetric.
  const float ySign = mirror ? -1.0f : 1.0f;
  const Vector2<> p0(this->p0.x, ySign * this->p0.y);
  const Vector2<> p1(this->p1.x, ySign * this->p1.y);
  const Vector2<> p2(this->p2.x, ySign * this->p2.y);
  const Vector2<> p3(this->p3.x, ySign * this->p3.y);

  // At the contact point, the tangent on the foot contour (the bezier curve) is
  // perpendicular to the kick direction. So we calculate at what 't' we have such a tangent
  // and then evaluate the curve with that t to get the contact point.
  // Since we have a cubic bezier curve, the tangent is a quadratic equation with respect ot t,
  // so we can solve for t by completing squares.

  // Calculate t
  const float a = r.x * (3.0f * p2.x - 3.0f * p1.x - p3.x + p0.x) + r.y * (2.0f * p3.y - 2.0f * p0.y);
  const float b = r.x * (4.0f * p1.x - 2.0f * p0.x - 2.0f * p2.x) + r.y * (2.0f * p0.y - 2.0f * p3.y);
  const float c = r.x * (p0.x - p1.x);

  const float radicand = b * b - 4.0f * a * c;
  ASSERT(radicand >= 0.0f);
  const float root = std::sqrt(radicand);
  const float numerator1 = -b + root;
  const float numerator2 = -b - root;
  const float t1 = numerator1 / (2.0f * a);
  const float t2 = numerator2 / (2.0f * a);

  const float t = (0.0f <= t1 && t1 <= 1.0f) ? t1 : t2;

  // Evaluate bezier curve at t.
  const float oneMinusT = 1 - t;
  const float oneMinusTSqr = oneMinusT * oneMinusT;
  const float tSqr = t * t;

  const Vector2<> btt0 = p0 * oneMinusTSqr + p1 * 2 * oneMinusT * t + p2 * tSqr;
  const Vector2<> btt1 = p1 * oneMinusTSqr + p2 * 2 * oneMinusT * t + p3 * tSqr;
  tangent = btt1 - btt0;

  const Vector2<> point = btt0 * oneMinusT + btt1 * t;
  return point;
}

void FrontContour::draw3D()
{
  const float segments = 100.0f;
  Vector2<> point0(p0.x, p0.y);
  for(int i = 1; i < 100; ++i)
  {
    const float t = static_cast<float>(i) / segments;
    const float oneMinusT = 1 - t;
    const float oneMinusTSqr = oneMinusT * oneMinusT;
    const float tSqr = t * t;

    const Vector2<> point1 = p0 * (oneMinusT * oneMinusTSqr) + p1 * (3.0f * oneMinusTSqr * t)
      + p2 * (3.0f * oneMinusT * tSqr) + p3 * (tSqr * t);

    LINE3D("module:IndykickEngine:rightfootcontour", point0.x, point0.y, -40.0f, point1.x, point1.y, -40.0f, 5, ColorClasses::red);
    point0 = point1;
  }
  LINE3D("module:IndykickEngine:rightfootcontour", point0.x, point0.y, -40.0f, p3.x, p3.y, -40.0f, 5, ColorClasses::red);
}
