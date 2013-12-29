/**
 * @file BezierBase.cpp
 * Implementation of the methods of the bezier curve and bezier curve generator base classes.
 * @author Felix Wenk
 */

#include "Tools/Debugging/Asserts.h"
#include "BezierBase.h"
#include "Tools/Math/Vector.h"

template<class Point>
BezierCurveReferenceTemplate<Point>::BezierCurveReferenceTemplate(const Point& p0, const Point& p1, const Point& p2, const Point& p3)
: p0(p0), p1(p1), p2(p2), p3(p3)
{}

template<class Point>
void BezierCurveReferenceTemplate<Point>::evaluate(const float t, Point& point) const
{
  ASSERT(t <= 1.0f);
  ASSERT(t >= 0.0f);

  const float oneMinusT = 1 - t;
  const float oneMinusTSqr = oneMinusT * oneMinusT;
  const float tSqr = t * t;

  point = p0 * (oneMinusT * oneMinusTSqr) + p1 * (3.0f * oneMinusTSqr * t)
  + p2 * (3.0f * oneMinusT * tSqr) + p3 * (tSqr * t);
}

template<class Point>
void BezierCurveReferenceTemplate<Point>::evaluateFirstDerivative(const float progress, Point& point, const float duration) const
{
  ASSERT(progress <= 1.0f);
  ASSERT(progress >= 0.0f);

  const float t = duration * progress;

  const float tSqr = sqr(t);
  const float bmt = duration - t;
  const float bmtSqr = sqr(bmt);

  point = p3 * tSqr;
  point += p2 * (2.0f * bmt * t - tSqr);
  point += p1 * (bmtSqr - 2.0f * bmt * t);
  point -= p0 * bmtSqr;
  point *= 3.0f / (duration * sqr(duration));
}

template<class Point>
BezierCurveTemplate<Point>::BezierCurveTemplate(const Point& p0, const Point& p1, const Point& p2, const Point& p3)
: p0(p0), p1(p1), p2(p2), p3(p3)
{}

/*
 * The following function looks exaclty the same as the above one,
 * but this one does use actual objects and not const reference to them.
 * This kind of sucks, but I couldn't come up with a class hierarchy
 * that fixes this. Having template arguments which a references
 * leads to 'references to references' in the argument for this function.
 */
template<class Point>
void BezierCurveTemplate<Point>::evaluate(const float t, Point &point) const
{
  ASSERT(t <= 1.0f);
  ASSERT(t >= 0.0f);

  const float oneMinusT = 1 - t;
  const float oneMinusTSqr = oneMinusT * oneMinusT;
  const float tSqr = t * t;

  point = p0 * (oneMinusT * oneMinusTSqr) + p1 * (3.0f * oneMinusTSqr * t)
  + p2 * (3.0f * oneMinusT * tSqr) + p3 * (tSqr * t);
}

template<class Point>
void BezierCurveTemplate<Point>::evaluateFirstDerivative(const float progress, Point& point, const float duration) const
{
  ASSERT(progress <= 1.0f);
  ASSERT(progress >= 0.0f);

  const float t = duration * progress;

  const float tSqr = sqr(t);
  const float bmt = duration - t;
  const float bmtSqr = sqr(bmt);

  point = p3 * tSqr;
  point += p2 * (2.0f * bmt * t - tSqr);
  point += p1 * (bmtSqr - 2.0f * bmt * t);
  point -= p0 * bmtSqr;
  point *= 3.0f / (duration * sqr(duration));
}

template <typename BezierTemplateBase>
AngleBezierCurveTemplate<BezierTemplateBase>::AngleBezierCurveTemplate(const Vector3<>& p0, const Vector3<>& p1, const Vector3<>& p2, const Vector3<>& p3)
: BezierTemplateBase(p0, p1, p2, p3)
{}

template <typename BezierTemplateBase>
void AngleBezierCurveTemplate<BezierTemplateBase>::evaluate(const float t, RotationMatrix &rotation) const
{
  Vector3<> angleAxis;
  BezierTemplateBase::evaluate(t, angleAxis);
  rotation = RotationMatrix(angleAxis);
}

template class BezierCurveReferenceTemplate<Vector3<> >;
template class AngleBezierCurveTemplate<BezierCurve>;
template class AngleBezierCurveTemplate<BezierCurveReference>;

template class BezierCurveTemplate<Vector3<> >;
template class BezierCurveTemplate<Vector2f>;
template class BezierCurveTemplate<Vector4f>;
template class BezierCurveTemplate<float>;
