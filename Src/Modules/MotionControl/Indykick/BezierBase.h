/**
 * @file BezierBase.h
 * Declarations of the base classes to represent and generate cubic bezier curves.
 * @author Felix Wenk
 */

#pragma once

#include "BKickBase.h"

class RobotModel;
class TorsoMatrix;
class BKickRequest;
class RobotDimensions;

template <class Point>
class BezierCurveReferenceTemplate {
public:
  BezierCurveReferenceTemplate(const Point& p0, const Point& p1, const Point& p2, const Point& p3);
  void evaluate(const float t, Point& point) const;
  void evaluateFirstDerivative(const float progress, Point& point, const float duration = 1.0f) const;

  const Point& p0;
  const Point& p1;
  const Point& p2;
  const Point& p3;
};

/**
 * Class representing a cubic bezier curve.
 */
template <class Point>
class BezierCurveTemplate
{
public:
  BezierCurveTemplate(const Point& p0, const Point& p1, const Point& p2, const Point& p3);

  /**
   * Evaluates this bezier curve at the point in time t.
   * Note that 0 <= t <= 1.
   */
  void evaluate(const float t, Point& point) const;
  void evaluateFirstDerivative(const float t, Point& point, const float duration = 1.0f) const;

  Point p0, p1, p2, p3; /**< The control points of the curve. */
};

typedef BezierCurveReferenceTemplate<Vector3<> > BezierCurveReference;
typedef BezierCurveTemplate<Vector3<> > BezierCurve;

template <typename BezierTemplateBase>
class AngleBezierCurveTemplate : public BezierTemplateBase
{
public:
  AngleBezierCurveTemplate(const Vector3<>& p0, const Vector3<>& p1, const Vector3<>& p2, const Vector3<>& p3);

  /**
   * Evaluates this bezier curve at the point in time t.
   * Note that 0 <= t <= 1.
   */
  void evaluate(const float t, RotationMatrix& rotation) const;
};

typedef AngleBezierCurveTemplate<BezierCurveReference> AngleBezierCurveReference;
typedef AngleBezierCurveTemplate<BezierCurve> AngleBezierCurve;
