/**
 * @file FrontContour.h
 * Declaration of the representation of the front contour of the Nao foot.
 * @author Felix Wenk
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"

class FrontContour : public Streamable
{
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(points);
    if(in)
      approximate();
    STREAM_REGISTER_FINISH;
  }
public:
  std::vector<Vector2<> > points; /**< Points on the foot contour. */
  /** Bezier curve control points. */
  Vector2<> p0;
  Vector2<> p1;
  Vector2<> p2;
  Vector2<> p3;

  /**
   * Approximates the control points of the cubic bezier curve representing
   * the front contour using the measured points on the contour (points vector).
   */
  void approximate();

  /**
   * Calculates the contant point on the foot contour with the ball
   * depending on the kick direction vector.
   *
   * @param r The kick direction vector.
   * @param mirror If true, the control points will be mirrored with the x-axis
   *               before the contact point is calculated (to calculate the point
   *               for an axially symmetric foot).
   * @return The contact point in kick foot coordinates.
   */
  Vector2<> calcContactPoint(const Vector2<>& r, const bool mirror, Vector2<>& tangent) const;

  /**
   * Draws the contour of the right foot into the 3d debug drawing with id
   * module:IndykickEngine:rightfootcontour
   */
  void draw3D();
private:
  /**
   * Calculates the value for 't' corresponding to the y coordinate on the bezier curve
   * with the currently assumed control points.
   */
  float calcTforCoordinate(float y) const;
};
