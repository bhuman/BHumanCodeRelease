/**
 * @file ShellLimits.h
 * Declaration of a class representing joint angle limits due to the shell of the robot.
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Common.h"

using namespace std;

class ShellLimits : public Streamable
{
public:
  STREAMABLE(Point,
  {
  public:
    bool operator<(const Point& other) const,

    (float) x,
    (float) min,
    (float) max,
  });

  /**
   * Evaluates the piecewise-linear shell limits function.
   * For a given x, the min and max angles of the corresponding joint
   * are returned in the output parameters min and max.
   *
   * This method returns false if x out of bounds and thus min and max are
   * invalid. Otherwise true is returned.
   */
  bool evaluate(float x, float& min, float& max) const;

private:
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    if(out)
    {
      std::vector<Point> points(this->points.size());
      for(size_t i = 0; i < points.size(); ++i)
      {
        points[i].x = toDegrees(this->points[i].x);
        points[i].min = toDegrees(this->points[i].min);
        points[i].max = toDegrees(this->points[i].max);
      }
      STREAM(points);
    }
    else if(in)
    {
      STREAM(points);
      for(size_t i = 0; i < points.size(); ++i)
      {
        points[i].x = fromDegrees(points[i].x);
        points[i].min = fromDegrees(points[i].min);
        points[i].max = fromDegrees(points[i].max);
      }
    }
    STREAM_REGISTER_FINISH;
  }
  
public:
  std::vector<Point> points; /**< Points on the piecewise-linear joint limit functions (min and max); */
};
