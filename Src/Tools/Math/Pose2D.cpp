/**
* @file Pose2D.cpp
* Contains class Pose2D
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#include "Pose2D.h"
#include "Tools/Range.h"
#include "Tools/Math/Random.h"

Pose2D Pose2D::random(const Range<float>& x,
                      const Range<float>& y,
                      const Range<float>& angle)
{
  // angle should even work in wrap around case!
  return Pose2D(float(::randomFloat() * (angle.max - angle.min) + angle.min),
                Vector2<>(float(::randomFloat() * (x.max - x.min) + x.min),
                          float(::randomFloat() * (y.max - y.min) + y.min)));
}
