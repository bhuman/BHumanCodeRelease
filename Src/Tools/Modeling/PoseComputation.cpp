/**
 * @file PoseComputation.cpp
 *
 * Some functions that compute a robot pose on the field given
 * some observations of field elements (goals, center circle, ...)
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "PoseComputation.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Geometry.h"

Pose2f PoseComputation::computePoseFromTwoGoalposts(const Vector2f& pl, const Vector2f& pr,
                                                    const Vector2f& plWorld, const Vector2f& prWorld, float goalpostRadius)
{
  // Determine x-coordinate by computing the distance to the ground line (i.e. the line that connects both posts)
  const Vector2f lineDirection = pr - pl;
  const float distanceToGroundLine = std::abs(Geometry::getDistanceToLine(Geometry::Line(pl, lineDirection), Vector2f::Zero()));
  Pose2f result;
  result.translation.x() = plWorld.x() > 0.f ? plWorld.x() - distanceToGroundLine : plWorld.x() + distanceToGroundLine;

  // text ...
  const float absYOffsetToLeft  = std::sqrt(sqr(pl.norm()) - sqr(plWorld.x() - result.translation.x()));
  const float absYOffsetToRight = std::sqrt(sqr(pr.norm()) - sqr(prWorld.x() - result.translation.x()));
  const float goalWidth = std::abs(plWorld.y()) * 2.f;

  // y between goalposts
  float y1, y2;
  if(absYOffsetToLeft < goalWidth && absYOffsetToRight < goalWidth)
  {
    if(plWorld.y() < 0.f)
    {
      y1 = plWorld.y() + absYOffsetToLeft;
      y2 = prWorld.y() - absYOffsetToRight;
    }
    else
    {
      y1 = plWorld.y() - absYOffsetToLeft;
      y2 = prWorld.y() + absYOffsetToRight;
    }
  }
  // y on the right side
  else if(absYOffsetToLeft > absYOffsetToRight)
  {
    if(plWorld.y() < 0.f)
    {
      y1 = plWorld.y() + absYOffsetToLeft;
      y2 = prWorld.y() + absYOffsetToRight;
    }
    else
    {
      y1 = plWorld.y() - absYOffsetToLeft;
      y2 = prWorld.y() - absYOffsetToRight;
    }
  }
  // y on the left side
  else // if(absYOffsetToLeft < absYOffsetToRight)
  {
    if(plWorld.y() < 0.f)
    {
      y1 = plWorld.y() - absYOffsetToLeft;
      y2 = prWorld.y() - absYOffsetToRight;
    }
    else
    {
      y1 = plWorld.y() + absYOffsetToLeft;
      y2 = prWorld.y() + absYOffsetToRight;
    }

  }
  result.translation.y() = (y1 + y2) / 2.f;

  // Rotation
  const float origLAngle = (plWorld - result.translation).angle();
  const float observedLAngle = pl.angle();
  Angle robotLAngle = origLAngle - observedLAngle;
  robotLAngle.normalize();

  const float origRAngle = (prWorld - result.translation).angle();
  const float observedRAngle = pr.angle();
  Angle robotRAngle = origRAngle - observedRAngle;
  robotRAngle.normalize();

  result.rotation = std::atan2(std::sin(robotRAngle) + std::sin(robotLAngle), std::cos(robotRAngle) + std::cos(robotLAngle));
  return result;
}