/**
 * @file TeamPose.cpp
 *
 * TODO: Description
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaﬂ</a>
 */

#include "TeamPose.h"
#include "Tools/Debugging/DebugDrawings.h"

void TeamPose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamPose:pose", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:TeamPose:covariance", "drawingOnField");

  if(update)
  {
    DRAW_ROBOT_POSE("representation:TeamPose:pose", pose, ColorRGBA::violet);
    COVARIANCE2D("representation:TeamPose:covariance", covariance, pose.translation);
  }
}

void TeamPose::verify() const
{
  ASSERT(std::isfinite(pose.rotation));
  ASSERT(std::isfinite(pose.translation.x()));
  ASSERT(std::isfinite(pose.translation.y()));

  ASSERT(std::isnormal(covariance(0, 0)));
  ASSERT(std::isnormal(covariance(1, 1)));
  ASSERT(std::isnormal(covariance(2, 2)));
  ASSERT(std::isfinite(covariance(0, 1)));
  ASSERT(std::isfinite(covariance(0, 2)));
  ASSERT(std::isfinite(covariance(1, 0)));
  ASSERT(std::isfinite(covariance(1, 2)));
  ASSERT(std::isfinite(covariance(2, 0)));
  ASSERT(std::isfinite(covariance(2, 1)));
}