/**
 * @file AlternativeRobotPoseHypothesis.cpp
 *
 * A new robot pose, based on recently observed field features.
 * This pose is meant to be used for sensor resetting inside the SelfLocalization
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "AlternativeRobotPoseHypothesis.h"
#include "Tools/Debugging/DebugDrawings.h"

void AlternativeRobotPoseHypothesis::draw() const
{
  DEBUG_DRAWING("representation:AlternativeRobotPoseHypothesis", "drawingOnField")
  {
    if(isValid)
    {
      ColorRGBA col = isInOwnHalf ? ColorRGBA(180, 180, 255) : ColorRGBA(255, 0, 0);
      DRAW_ROBOT_POSE("representation:AlternativeRobotPoseHypothesis", pose, ColorRGBA(0, 0, 0));
      CIRCLE("representation:AlternativeRobotPoseHypothesis", pose.translation.x(), pose.translation.y(),
             500, 40, Drawings::solidPen, col, Drawings::noBrush, col);
      DRAWTEXT("representation:AlternativeRobotPoseHypothesis", pose.translation.x(), pose.translation.y() + 700, 200,
               ColorRGBA(0, 0, 0), numOfContributingObservations);
    }
  }
}

void AlternativeRobotPoseHypothesis::verify() const
{
  ASSERT(std::isfinite(pose.translation.x()));
  ASSERT(std::isfinite(pose.translation.y()));
  ASSERT(std::isfinite(pose.rotation));
  ASSERT(pose.rotation >= -pi);
  ASSERT(pose.rotation <= pi);
}
