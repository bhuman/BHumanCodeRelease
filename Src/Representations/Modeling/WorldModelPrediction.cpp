/**
 * @file WorldModelPrediction.cpp
 *
 * This file implements a class that represents data computed by modeling modules in the previous frame and
 * that is required before the excution of these modules in the current frame.
 * All positions have received odometry updates. If the ball was rolling, a dynamic update has been performed, too.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "WorldModelPrediction.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"

void WorldModelPrediction::draw() const
{
  DEBUG_DRAWING("representation:WorldModelPrediction:ballPosition", "drawingOnField")
  {
    CIRCLE("representation:WorldModelPrediction:ballPosition", ballPosition.x(), ballPosition.y(),
           60, 0, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
  }
}

void WorldModelPrediction::verify() const
{
  ASSERT(std::isfinite(ballPosition.x()));
  ASSERT(std::isfinite(ballPosition.y()));
  ASSERT(std::isfinite(ballVelocity.x()));
  ASSERT(std::isfinite(ballVelocity.y()));
  ASSERT(std::isfinite(robotPose.translation.x()));
  ASSERT(std::isfinite(robotPose.translation.y()));
  ASSERT(std::isfinite(robotPose.rotation));
}
