/**
 * @file BallContactWithRobot.cpp
 *
 * Implementation of a representation that represents information about
 * the last contact of the robot and the ball.
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#include "BallContactWithRobot.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

void BallContactWithRobot::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallContactWithRobot", "drawingOnField");
  if(Blackboard::getInstance().exists("FrameInfo"))
  {
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
    const float maxTime = 1500;
    const float timeSinceContact = static_cast<float>(theFrameInfo.getTimeSince(timeOfLastContact));
    if(timeSinceContact < maxTime)
    {
      const float minSize = 70;
      const float maxSize = 400;
      const float minAlpha = 200;
      const float maxAlpha = 20;
      const unsigned char alphaValue = static_cast<unsigned char>(mapToRange(timeSinceContact, 0.f, maxTime, minAlpha, maxAlpha));
      const float circleSize = mapToRange(timeSinceContact, 0.f, maxTime, minSize, maxSize);
      ColorRGBA collisionColor;
      if(contactType == leftFoot)
        collisionColor = ColorRGBA(255, 0, 0, alphaValue);
      else if(contactType == rightFoot)
        collisionColor = ColorRGBA(1, 158, 224, alphaValue);
      else if(contactType == center)
        collisionColor = ColorRGBA(255, 236, 0, alphaValue);
      else
        collisionColor = ColorRGBA(128, 128, 128, alphaValue);
      CIRCLE("representation:BallContactWithRobot", newPosition.x(), newPosition.y(), circleSize, 20, Drawings::solidPen, collisionColor, Drawings::solidBrush, collisionColor);
    }
  }
}

void BallContactWithRobot::verify() const
{
  ASSERT(std::isfinite(newPosition.x()));
  ASSERT(std::isfinite(newPosition.y()));
  ASSERT(std::isfinite(newVelocity.x()));
  ASSERT(std::isfinite(newVelocity.y()));
  ASSERT(std::isnormal(addVelocityCov.x()));
  ASSERT(std::isnormal(addVelocityCov.y()));
}
