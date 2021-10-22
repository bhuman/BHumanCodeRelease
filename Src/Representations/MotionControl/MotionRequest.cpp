/**
 * @file MotionRequest.cpp
 *
 * This file implements a struct that represents the request of the behavior to the motion.
 *
 * @author Colin Graf
 * @author Arne Hasselbring
 */

#include "MotionRequest.h"
#include "Platform/BHAssert.h"
#include "Tools/Debugging/DebugDrawings.h"

void MotionRequest::draw() const
{
  auto drawObstacleAvoidance = [this]()
  {
    if(obstacleAvoidance.avoidance != Vector2f::Zero())
      ARROW("representation:MotionRequest", 0, 0, obstacleAvoidance.avoidance.x() * 100.f, obstacleAvoidance.avoidance.y() * 100.f, 0, Drawings::solidPen, ColorRGBA(0x8e, 0, 0, 127));
  };

  DEBUG_DRAWING("representation:MotionRequest", "drawingOnField")
  {
    switch(motion)
    {
      case walkAtAbsoluteSpeed:
      case walkAtRelativeSpeed:
      {
        Vector2f translation = motion == walkAtAbsoluteSpeed ? walkSpeed.translation * 10.f : walkSpeed.translation * 1000.f;
        ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        if(walkSpeed.rotation != 0.0f)
        {
          translation.x() = translation.norm();
          translation.y() = 0;
          translation.rotate(walkSpeed.rotation);
          ARROW("representation:MotionRequest", 0, 0, translation.x(), translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        }
        break;
      }
      case walkToPose:
      {
        LINE("representation:MotionRequest", 0, 0, walkTarget.translation.x(), walkTarget.translation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        CROSS("representation:MotionRequest", walkTarget.translation.x(), walkTarget.translation.y(), 50, 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        Vector2f rotation(500.f, 0.f);
        rotation.rotate(walkTarget.rotation);
        ARROW("representation:MotionRequest", walkTarget.translation.x(), walkTarget.translation.y(), walkTarget.translation.x() + rotation.x(), walkTarget.translation.y() + rotation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        drawObstacleAvoidance();
        break;
      }
      case walkToBallAndKick:
      case dribble:
      {
        LINE("representation:MotionRequest", 0, 0, ballEstimate.position.x(), ballEstimate.position.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        CROSS("representation:MotionRequest", ballEstimate.position.x(), ballEstimate.position.y(), 50, 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
        Vector2f rotation(500.f, 0.f);
        rotation.rotate(targetDirection);
        ARROW("representation:MotionRequest", ballEstimate.position.x(), ballEstimate.position.y(), ballEstimate.position.x() + rotation.x(), ballEstimate.position.y() + rotation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
        drawObstacleAvoidance();
        break;
      }
    }
  }
}

void MotionRequest::verify() const
{
  ASSERT(motion >= static_cast<Motion>(0) && motion < numOfMotions);
  if(motion == walkAtAbsoluteSpeed || motion == walkAtRelativeSpeed || motion == walkToPose || motion == walkToBallAndKick || motion == dribble)
    ASSERT(walkSpeed.isFinite());

  if(motion == walkToPose)
    ASSERT(walkTarget.isFinite());

  if(motion == walkToPose || motion == walkToBallAndKick || motion == dribble)
  {
    ASSERT(std::isfinite(obstacleAvoidance.avoidance.x()));
    ASSERT(std::isfinite(obstacleAvoidance.avoidance.y()));
    for(const auto& segment : obstacleAvoidance.path)
    {
      ASSERT(std::isfinite(segment.obstacle.center.x()));
      ASSERT(std::isfinite(segment.obstacle.center.y()));
      ASSERT(std::isfinite(segment.obstacle.radius));
      ASSERT(segment.obstacle.radius >= 0.f);
    }
  }

  if(motion == walkToBallAndKick || motion == dribble)
  {
    ASSERT(std::isfinite(targetDirection));
    ASSERT(targetDirection >= -pi && targetDirection <= pi);
    ASSERT(std::isfinite(directionPrecision.min));
    ASSERT(std::isfinite(directionPrecision.max));
  }

  if(motion == walkToBallAndKick)
  {
    ASSERT(kickType >= static_cast<KickInfo::KickType>(0) && kickType < KickInfo::numOfKickTypes);
    ASSERT(kickPower >= 0.f && kickPower <= 1.f);
  }

  if(motion == keyframeMotion)
    ASSERT(keyframeMotionRequest.keyframeMotion >= KeyframeMotionRequest::firstNonGetUpAction && keyframeMotionRequest.keyframeMotion < KeyframeMotionRequest::numOfKeyframeMotionIDs);
}
