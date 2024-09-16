/**
 * @file MotionRequest.cpp
 *
 * This file implements a struct that represents the request of the behavior to the motion.
 *
 * @author Colin Graf
 * @author Arne Hasselbring
 */

#include "MotionRequest.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Platform/BHAssert.h"

void MotionRequest::draw() const
{
  auto drawObstacleAvoidance = [this]
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

  DEBUG_DRAWING3D("representation:MotionRequest:kickDirection", "field")
  {
    // Only those requests kick the ball
    if((motion == MotionRequest::walkToBallAndKick || motion == MotionRequest::dribble) &&
       Blackboard::getInstance().exists("FieldInterceptBall") && Blackboard::getInstance().exists("RobotPose"))
    {
      const FieldInterceptBall& theFieldInterceptBall = static_cast<FieldInterceptBall&>(Blackboard::getInstance()["FieldInterceptBall"]);
      const RobotPose& theRobotPose = static_cast<RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const Vector2f ballTarget = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(kickLength, targetDirection + theRobotPose.rotation);
      const Vector2f ballTargetMin = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(kickLength, targetDirection + theRobotPose.rotation + directionPrecision.min);
      const Vector2f ballTargetMax = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(kickLength, targetDirection + theRobotPose.rotation + directionPrecision.max);
      LINE3D("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), 0, ballTargetMin.x(), ballTargetMin.y(), 0,
             5, ColorRGBA::blue);
      LINE3D("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), 0, ballTargetMax.x(), ballTargetMax.y(), 0,
             5, ColorRGBA::blue);
      LINE3D("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), 0, ballTarget.x(), ballTarget.y(), 0,
             5, ColorRGBA::red);
      CROSS3D("representation:MotionRequest:kickDirection", ballTarget.x(), ballTarget.y(), 0,
              15, 15, ColorRGBA::red);
    }
  }

  DEBUG_DRAWING("representation:MotionRequest:kickDirection", "drawingOnField")
  {
    // Only those requests kick the ball
    if((motion == MotionRequest::walkToBallAndKick || motion == MotionRequest::dribble) &&
       Blackboard::getInstance().exists("FieldInterceptBall") && Blackboard::getInstance().exists("RobotPose"))
    {
      const FieldInterceptBall& theFieldInterceptBall = static_cast<FieldInterceptBall&>(Blackboard::getInstance()["FieldInterceptBall"]);
      const RobotPose& theRobotPose = static_cast<RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const float useKickLength = std::min(10000.f, kickLength);
      const Vector2f ballTarget = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(useKickLength, targetDirection + theRobotPose.rotation);
      const Vector2f ballTargetMin = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(useKickLength, targetDirection + theRobotPose.rotation + directionPrecision.min);
      const Vector2f ballTargetMax = theFieldInterceptBall.interceptedEndPositionOnField + Vector2f::polar(useKickLength, targetDirection + theRobotPose.rotation + directionPrecision.max);
      LINE("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), ballTargetMin.x(), ballTargetMin.y(),
           5, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), ballTargetMax.x(), ballTargetMax.y(),
           5, Drawings::solidPen, ColorRGBA::blue);
      LINE("representation:MotionRequest:kickDirection", theFieldInterceptBall.interceptedEndPositionOnField.x(), theFieldInterceptBall.interceptedEndPositionOnField.y(), ballTarget.x(), ballTarget.y(),
           5, Drawings::solidPen, ColorRGBA::red);
      CROSS("representation:MotionRequest:kickDirection", ballTarget.x(), ballTarget.y(), 15, 15, Drawings::solidPen, ColorRGBA::red);
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
    for([[maybe_unused]] const auto& segment : obstacleAvoidance.path)
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
    ASSERT(kickLength >= 0.f);
    ASSERT(std::isfinite(kickLength));
  }

  if(motion == dive)
    ASSERT(diveRequest >= static_cast<Dive::Request>(0) && diveRequest < Dive::numOfRequests);

  if(motion == special)
    ASSERT(specialRequest >= static_cast<Special::Request>(0) && specialRequest < Special::numOfRequests);
}
