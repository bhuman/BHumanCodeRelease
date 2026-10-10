/**
 * @file BallModel.cpp
 * Implementation of the BallModel's drawing functions
 */

#include "BallModel.h"
#include "Platform/Time.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Approx.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Framework/Blackboard.h"

void BallModel::verify() const
{
#ifndef NDEBUG
  ASSERT(std::isfinite(lastPerception.x()));
  ASSERT(std::isfinite(lastPerception.y()));
  ASSERT(seenPercentage <= 100);

  const auto check = [](const BallState& estimate)
  {
    ASSERT(std::isfinite(estimate.position.x()));
    ASSERT(std::isfinite(estimate.position.y()));
    ASSERT(std::isfinite(estimate.velocity.x()));
    ASSERT(std::isfinite(estimate.velocity.y()));

    ASSERT(std::isnormal(estimate.covariance(0, 0)));
    ASSERT(std::isnormal(estimate.covariance(1, 1)));
    ASSERT(std::isfinite(estimate.covariance(0, 1)));
    ASSERT(std::isfinite(estimate.covariance(1, 0)));
    ASSERT(Approx::isEqual(estimate.covariance(0, 1), estimate.covariance(1, 0), 1e-20f));
  };
  check(estimate);
#endif
}

void BallModel::draw() const
{
  // drawing of the ball model in the field view
  DEBUG_DRAWING("representation:BallModel", "drawingOnField")
  {
    const Vector2f& position(estimate.position);
    const Vector2f& velocity(estimate.velocity);
    // The ball:
    CIRCLE("representation:BallModel",
           position.x(), position.y(), estimate.radius - 5.f, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange,
           Drawings::solidBrush, ColorRGBA::orange);
    // Dashed line (and arrow) indicating the velocity:
    if(velocity != Vector2f::Zero() && Blackboard::getInstance().exists("BallSpecification"))
    {
      const BallSpecification& ballSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
      Vector2f endPosition = BallPhysics::getEndPosition(position, velocity, ballSpecification);
      LINE("representation:BallModel", position.x(), position.y(), endPosition.x(), endPosition.y(), 40, Drawings::dashedPen, ColorRGBA::orange);
      const float arrowLength = 400.f;
      const float ballRollDistance = (endPosition - position).norm();
      if(ballRollDistance > arrowLength)
      {
        const Vector2f arrowStart = position + (endPosition - position).normalized() * (ballRollDistance - arrowLength);
        ARROW("representation:BallModel", arrowStart.x(), arrowStart.y(),
              endPosition.x(), endPosition.y(), 40, Drawings::solidPen, ColorRGBA::orange);
      }
    }
  }

  DEBUG_DRAWING("representation:BallModel:covariance", "drawingOnField")
  {
    COVARIANCE_ELLIPSES_2D("representation:BallModel:covariance", estimate.covariance, estimate.position);
  }

  DEBUG_DRAWING3D("representation:BallModel", "robot")
  {
    TRANSLATE3D("representation:BallModel", 0, 0, -230);
    if(Time::getTimeSince(timeWhenLastSeen) < 5000 && Time::getTimeSince(timeWhenDisappeared) < 1000)
    {
      SPHERE3D("representation:BallModel", estimate.position.x(), estimate.position.y(), estimate.radius, estimate.radius, ColorRGBA::orange);
      LINE3D("representation:BallModel", 0, 0, 1.f, estimate.position.x(), estimate.position.y(), 1.f, 5.f, ColorRGBA::orange);
    }
  }
}

void GroundTruthBallModel::draw() const
{
  DEBUG_DRAWING("representation:GroundTruthBallModel", "drawingOnField")
  {
    const Vector2f& position(estimate.position);
    const Vector2f& velocity(estimate.velocity);
    ColorRGBA transparentOrange(255, 128, 0, 128);
    CIRCLE("representation:GroundTruthBallModel",
           position.x(), position.y(), estimate.radius - 5.f, 4, // pen width
           Drawings::solidPen, ColorRGBA::gray,
           Drawings::solidBrush, transparentOrange);
    ARROW("representation:GroundTruthBallModel", position.x(), position.y(),
          position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, transparentOrange);
  }
}
