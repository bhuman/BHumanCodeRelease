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
  if(riskyMovingEstimateIsValid)
    check(riskyMovingEstimate);
#endif
}

void BallModel::draw() const
{
  // drawing of the ball model in the field view
  DEBUG_DRAWING("representation:BallModel", "drawingOnField")
  {
    const Vector2f& position(estimate.position);
    const Vector2f& velocity(estimate.velocity);
    CIRCLE("representation:BallModel",
           position.x(), position.y(), 45, 0, // pen width
           Drawings::solidPen, ColorRGBA::orange,
           Drawings::solidBrush, ColorRGBA::orange);
    ARROW("representation:BallModel", position.x(), position.y(),
          position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::orange);
    if(riskyMovingEstimateIsValid)
    {
      const Vector2f& riskyVelocity(riskyMovingEstimate.velocity);
      ARROW("representation:BallModel", position.x(), position.y(),
            position.x() + riskyVelocity.x(), position.y() + riskyVelocity.y(), 5, 1, ColorRGBA::blue);
    }
  }

  DEBUG_DRAWING("representation:BallModel:covariance", "drawingOnField")
  {
    COVARIANCE_ELLIPSES_2D("representation:BallModel:covariance", estimate.covariance, estimate.position);
  }

  // drawing of the end position
  DEBUG_DRAWING("representation:BallModel:endPosition", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("BallSpecification"))
    {
      const BallSpecification& ballSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
      Vector2f position = BallPhysics::getEndPosition(estimate.position, estimate.velocity, ballSpecification.friction);
      ColorRGBA violet = ColorRGBA(168, 25, 99, 220); //So you know what you see in the world state...
      CIRCLE("representation:BallModel:endPosition",
             position.x(),  position.y(), 45, 0, // pen width
             Drawings::solidPen, ColorRGBA::black,
             Drawings::solidBrush, violet);
      if(riskyMovingEstimateIsValid)
      {
        position = BallPhysics::getEndPosition(riskyMovingEstimate.position, riskyMovingEstimate.velocity, ballSpecification.friction);
        CIRCLE("representation:BallModel:endPosition",
               position.x(), position.y(), 45, 0, // pen width
               Drawings::solidPen, ColorRGBA::black,
               Drawings::solidBrush, ColorRGBA::cyan);
      }
    }
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
           position.x(), position.y(), 45, 4, // pen width
           Drawings::solidPen, ColorRGBA::gray,
           Drawings::solidBrush, transparentOrange);
    ARROW("representation:GroundTruthBallModel", position.x(), position.y(),
          position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, transparentOrange);
  }
}
