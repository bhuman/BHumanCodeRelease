/**
 * @file BallModel.cpp
 * Implementation of the BallModel's drawing functions
 */

#include "BallModel.h"
#include "Platform/Time.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Approx.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Blackboard.h"

void BallModel::verify() const
{
  ASSERT(std::isfinite(lastPerception.x()));
  ASSERT(std::isfinite(lastPerception.y()));
  ASSERT(seenPercentage <= 100);

  ASSERT(std::isfinite(estimate.position.x()));
  ASSERT(std::isfinite(estimate.position.y()));
  ASSERT(std::isfinite(estimate.velocity.x()));
  ASSERT(std::isfinite(estimate.velocity.y()));

  ASSERT(std::isnormal(estimate.covariance(0, 0)));
  ASSERT(std::isnormal(estimate.covariance(1, 1)));
  ASSERT(std::isfinite(estimate.covariance(0, 1)));
  ASSERT(std::isfinite(estimate.covariance(1, 0)));
  ASSERT(Approx::isEqual(estimate.covariance(0, 1), estimate.covariance(1, 0), 1e-20f)); 
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
  }

  DEBUG_DRAWING("representation:BallModel:covariance", "drawingOnField")
  {
    COVARIANCE2D("representation:BallModel:covariance", estimate.covariance, estimate.position);
  }

  // drawing of the end position
  DEBUG_DRAWING("representation:BallModel:endPosition", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& fieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      Vector2f position;
      position = BallPhysics::getEndPosition(estimate.position, estimate.velocity, fieldDimensions.ballFriction);
      ColorRGBA violet = ColorRGBA(168, 25, 99, 220); //So you know what you see in the world state...
      CIRCLE("representation:BallModel:endPosition",
             position.x(),  position.y(), 45, 0, // pen width
             Drawings::solidPen, ColorRGBA::black,
             Drawings::solidBrush, violet);
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

BallModelCompressed::BallModelCompressed(const BallModel& ballModel) :
  lastPerception(ballModel.lastPerception.cast<short>()),
  position(ballModel.estimate.position),
  velocity(ballModel.estimate.velocity),
  timeWhenLastSeen(ballModel.timeWhenLastSeen),
  timeWhenDisappeared((ballModel.timeWhenDisappeared & 0x00ffffff) | ballModel.seenPercentage << 24)
{
  covariance[0] = ballModel.estimate.covariance(0, 0);
  covariance[1] = ballModel.estimate.covariance(1, 1);
  covariance[2] = (ballModel.estimate.covariance(0, 1) + ballModel.estimate.covariance(1, 0)) / 2.f;
}

BallModelCompressed::operator BallModel() const
{
  BallModel ballModel;
  ballModel.lastPerception = lastPerception.cast<float>();
  ballModel.estimate.position = position.cast<float>();
  ballModel.estimate.velocity = velocity.cast<float>();
  ballModel.estimate.covariance << covariance[0], covariance[2], covariance[2], covariance[1];
  ballModel.timeWhenLastSeen = timeWhenLastSeen;
  ballModel.timeWhenDisappeared = timeWhenDisappeared & 0x00ffffff;
  ballModel.seenPercentage = static_cast<unsigned char>(timeWhenDisappeared >> 24);
  return ballModel;
}

void BallModel3D::draw() const
{
  // drawing of the ball model in the field view
  DEBUG_DRAWING("representation:BallModel3D", "drawingOnField")
  {
    const Vector3f& position(estimate.position);
    const Vector3f& velocity(estimate.velocity);
    CIRCLE("representation:BallModel3D",
           position.x(), position.y(), 45, 0, // pen width
           Drawings::solidPen, ColorRGBA::red,
           Drawings::solidBrush, ColorRGBA::red);
    ARROW("representation:BallModel3D", position.x(), position.y(),
          position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::red);
  }

  DEBUG_DRAWING3D("representation:BallModel3D", "robot")
  {
    TRANSLATE3D("representation:BallModel3D", 0, 0, -230);
    if(Time::getTimeSince(timeWhenLastSeen) < 5000 && Time::getTimeSince(timeWhenDisappeared) < 1000)
    {
      SPHERE3D("representation:BallModel3D", estimate.position.x(), estimate.position.y(), estimate.position.z(), estimate.radius, ColorRGBA::orange);
    }
  }
}
