/**
* @file BallModel.cpp
* Implementation of the BallModel's drawing functions
*/

#include "BallModel.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallModel::draw() const
{
  // drawing of the ball model in the field view
  DECLARE_DEBUG_DRAWING("representation:BallModel", "drawingOnField",
    const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    CIRCLE("representation:BallModel",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorClasses::red,
            Drawings::bs_solid,
            ColorClasses::red);
    ARROW("representation:BallModel", position.x, position.y,
          position.x + velocity.x, position.y + velocity.y, 5, 1, ColorClasses::red);
  );
}

void BallModel::draw3D(const Pose2D& robotPose) const
{
  // drawing og the ball model in the scene
  DECLARE_DEBUG_DRAWING3D("representation:BallModel", "field",
  {
    if(SystemCall::getTimeSince(timeWhenLastSeen) < 5000 && SystemCall::getTimeSince(timeWhenDisappeared) < 1000)
    {
      Vector2<> ballRelToWorld = robotPose * estimate.position;
      SPHERE3D("representation:BallModel", ballRelToWorld.x, ballRelToWorld.y, 35.f, 35.f, ColorClasses::orange);
      LINE3D("representation:BallModel", robotPose.translation.x, robotPose.translation.y, 1.f, ballRelToWorld.x, ballRelToWorld.y, 1.f, 5.f, ColorClasses::orange);
    }
  });
}


void BallModel::drawEndPosition(float ballFriction) const
{
  // drawing of the end position
  DECLARE_DEBUG_DRAWING("representation:BallModel:endPosition", "drawingOnField",
    Vector2<> position = estimate.getEndPosition(ballFriction);
    CIRCLE("representation:BallModel:endPosition",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorClasses::black,
            Drawings::bs_solid,
            ColorRGBA(168, 25, 99, 220));
  );
}

void GroundTruthBallModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GroundTruthBallModel", "drawingOnField",
    const Vector2<>& position(estimate.position);
    const Vector2<>& velocity(estimate.velocity);
    CIRCLE("representation:GroundTruthBallModel",
            position.x,
            position.y,
            45,
            0, // pen width
            Drawings::ps_solid,
            ColorRGBA(255, 128, 0, 192),
            Drawings::bs_solid,
            ColorRGBA(255, 128, 0, 192));
    ARROW("representation:GroundTruthBallModel", position.x, position.y,
          position.x + velocity.x, position.y + velocity.y, 5, 1, ColorRGBA(255, 128, 0, 192));
  );
}

BallModelCompressed::BallModelCompressed(const BallModel& ballModel)
: lastPerception(ballModel.lastPerception),
  position(ballModel.estimate.position),
  velocity(ballModel.estimate.velocity),
  timeWhenLastSeen(ballModel.timeWhenLastSeen),
timeWhenDisappeared(ballModel.timeWhenDisappeared) {}

BallModelCompressed::operator BallModel() const
{
  BallModel ballModel;
  ballModel.lastPerception = Vector2<>(lastPerception);
  ballModel.estimate.position = Vector2<>(position);
  ballModel.estimate.velocity = Vector2<>(velocity);
  ballModel.timeWhenLastSeen = timeWhenLastSeen;
  ballModel.timeWhenDisappeared = timeWhenDisappeared;
  return ballModel;
}

