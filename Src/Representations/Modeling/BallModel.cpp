/**
* @file BallModel.cpp
* Implementation of the BallModel's drawing functions
*/

#include "BallModel.h"
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
            ColorRGBA::red,
            Drawings::bs_solid,
            ColorRGBA::red);
    ARROW("representation:BallModel", position.x, position.y,
          position.x + velocity.x, position.y + velocity.y, 5, 1, ColorRGBA::red);
  );

  DECLARE_DEBUG_DRAWING3D("representation:BallModel", "robot",
  {
    TRANSLATE3D("representation:BallModel", 0, 0, -230);
    if(SystemCall::getTimeSince(timeWhenLastSeen) < 5000 && SystemCall::getTimeSince(timeWhenDisappeared) < 1000)
    {
      SPHERE3D("representation:BallModel", estimate.position.x, estimate.position.y, 35.f, 35.f, ColorRGBA::orange);
      LINE3D("representation:BallModel", 0, 0, 1.f, estimate.position.x, estimate.position.y, 1.f, 5.f, ColorRGBA::orange);
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
            ColorRGBA::black,
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
  timeWhenDisappeared((ballModel.timeWhenDisappeared & 0x00ffffff) | ballModel.seenPercentage << 24) {}

BallModelCompressed::operator BallModel() const
{
  BallModel ballModel;
  ballModel.lastPerception = Vector2<>(lastPerception);
  ballModel.estimate.position = Vector2<>(position);
  ballModel.estimate.velocity = Vector2<>(velocity);
  ballModel.timeWhenLastSeen = timeWhenLastSeen;
  ballModel.timeWhenDisappeared = timeWhenDisappeared & 0x00ffffff;
  ballModel.seenPercentage = (unsigned char) (timeWhenDisappeared >> 24);
  return ballModel;
}

