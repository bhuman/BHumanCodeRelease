/**
* @file GlobalOpponentsModel.cpp
*
* Implementation of a representation that stores the estimated states
* of the opponent robots.
* The representation is a result of the combined estimates of
* all teammates.
*
* @author Tim Laue
* @author Michelle Gusev
*/

#include "GlobalOpponentsModel.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Math/Approx.h"
#include "Tools/Modeling/Obstacle.h"
#include "Framework/Blackboard.h"

void GlobalOpponentsModel::verify() const
{
  DECLARE_DEBUG_RESPONSE("representation:GlobalOpponentsModel:verify");
  for([[maybe_unused]] const auto& opponent : opponents)
  {
    ASSERT(std::isfinite(opponent.position.x()));
    ASSERT(std::isfinite(opponent.position.y()));

    ASSERT(std::isfinite(opponent.left.x()));
    ASSERT(std::isfinite(opponent.left.y()));

    ASSERT(std::isfinite(opponent.right.x()));
    ASSERT(std::isfinite(opponent.right.y()));
  }
}

void GlobalOpponentsModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:rectangle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:centerCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:leftCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:rightCross", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:leftRight", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel:circle", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel", "drawingOnField");

  const ColorRGBA opponentColor = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
    static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red));

  for(const auto& opponent : opponents)
  {
    const Vector2f& center = opponent.position;
    const Vector2f& left = opponent.left;
    const Vector2f& right = opponent.right;

    CYLINDER3D("representation:GlobalOpponentsModel", center.x(), center.y(), -210, 0, 0, 0, (left - right).norm(), 10, opponentColor);
    CROSS("representation:GlobalOpponentsModel:centerCross", center.x(), center.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, opponentColor);
    CROSS("representation:GlobalOpponentsModel:leftCross", left.x(), left.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, ColorRGBA::cyan);
    CROSS("representation:GlobalOpponentsModel:rightCross", right.x(), right.y(), Obstacle::getRobotDepth(), 10, Drawings::solidPen, ColorRGBA::magenta);

    float obstacleRadius = (left - right).norm() * .5f;
    Angle robotRotation = Blackboard::getInstance().exists("RobotPose") ? static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]).rotation : Angle();
    Vector2f frontRight(-Obstacle::getRobotDepth(), -obstacleRadius);
    frontRight = center + frontRight;
    RECTANGLE2("representation:GlobalOpponentsModel:rectangle", frontRight, obstacleRadius * 2, obstacleRadius * 2, -robotRotation, 16, Drawings::PenStyle::solidPen, ColorRGBA::black, Drawings::solidBrush, opponentColor);

    LINE("representation:GlobalOpponentsModel:leftRight", center.x(), center.y(), left.x(), left.y(), 20, Drawings::dottedPen, opponentColor);
    LINE("representation:GlobalOpponentsModel:leftRight", center.x(), center.y(), right.x(), right.y(), 20, Drawings::dottedPen, opponentColor);
    CIRCLE("representation:GlobalOpponentsModel:circle", center.x(), center.y(), obstacleRadius, 10, Drawings::dottedPen, opponentColor, Drawings::noBrush, opponentColor);
  }
}
