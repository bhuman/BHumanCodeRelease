/**
* @file GlobalOpponentsModel.cpp
*
* Declaration of a representation that stores the estimated states
* of the opponent robots.
* For each robot, only the position (and not the size/width) is considered. Thus, this model should
* only be used for tactical purposes but not for stuff like obstacle avoidance.
*
* @author Tim Laue
* @author Michelle Gusev
*/

#include "GlobalOpponentsModel.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Math/Approx.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Tools/Modeling/Obstacle.h"

void GlobalOpponentsModel::verify() const
{
  for([[maybe_unused]] const auto& opponent : opponents)
  {
    ASSERT(std::isfinite(opponent.position.x()));
    ASSERT(std::isfinite(opponent.position.y()));
  }
}

void GlobalOpponentsModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GlobalOpponentsModel", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:GlobalOpponentsModel", "field");

  const ColorRGBA oppCol = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
    static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red));

  for(const auto& opponent : opponents)
  {
    const Vector2f& center = opponent.position;
    const float obstacleRadius = 300.f;
    const ColorRGBA fillColor = opponent.unidentified ? ColorRGBA(67,67,67,200) : ColorRGBA(oppCol.r, oppCol.g, oppCol.b,128);

    CYLINDER3D("representation:GlobalOpponentsModel", center.x(), center.y(), -210, 0, 0, 0, 2 * obstacleRadius, 10, fillColor);

    CIRCLE("representation:GlobalOpponentsModel", center.x(), center.y(), obstacleRadius, 1, Drawings::solidPen, fillColor, Drawings::solidBrush, fillColor);
    if(opponent.unidentified)
    {
      DRAW_TEXT("representation:GlobalOpponentsModel", center.x()-20, center.y()-50, obstacleRadius, ColorRGBA::black, "?");
    }
    else
    {
      const Vector2f dirVec = center + Vector2f(-200, 0);
      ROBOT("representation:GlobalOpponentsModel", center, dirVec, dirVec, 1.f, fillColor, ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
    }
  }
}
