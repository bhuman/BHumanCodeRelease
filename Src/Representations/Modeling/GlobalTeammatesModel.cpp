/**
* @file GlobalTeammatesModel.cpp
*
* Implementation of a representation that stores the estimated states
* of the teammates.
* The representation is a result of the combination of receveived position information
* and own observations of teammates.
*
* @author Tim Laue
*/

#include "Framework/Blackboard.h"
#include "GlobalTeammatesModel.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/GameState.h"

Vector2f GlobalTeammatesModel::TeammateEstimate::getFuturePosition(unsigned time) const
{
  return Teammate::getEstimatedPosition(pose, relativeWalkTarget, speed, time);
};

void GlobalTeammatesModel::verify() const
{
  for(std::size_t i=0; i < teammates.size(); i++)
  {
    const TeammateEstimate& tm = teammates.at(i);
    [[maybe_unused]] const Pose2f& p = tm.pose;
    ASSERT(std::isfinite(p.translation.x()));
    ASSERT(std::isfinite(p.translation.y()));
    ASSERT(std::isfinite(p.rotation));
    ASSERT(p.rotation >= -pi);
    ASSERT(p.rotation <= pi);
    ASSERT(std::isfinite(tm.relativeWalkTarget.x()));
    ASSERT(std::isfinite(tm.relativeWalkTarget.y()));
    ASSERT(std::isfinite(tm.speed));
    ASSERT(tm.speed >= 0);
    if(i > 0)
    {
      [[maybe_unused]] const TeammateEstimate& tmPrev = teammates.at(i-1);
      ASSERT(tm.playerNumber > tmPrev.playerNumber);
    }
  }
}

void GlobalTeammatesModel::draw() const
{
  DEBUG_DRAWING("representation:GlobalTeammatesModel", "drawingOnField")
  {
    const ColorRGBA ownTeamColorForDrawing = ColorRGBA::fromTeamColor(
      static_cast<int>(Blackboard::getInstance().exists("GameState") ? static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.color : GameState::Team::Color::black));
    ColorRGBA ownTeamColorForCircleDrawing = ownTeamColorForDrawing;
    ownTeamColorForCircleDrawing.a = 100;
    for(const TeammateEstimate& t : teammates)
    {
      const Vector2f dirVec = t.pose * Vector2f(200, 0);
      CIRCLE("representation:GlobalTeammatesModel", t.pose.translation.x(), t.pose.translation.y(), 300, 1, Drawings::solidPen, ownTeamColorForCircleDrawing, Drawings::solidBrush, ownTeamColorForCircleDrawing);
      ROBOT("representation:GlobalTeammatesModel", t.pose, dirVec, dirVec, 1.f, ownTeamColorForDrawing,
            ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
      DRAW_TEXT("representation:GlobalTeammatesModel", t.pose.translation.x() + 200, t.pose.translation.y(),
            250, ColorRGBA::white, t.playerNumber);
    }
  }
}
