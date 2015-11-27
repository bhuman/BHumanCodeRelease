/**
 * @file Representations/Modeling/TeamPlayersModel.cpp
 * Implementation of a debug drawing of the combined world model
 * @author Katharina Gillmann
 */

#include "TeamPlayersModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Modeling/Obstacle.h"

void TeamPlayersModel::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamPlayersModel:ownRobots", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:TeamPlayersModel:oppRobots", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:TeamPlayersModel:oppRobotsCovariance", "drawingOnField");

  DEBUG_DRAWING3D("representation:TeamPlayersModel", "field")
  {
    // draw robots
    for(const auto& obstacle : obstacles)
    {
      if(obstacle.type == Obstacle::teammate || obstacle.type == Obstacle::fallenTeammate)
        CYLINDER3D("representation:TeamPlayersModel", obstacle.center.x(), obstacle.center.y(), 0.0f, 0.0f, 0.0f, 0.0f, 35.0f, 60.0f, ColorRGBA(0, 0, 0, 100));
      else
        CYLINDER3D("representation:TeamPlayersModel", obstacle.center.x(), obstacle.center.y(), 0.0f, 0.0f, 0.0f, 0.0f, 35.0f, 20.0f, ColorRGBA(0, 0, 255, 100));
    }
  }

  COMPLEX_DRAWING("representation:TeamPlayersModel:ownRobots")
  {
    for(const auto& obstacle : obstacles)
    {
      if(obstacle.type == Obstacle::teammate || obstacle.type == Obstacle::fallenTeammate)
      {
        CROSS("representation:TeamPlayersModel:ownRobots", obstacle.center.x(), obstacle.center.y(), 20, 40, Drawings::solidPen, ColorRGBA::black);
        CIRCLE("representation:TeamPlayersModel:ownRobots", obstacle.center.x(), obstacle.center.y(), 500, 20, Drawings::solidPen, ColorRGBA::blue, Drawings::noBrush, ColorRGBA());
      }
    }
  }

  COMPLEX_DRAWING("representation:TeamPlayersModel:oppRobots")
  {
    for(const auto& obstacle : obstacles)
    {
      if(obstacle.type != Obstacle::teammate && obstacle.type != Obstacle::fallenTeammate)
      {
        float xExpansion, yExpansion, rotation;
        Covariance::errorEllipse(obstacle.covariance, xExpansion, yExpansion, rotation);
        CROSS("representation:TeamPlayersModel:oppRobots", obstacle.center.x(), obstacle.center.y(), 20, 40, Drawings::solidPen, ColorRGBA::blue);
        ELLIPSE("representation:TeamPlayersModel:oppRobotsCovariance", obstacle.center, sqrt(3.0f) * xExpansion, sqrt(3.0f) * yExpansion, rotation,
                10, Drawings::solidPen, ColorRGBA(100, 100, 255, 100), Drawings::solidBrush, ColorRGBA(0, 0, 255, 100));
        ELLIPSE("representation:TeamPlayersModel:oppRobotsCovariance", obstacle.center, sqrt(2.0f) * xExpansion, sqrt(2.0f) * yExpansion, rotation,
                10, Drawings::solidPen, ColorRGBA(150, 150, 100, 100), Drawings::solidBrush, ColorRGBA(0, 255, 0, 100));
        ELLIPSE("representation:TeamPlayersModel:oppRobotsCovariance", obstacle.center, xExpansion, yExpansion, rotation,
                10, Drawings::solidPen, ColorRGBA(255, 100, 100, 100), Drawings::solidBrush, ColorRGBA(255, 255, 0, 100));
        CIRCLE("representation:TeamPlayersModel:oppRobots", obstacle.center.x(), obstacle.center.y(), 600, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::noBrush, ColorRGBA());
      }
    }
  }
}
