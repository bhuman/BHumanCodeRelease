/**
 * @file ObstaclesFieldPercept.cpp
 *
 * This file implements a representation that lists the obstacles that were detected in
 * the current image in robot-relative field coordinates. Only obstacles the lower
 * end of which were visible are represented.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 */

#include "ObstaclesFieldPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Framework/Blackboard.h"
#include <cmath>

void ObstaclesFieldPercept::draw() const
{
  static const ColorRGBA penColors[] =
  {
    ColorRGBA::gray,
    ColorRGBA::white,
    ColorRGBA::white
  };

  // Define standard colors for drawing, try to get original team color information:
  const ColorRGBA teamColors[] =
  {
    ColorRGBA::white,
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.fieldPlayerColor : GameState::Team::Color::black)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::red)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.goalkeeperColor : GameState::Team::Color::purple)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.goalkeeperColor : GameState::Team::Color::blue))
  };

  // Draw robots in field view:
  DEBUG_DRAWING("representation:ObstaclesFieldPercept:field", "drawingOnField")
  {
    for(const Obstacle& obstacle : obstacles)
    {
      const float radius = std::max((obstacle.left - obstacle.right).norm() / 2.f, 50.f);
      CIRCLE("representation:ObstaclesFieldPercept:field", obstacle.center.x(), obstacle.center.y(), radius, 10, Drawings::solidPen,
             penColors[obstacle.type], Drawings::solidBrush, teamColors[obstacle.type]);
    }
  }

  // Drawing in image (is this really useful here?)
  DEBUG_DRAWING("representation:ObstaclesFieldPercept:image", "drawingOnImage")
  {
    if(Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("CameraMatrix")
       && Blackboard::getInstance().exists("ImageCoordinateSystem"))
    {
      const CameraInfo& cameraInfo = static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const CameraMatrix& cameraMatrix = static_cast<CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      const ImageCoordinateSystem& imageCoordinateSystem = static_cast<ImageCoordinateSystem&>(Blackboard::getInstance()["ImageCoordinateSystem"]);

      for(const Obstacle& obstacle : obstacles)
      {
        Vector2f pointsInImage[3];
        if(Transformation::robotToImage(obstacle.left, cameraMatrix, cameraInfo, pointsInImage[0])
           && Transformation::robotToImage(obstacle.right, cameraMatrix, cameraInfo, pointsInImage[1])
           && Transformation::robotToImage(Vector3f(obstacle.left.x(), obstacle.left.y(), 580.f), cameraMatrix, cameraInfo, pointsInImage[2]))
        {
          for(int i = 0; i < 3; ++i)
            pointsInImage[i] = imageCoordinateSystem.fromCorrected(pointsInImage[i]);
          if(obstacle.type == unknown)
          {
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 2.5f, pointsInImage[0].y() + 2.5f, pointsInImage[1].x() + 2.5f, pointsInImage[2].y() - 2.5f, 1, Drawings::solidPen, ColorRGBA::red);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 1.5f, pointsInImage[0].y() + 1.5f, pointsInImage[1].x() + 1.5f, pointsInImage[2].y() - 1.5f, 1, Drawings::solidPen, ColorRGBA::orange);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() - 0.5f, pointsInImage[0].y() + 0.5f, pointsInImage[1].x() + 0.5f, pointsInImage[2].y() - 0.5f, 1, Drawings::solidPen, ColorRGBA::yellow);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 0.5f, pointsInImage[0].y() - 0.5f, pointsInImage[1].x() - 0.5f, pointsInImage[2].y() + 0.5f, 1, Drawings::solidPen, ColorRGBA::green);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 1.5f, pointsInImage[0].y() - 1.5f, pointsInImage[1].x() - 1.5f, pointsInImage[2].y() + 1.5f, 1, Drawings::solidPen, ColorRGBA::blue);
            RECTANGLE("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 2.5f, pointsInImage[0].y() - 2.5f, pointsInImage[1].x() - 2.5f, pointsInImage[2].y() + 2.5f, 1, Drawings::solidPen, ColorRGBA::violet);
          }
          else
          {
            RECTANGLE("representation:ObstaclesFieldPercept:image",
                      pointsInImage[0].x(), pointsInImage[0].y(), pointsInImage[1].x(), pointsInImage[2].y(),
                      6, Drawings::solidPen, teamColors[obstacle.type]);
          }
          //DRAW_TEXT("representation:ObstaclesFieldPercept:image", pointsInImage[0].x() + 2, pointsInImage[0].y() + 14, 10, teamColors[obstacle.type], "Team Conf: " << std::to_string(obstacle.confidence).substr(0, 4));
        }
      }
    }
  }
}

void ObstaclesFieldPercept::verify() const
{
  for([[maybe_unused]] const Obstacle& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));
    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));
    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));
  }
}
