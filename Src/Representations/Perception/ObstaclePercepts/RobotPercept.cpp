/**
 * @file RobotPercept.cpp
 *
 * This file implements a representation that lists the robots that were detected in
 * the current image in robot-relative field coordinates. Only robots the lower
 * end of which were visible are represented.
 *
 * @author Andre Mühlenbrock
 * @author Tim Laue
 */

#include "RobotPercept.h"
#include "Debugging/DebugDrawings.h"
#include "Framework/Blackboard.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Streaming/Global.h"
#include "Tools/Math/Transformation.h"
#include <cmath>

void RobotPercept::draw() const
{
  // Define standard colors for drawing, try to get original team color information:
  const ColorRGBA teamColors[] =
  {
    ColorRGBA(67,67,67),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.fieldPlayerColor : GameState::Team::Color::red)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.fieldPlayerColor : GameState::Team::Color::white)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).ownTeam.goalkeeperColor : GameState::Team::Color::blue)),
    ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
        static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).opponentTeam.goalkeeperColor : GameState::Team::Color::purple))
  };
  // Depth for drawing robot rectangles in field view
  const float robotDepth = Blackboard::getInstance().exists("RobotDimensions") ? static_cast<const RobotDimensions&>(Blackboard::getInstance()["RobotDimensions"]).robotDepth : 150.f;

  // Draw robots in field view:
  DEBUG_DRAWING("representation:RobotPercept:field", "drawingOnField")
  {
    for(const Robot& obstacle : robots)
    {
      const Vector2f a = obstacle.left.normalized(obstacle.left.norm() - robotDepth);
      const Vector2f b = obstacle.right.normalized(obstacle.right.norm() - robotDepth);
      const Vector2f c  = obstacle.right.normalized(obstacle.right.norm() + robotDepth);
      const Vector2f d  = obstacle.left.normalized(obstacle.left.norm() + robotDepth);
      QUADRANGLE("representation:RobotPercept:field", a.x(), a.y(), b.x(), b.y(), c.x(), c.y(), d.x(), d.y(),
                 40, Drawings::solidPen, teamColors[obstacle.type]);
    }
  }

  // Drawing in image:
  DEBUG_DRAWING("representation:RobotPercept:image", "drawingOnImage")
  {
    if(Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("CameraMatrix"))
    {
      const CameraInfo& cameraInfo = static_cast<CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      const CameraMatrix& cameraMatrix = static_cast<CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);

      const Settings::RobotType robotType = Global::getSettings().robotType;
      const float assumedRobotHeight = robotType == Settings::k1 ? 950.f : 1180.f;

      for(const Robot& obstacle : robots)
      {
        Vector2f pointsInImage[4];
        if(Transformation::robotToImage(obstacle.left, cameraMatrix, cameraInfo, pointsInImage[0])
           && Transformation::robotToImage(obstacle.right, cameraMatrix, cameraInfo, pointsInImage[1])
           && Transformation::robotToImage(Vector3f(obstacle.right.x(), obstacle.right.y(), assumedRobotHeight), cameraMatrix, cameraInfo, pointsInImage[2])
           && Transformation::robotToImage(Vector3f(obstacle.left.x(), obstacle.left.y(), assumedRobotHeight), cameraMatrix, cameraInfo, pointsInImage[3]))
        {
          if(obstacle.type == unknown)
          {
            QUADRANGLE("representation:RobotPercept:image",pointsInImage[0].x(), pointsInImage[0].y(), pointsInImage[1].x(), pointsInImage[1].y(),pointsInImage[2].x(), pointsInImage[2].y(), pointsInImage[3].x(), pointsInImage[3].y(),
                      6, Drawings::dashedPen, ColorRGBA::black);
            QUADRANGLE("representation:RobotPercept:image",pointsInImage[0].x(), pointsInImage[0].y(), pointsInImage[1].x(), pointsInImage[1].y(),pointsInImage[2].x(), pointsInImage[2].y(), pointsInImage[3].x(), pointsInImage[3].y(),
                      6, Drawings::dottedPen, ColorRGBA::gray);
          }
          else
          {
            QUADRANGLE("representation:RobotPercept:image",pointsInImage[0].x(), pointsInImage[0].y(), pointsInImage[1].x(), pointsInImage[1].y(),pointsInImage[2].x(), pointsInImage[2].y(), pointsInImage[3].x(), pointsInImage[3].y(),
                      6, Drawings::solidPen, teamColors[obstacle.type]);
          }
        }
        Vector2f centerInImage;
        if(Transformation::robotToImage(obstacle.center, cameraMatrix, cameraInfo, centerInImage))
        {
          CIRCLE("representation:RobotPercept:image", centerInImage.x(), centerInImage.y(), 4, 1, Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
        }
      }
    }
  }
}

void RobotPercept::verify() const
{
  for([[maybe_unused]] const Robot& obstacle : robots)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));
    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));
    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));
  }
}
