/**
 * @file Representations/Modeling/TeamPlayersModel.cpp
 * Implementation of a debug drawing of the combined world model
 * @author Katharina Gillmann
 */

#include "TeamPlayersModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Modeling/Obstacle.h"
#include "BallModel.h"
#include "RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"

void TeamPlayersModel::verify() const
{
#ifndef NDEBUG
  for(const auto& obstacle : obstacles)
  {
    ASSERT(std::isfinite(obstacle.center.x()));
    ASSERT(std::isfinite(obstacle.center.y()));

    ASSERT(std::isfinite(obstacle.left.x()));
    ASSERT(std::isfinite(obstacle.left.y()));

    ASSERT(std::isfinite(obstacle.right.x()));
    ASSERT(std::isfinite(obstacle.right.y()));

    ASSERT(std::isfinite(obstacle.velocity.x()));
    ASSERT(std::isfinite(obstacle.velocity.y()));

    ASSERT((obstacle.left - obstacle.right).squaredNorm() < sqr(2000.f));

    ASSERT(std::isnormal(obstacle.covariance(0, 0)));
    ASSERT(std::isnormal(obstacle.covariance(1, 1)));
    ASSERT(std::isfinite(obstacle.covariance(0, 1)));
    ASSERT(std::isfinite(obstacle.covariance(1, 0)));
    ASSERT(Approx::isEqual(obstacle.covariance(0, 1), obstacle.covariance(1, 0), 1e-20f));
  }
#endif
}

void TeamPlayersModel::draw() const
{
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

  DEBUG_DRAWING("representation:TeamPlayersModel:ownRobots", "drawingOnField")
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

  DEBUG_DRAWING("representation:TeamPlayersModel:teamCoverage", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("BallModel") && Blackboard::getInstance().exists("FieldDimensions")
       && Blackboard::getInstance().exists("RobotPose"))
    {
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);

      const Vector2f gloBallPos = theRobotPose * theBallModel.estimate.position;
      for(const auto& obstacle : obstacles)
        if(obstacle.type == Obstacle::teammate || obstacle.type == Obstacle::fallenTeammate)
        {
          const Vector2f offset = Vector2f(0.f, 1.f).rotate((gloBallPos - obstacle.center).angle());

          const float standRange = 80.f;
          const float genuflectRange = 200.f;

          auto drawOffset = [&](const float range, const ColorRGBA color)
          {
            const Vector2f rangeOffset = offset.normalized(range);
            const Vector2f left = obstacle.center + rangeOffset;
            const Vector2f right = obstacle.center - rangeOffset;

            const Geometry::Line leftLine(gloBallPos, left - gloBallPos);
            const Geometry::Line rightLine(gloBallPos, right - gloBallPos);

            const Vector2f bottomLeft(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);
            const Vector2f topRight(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline);

            Vector2f useLeft;
            Vector2f useRight;

            Vector2f intersection1;
            Vector2f intersection2;
            if(!Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, leftLine, intersection1, intersection2))
              return;
            useLeft = (intersection1 - obstacle.center).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

            if(!Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, rightLine, intersection1, intersection2))
              return;
            useRight = (intersection1 - obstacle.center).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

            const Vector2f points[3] = { gloBallPos, useRight, useLeft };
            POLYGON("representation:TeamPlayersModel:teamCoverage", 3, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
          };

          //const Vector2f points[3] = { gloBallPos , Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal) , Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal) };
          //POLYGON("representation:TeamPlayersModel:teamCoverage", 3, points, 10, Drawings::noPen, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50), Drawings::solidBrush, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50));

          drawOffset(genuflectRange, ColorRGBA(ColorRGBA::violet.r, ColorRGBA::violet.g, ColorRGBA::violet.b, 100));
          drawOffset(standRange, ColorRGBA(ColorRGBA::green.r, ColorRGBA::green.g, ColorRGBA::green.b, 200));
        }
    }
  }

  DECLARE_DEBUG_DRAWING("representation:TeamPlayersModel:oppRobotsCovariance", "drawingOnField");
  DEBUG_DRAWING("representation:TeamPlayersModel:oppRobots", "drawingOnField")
  {
    for(const auto& obstacle : obstacles)
    {
      if(obstacle.type != Obstacle::teammate && obstacle.type != Obstacle::fallenTeammate)
      {
        CROSS("representation:TeamPlayersModel:oppRobots", obstacle.center.x(), obstacle.center.y(), 20, 40, Drawings::solidPen, ColorRGBA::blue);
        CIRCLE("representation:TeamPlayersModel:oppRobots", obstacle.center.x(), obstacle.center.y(), 600, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::noBrush, ColorRGBA());
        COVARIANCE_ELLIPSES_2D("representation:TeamPlayersModel:oppRobotsCovariance", obstacle.covariance, obstacle.center);
      }
    }
  }
}
