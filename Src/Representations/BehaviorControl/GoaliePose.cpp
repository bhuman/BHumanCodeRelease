#include "GoaliePose.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Geometry.h"

void GoaliePose::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:GoaliPose:coverage", "drawingOnField");
  COMPLEX_DRAWING("representation:GoaliPose:coverage")
  {
    if(Blackboard::getInstance().exists("BallModel") && Blackboard::getInstance().exists("RobotPose") && Blackboard::getInstance().exists("FieldDimensions"))
    {
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
      const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      const Vector2f gloBallPos = theRobotPose * theBallModel.estimate.position;

      const Vector2f offset = Vector2f(0.f, 1.f).rotate(this->goaliePoseField.rotation);

      const float standRange = 80.f;
      const float genuflectRange = 200.f;
      const float jumpRange = 600.f;

      auto drawOffset = [&](const float range, const ColorRGBA color)
      {
        const Vector2f rangeOffset = offset.normalized(range);
        const Vector2f keeperLeft = this->goaliePoseField.translation + rangeOffset;
        const Vector2f keeperRight = this->goaliePoseField.translation - rangeOffset;

        const Geometry::Line leftLine(gloBallPos, keeperLeft - gloBallPos);
        const Geometry::Line rightLine(gloBallPos, keeperRight - gloBallPos);

        const Geometry::Line groundLine(Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f), Vector2f(0.f,1.f));

        Vector2f useKeeperLeft;
        Vector2f useKeeperRight;

        Vector2f intersections;
        if(!Geometry::getIntersectionOfLines(groundLine, leftLine, intersections) || std::abs(intersections.y()) > theFieldDimensions.yPosLeftGoal)
          useKeeperLeft = isNearLeftPost ? goaliePoseField.translation : Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal);
        else
          useKeeperLeft = intersections;

        if(!Geometry::getIntersectionOfLines(groundLine, rightLine, intersections) || std::abs(intersections.y()) > theFieldDimensions.yPosLeftGoal)
          useKeeperRight = isNearRightPost ? goaliePoseField.translation : Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal);
        else
          useKeeperRight = intersections;

        const Vector2f points[3] = { gloBallPos , useKeeperRight , useKeeperLeft };
        POLYGON("representation:GoaliPose:coverage", 3, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
      };

      const Vector2f points[3] = { gloBallPos , Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal) , Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal) };
      POLYGON("representation:GoaliPose:coverage", 3, points, 10, Drawings::noPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::red);

      drawOffset(jumpRange, ColorRGBA::orange);
      drawOffset(genuflectRange, ColorRGBA::violet);
      drawOffset(standRange, ColorRGBA::green);
      LINE("representation:GoaliPose:coverage", this->goaliePoseField.translation.x(), this->goaliePoseField.translation.y(), gloBallPos.x(), gloBallPos.y(), 20.f, Drawings::dashedPen, ColorRGBA::orange);
    }
  }
}

void GoaliePose::verify() const
{
  ASSERT(goaliePoseField.isFinite());
  ASSERT(goaliePoseField.translation.x() > -5000.f);
  ASSERT(goaliePoseField.translation.x() < -1000.f);
  ASSERT(goaliePoseField.translation.y() > -3000.f);
  ASSERT(goaliePoseField.translation.y() < 3000.f);
  ASSERT(goaliePoseField.rotation >= -pi);
  ASSERT(goaliePoseField.rotation <= pi);

  ASSERT(goaliePoseRel.isFinite());
  ASSERT(goaliePoseRel.rotation >= -pi);
  ASSERT(goaliePoseRel.rotation <= pi);
};
