/**
 * @file RobotPose.cpp
 *
 * contains the implementation of the streaming operators
 * for the struct RobotPose
 */

#include "RobotPose.h"
#include "BallModel.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/Covariance.h"

void RobotPose::onRead()
{
  inversePose = *this;
  inversePose.invert();
}

Pose2f RobotPose::inverse() const
{
  ASSERT(false);//use theRobotPose.inversePose
  return inversePose;//bad copy stuff
}

void RobotPose::verify() const
{
  ASSERT(std::isfinite(translation.x()));
  ASSERT(std::isfinite(translation.y()));
  ASSERT(std::isfinite(rotation));
  ASSERT(rotation >= -pi);
  ASSERT(rotation <= pi);

  ASSERT(validity >= 0.f);
  ASSERT(validity <= 1.f);

  ASSERT(std::isfinite(deviation));

  ASSERT(std::isnormal(covariance(0, 0)));
  ASSERT(std::isnormal(covariance(1, 1)));
  ASSERT(std::isnormal(covariance(2, 2)));
  ASSERT(std::isfinite(covariance(0, 1)));
  ASSERT(std::isfinite(covariance(0, 2)));
  ASSERT(std::isfinite(covariance(1, 0)));
  ASSERT(std::isfinite(covariance(1, 2)));
  ASSERT(std::isfinite(covariance(2, 0)));
  ASSERT(std::isfinite(covariance(2, 1)));

  Pose2f inv = *this;
  inv.invert();
  ASSERT(inv.rotation == inversePose.rotation);
  ASSERT(inv.translation.x() == inversePose.translation.x());
  ASSERT(inv.translation.y() == inversePose.translation.y());
}

void RobotPose::draw() const
{
  Vector2f bodyPoints[4] =
  {
    Vector2f(55, 90),
    Vector2f(-55, 90),
    Vector2f(-55, -90),
    Vector2f(55, -90)
  };
  for(int i = 0; i < 4; i++)
    bodyPoints[i] = *this * bodyPoints[i];
  const Vector2f dirVec = *this * Vector2f(200, 0);
  static const ColorRGBA colors[] =
  {
    ColorRGBA::blue,
    ColorRGBA::red,
    ColorRGBA::yellow,
    ColorRGBA::black
  };
  const ColorRGBA ownTeamColorForDrawing = colors[Blackboard::getInstance().exists("OwnTeamInfo") ?
      static_cast<const OwnTeamInfo&>(Blackboard::getInstance()["OwnTeamInfo"]).teamColor : TEAM_BLUE];

  DEBUG_DRAWING("representation:RobotPose", "drawingOnField")
  {
    LINE("representation:RobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
         20, Drawings::solidPen, ColorRGBA::white);
    POLYGON("representation:RobotPose", 4, bodyPoints, 20, Drawings::solidPen,
            ownTeamColorForDrawing, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("representation:RobotPose", translation.x(), translation.y(), 42, 0,
           Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);
  }

  DEBUG_DRAWING("representation:RobotPose:deviation", "drawingOnField")
  {
    if(deviation < 100000.f)
      DRAWTEXT("representation:RobotPose:deviation", -3000, -2300, 100, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: " << deviation);
    else
      DRAWTEXT("representation:RobotPose:deviation", -3000, -2300, 100, ColorRGBA(0xff, 0xff, 0xff), "pose deviation: unknown");
  }

  DEBUG_DRAWING3D("representation:RobotPose", "field")
  {
    LINE3D("representation:RobotPose", translation.x(), translation.y(), 10, dirVec.x(), dirVec.y(), 10, 1, ownTeamColorForDrawing);
    for(int i = 0; i < 4; ++i)
    {
      const Vector2f p1 = bodyPoints[i];
      const Vector2f p2 = bodyPoints[(i + 1) & 3];
      LINE3D("representation:RobotPose", p1.x(), p1.y(), 10, p2.x(), p2.y(), 10, 1, ownTeamColorForDrawing);
    }
  }

  DEBUG_DRAWING("representation:RobotPose:covariance", "drawingOnField")
  {
    float covaxis1, covaxis2, covangle;
    Covariance::errorEllipse(covariance.topLeftCorner<2, 2>(), covaxis1, covaxis2, covangle);
    ELLIPSE("representation:RobotPose:covariance", translation, covaxis1, covaxis2, covangle,
            10, Drawings::solidPen, ColorRGBA(100, 100, 255, 100), Drawings::solidBrush, ColorRGBA(100, 100, 255, 100));
  }

  DEBUG_DRAWING("representation:RobotPose:fieldOfView", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("FieldDimensions"))
    {
      const CameraMatrix& cameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      if(cameraMatrix.isValid)
      {
        const CameraInfo& cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const FieldDimensions& fieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const RobotPose& robotPose = *this;
        std::vector<Vector2f> p;
        Geometry::computeFieldOfViewInFieldCoordinates(robotPose, cameraMatrix, cameraInfo, fieldDimensions, p);
        POLYGON("representation:RobotPose:fieldOfView", 4, p, 20, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, ColorRGBA(255, 255, 255, 25));
      }
    }
  }

  DECLARE_DEBUG_DRAWING("origin:RobotPose", "drawingOnField"); // Set the origin to the robot's current position
  DECLARE_DEBUG_DRAWING("origin:RobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:RobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:RobotPoseWithoutRotation", translation.x(), translation.y(), 0);

  DEBUG_DRAWING("representation:RobotPose:coverage", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("BallModel") && Blackboard::getInstance().exists("FieldDimensions"))
    {
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);

      const Vector2f gloBallPos = (*this) * theBallModel.estimate.position;

      const Vector2f offset = Vector2f(0.f, 1.f).rotate((gloBallPos - this->translation).angle());

      const float standRange = 80.f;
      const float genuflectRange = 200.f;

      auto drawOffset = [&](const float range, const ColorRGBA color)
      {
        const Vector2f rangeOffset = offset.normalized(range);
        const Vector2f left = this->translation + rangeOffset;
        const Vector2f right = this->translation - rangeOffset;

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
        useLeft = (intersection1 - this->translation).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

        if(!Geometry::getIntersectionPointsOfLineAndRectangle(bottomLeft, topRight, rightLine, intersection1, intersection2))
          return;
        useRight = (intersection1 - this->translation).squaredNorm() < (intersection1 - gloBallPos).squaredNorm() ? intersection1 : intersection2;

        const Vector2f points[3] = {gloBallPos , useRight , useLeft};
        POLYGON("representation:RobotPose:coverage", 3, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
      };

      const Vector2f points[3] = {gloBallPos , Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal),
                                  Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal)};
      POLYGON("representation:RobotPose:coverage", 3, points, 10, Drawings::noPen, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50),
              Drawings::solidBrush, ColorRGBA(ColorRGBA::red.r, ColorRGBA::red.g, ColorRGBA::red.b, 50));

      drawOffset(genuflectRange, ColorRGBA(ColorRGBA::violet.r, ColorRGBA::violet.g, ColorRGBA::violet.b, 100));
      drawOffset(standRange, ColorRGBA(ColorRGBA::green.r, ColorRGBA::green.g, ColorRGBA::green.b, 200));
    }
  }
}

void GroundTruthRobotPose::draw() const
{
  DEBUG_DRAWING("representation:GroundTruthRobotPose", "drawingOnField")
  {
    const ColorRGBA transparentWhite(255, 255, 255, 128);
    Vector2f bodyPoints[4] =
    {
      Vector2f(55, 90),
      Vector2f(-55, 90),
      Vector2f(-55, -90),
      Vector2f(55, -90)
    };
    for(int i = 0; i < 4; i++)
      bodyPoints[i] = *this * bodyPoints[i];
    Vector2f dirVec(200, 0);
    dirVec = *this * dirVec;
    const ColorRGBA ownTeamColorForDrawing(0, 0, 0, 128);
    LINE("representation:GroundTruthRobotPose", translation.x(), translation.y(), dirVec.x(), dirVec.y(),
         20, Drawings::solidPen, transparentWhite);
    POLYGON("representation:GroundTruthRobotPose", 4, bodyPoints, 20, Drawings::solidPen,
            ownTeamColorForDrawing, Drawings::solidBrush, transparentWhite);
    CIRCLE("representation:GroundTruthRobotPose", translation.x(), translation.y(), 42, 0,
           Drawings::solidPen, ownTeamColorForDrawing, Drawings::solidBrush, ownTeamColorForDrawing);
  }

  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPose", "drawingOnField"); // Set the origin to the robot's ground truth position
  DECLARE_DEBUG_DRAWING("origin:GroundTruthRobotPoseWithoutRotation", "drawingOnField");
  ORIGIN("origin:GroundTruthRobotPose", translation.x(), translation.y(), rotation);
  ORIGIN("origin:GroundTruthRobotPoseWithoutRotation", translation.x(), translation.y(), 0);
}

RobotPoseCompressed::RobotPoseCompressed(const RobotPose& robotPose) :
  translation(robotPose.translation), rotation(robotPose.rotation), deviation(robotPose.deviation)
{
  validity = static_cast<unsigned char>(robotPose.validity * 255.f);
  covariance[0] = robotPose.covariance(0, 0);
  covariance[1] = robotPose.covariance(1, 1);
  covariance[2] = robotPose.covariance(2, 2);
  covariance[3] = (robotPose.covariance(1, 0) + robotPose.covariance(0, 1)) / 2.f;
  covariance[4] = (robotPose.covariance(2, 0) + robotPose.covariance(0, 2)) / 2.f;
  covariance[5] = (robotPose.covariance(2, 1) + robotPose.covariance(1, 2)) / 2.f;
}

RobotPoseCompressed::operator RobotPose() const
{
  RobotPose robotPose;
  robotPose.translation = translation;
  robotPose.rotation = rotation;
  robotPose.validity = validity / 255.f;
  robotPose.deviation = deviation;
  robotPose.covariance(0, 0) = covariance[0];
  robotPose.covariance(1, 1) = covariance[1];
  robotPose.covariance(2, 2) = covariance[2];
  robotPose.covariance(1, 0) = robotPose.covariance(0, 1) = covariance[3];
  robotPose.covariance(2, 0) = robotPose.covariance(0, 2) = covariance[4];
  robotPose.covariance(2, 1) = robotPose.covariance(1, 2) = covariance[5];

  return robotPose;
}
