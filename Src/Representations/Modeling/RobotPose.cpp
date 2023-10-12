/**
 * @file RobotPose.cpp
 *
 * contains the implementation of the streaming operators
 * for the struct RobotPose
 */

#include "RobotPose.h"
#include "BallModel.h"
#include "Platform/Time.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Tools/Math/Projection.h"
#include "Framework/Blackboard.h"

void RobotPose::operator>>(BHumanMessage& m) const
{
  RobotPoseCompact compact(*this);
  Streaming::streamIt(*m.out, "theRobotPose", compact);
}

void RobotPose::operator<<(const BHumanMessage& m)
{
  RobotPoseCompact compact(*this);
  Streaming::streamIt(*m.in, "theRobotPose", compact);
  // inversePose is set by RobotPoseCompact::onRead.
}

void RobotPose::onRead()
{
  inversePose = *this;
  inversePose.invert();
}

void RobotPose::verify() const
{
  ASSERT(std::isfinite(translation.x()));
  ASSERT(std::isfinite(translation.y()));
  ASSERT(std::isfinite(rotation));
  ASSERT(rotation >= -pi);
  ASSERT(rotation <= pi);

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
  const ColorRGBA ownTeamColorForDrawing = ColorRGBA::fromTeamColor(static_cast<int>(Blackboard::getInstance().exists("GameState") ?
                                                                                     static_cast<const GameState&>(Blackboard::getInstance()["GameState"]).color() :
                                                                                     GameState::Team::Color::black));

  DEBUG_DRAWING("representation:RobotPose", "drawingOnField")
  {
    ROBOT("representation:RobotPose", static_cast<Pose2f>(*this), dirVec, dirVec, 1.f, ColorRGBA::black, ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
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
    const Matrix2f cov = covariance.topLeftCorner<2, 2>();
    COVARIANCE_ELLIPSES_2D("representation:RobotPose:covariance", cov, translation);
  }

  DEBUG_DRAWING("representation:RobotPose:fieldOfView", "drawingOnField")
  {
    if(Blackboard::getInstance().exists("CameraMatrix") && Blackboard::getInstance().exists("CameraInfo") && Blackboard::getInstance().exists("FieldDimensions"))
    {
      const CameraMatrix& theCameraMatrix = static_cast<const CameraMatrix&>(Blackboard::getInstance()["CameraMatrix"]);
      if(theCameraMatrix.isValid)
      {
        const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
        const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
        const RobotPose& theRobotPose = *this;
        std::vector<Vector2f> p;
        Projection::computeFieldOfViewInFieldCoordinates(theRobotPose, theCameraMatrix, theCameraInfo, theFieldDimensions, p);
        THREAD("representation:RobotPose:fieldOfView", theCameraInfo.getThreadName());
        POLYGON("representation:RobotPose:fieldOfView", 4, p, 20, Drawings::noPen, ColorRGBA(), Drawings::solidBrush, ColorRGBA(255, 255, 255, 25));
      }
    }
  }

  DEBUG_DRAWING("perception:RobotPose", "drawingOnField") // Set the origin to the robot's current position
  {
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo& theCameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      THREAD("perception:RobotPose", theCameraInfo.getThreadName());
    }
    ORIGIN("perception:RobotPose", translation.x(), translation.y(), rotation);
  }

  DEBUG_DRAWING("cognition:RobotPose", "drawingOnField") // Set the origin to the robot's current position
  {
    ORIGIN("cognition:RobotPose", translation.x(), translation.y(), rotation);
  }

  DEBUG_DRAWING("cognition:Reset", "drawingOnField") // Set the origin to the fields origin
  {
    ORIGIN("cognition:Reset", 0, 0, 0);
  }

  DEBUG_DRAWING("motion:RobotPose", "drawingOnField")  // Set the origin to the robot's current position
  {
    THREAD("motion:RobotPose", "Motion");
    ORIGIN("motion:RobotPose", translation.x(), translation.y(), rotation);
  }

  DEBUG_DRAWING("motion:Reset", "drawingOnField") // Set the origin to the fields origin
  {
    THREAD("motion:RobotPose", "Motion");
    ORIGIN("motion:Reset", 0, 0, 0);
  }

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

      auto drawOffset = [&](const float range, [[maybe_unused]] const ColorRGBA color)
      {
        const Vector2f rangeOffset = offset.normalized(range);
        const Vector2f left = this->translation + rangeOffset;
        const Vector2f right = this->translation - rangeOffset;

        const Geometry::Line leftLine(gloBallPos, left - gloBallPos);
        const Geometry::Line rightLine(gloBallPos, right - gloBallPos);

        const Vector2f bottomLeft(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
        const Vector2f topRight(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);

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

        const Vector2f points[3] = {gloBallPos, useRight, useLeft};
        POLYGON("representation:RobotPose:coverage", 3, points, 10, Drawings::noPen, color, Drawings::solidBrush, color);
      };

      const Vector2f points[3] =
      {
        gloBallPos,
        Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal),
        Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal)
      };
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
    Vector2f dirVec(200, 0);
    dirVec = *this * dirVec;
    const ColorRGBA ownTeamColorForDrawing(0, 0, 0);
    ROBOT("representation:GroundTruthRobotPose", static_cast<Pose2f>(*this), dirVec, dirVec, .5f,
          ownTeamColorForDrawing, ColorRGBA(255, 255, 255, 128), ColorRGBA(0, 0, 0, 0));
  }

  DEBUG_DRAWING("cognition:GroundTruthRobotPose", "drawingOnField") // Set the origin to the robot's ground truth position
  {
    ORIGIN("cognition:GroundTruthRobotPose", translation.x(), translation.y(), rotation);
  }
}

RobotPoseCompact::RobotPoseCompact(const RobotPose& robotPose)
  : robotPose(const_cast<RobotPose&>(robotPose)),
    rotation(robotPose.rotation),
    translation(robotPose.translation),
    quality(robotPose.quality),
    timestampLastJump(robotPose.timestampLastJump)
{
  covariance(0, 0) = float2half(robotPose.covariance(0, 0));
  covariance(0, 1) = float2half(robotPose.covariance(0, 1));
  covariance(1, 0) = float2half(robotPose.covariance(1, 0));
  covariance(1, 1) = float2half(robotPose.covariance(1, 1));
}

void RobotPoseCompact::onRead()
{
  robotPose.rotation = rotation;
  robotPose.translation = translation;
  robotPose.quality = quality;
  robotPose.covariance(0, 0) = half2float(covariance(0, 0));
  robotPose.covariance(0, 1) = half2float(covariance(0, 1));
  robotPose.covariance(1, 0) = half2float(covariance(1, 0));
  robotPose.covariance(1, 1) = half2float(covariance(1, 1));
  robotPose.timestampLastJump = timestampLastJump;
  robotPose.onRead();
}

float RobotPoseCompact::half2float(short halfValue)
{
  float floatValue = 0.f;
  reinterpret_cast<short*>(&floatValue)[1] = halfValue;
  return floatValue;
}

short RobotPoseCompact::float2half(float floatValue)
{
  if(floatValue == 0.f)
    floatValue = 0.0000001f;
  return reinterpret_cast<short*>(&floatValue)[1];
}
