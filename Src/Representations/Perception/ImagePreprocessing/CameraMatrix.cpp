/**
 * @file CameraMatrix.cpp
 *
 * Implementation of struct CameraMatrix.
 */

#include "CameraMatrix.h"
#include "Tools/Boundary.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Module/Blackboard.h"

CameraMatrix::CameraMatrix(const Pose3f& pose) :
  Pose3f(pose), invPos(Pose3f::inverse()), isValid(true)
{}

void CameraMatrix::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:CameraMatrix:image", "drawingOnImage"); // Shows the robot coordinate system
  DECLARE_DEBUG_DRAWING("representation:CameraMatrix:field", "drawingOnField"); // Shows the robot coordinate system

  COMPLEX_DRAWING("representation:CameraMatrix:field")
  {
    CameraInfo cameraInfo;
    if(Blackboard::getInstance().exists("CameraInfo"))
      cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    Vector2i pointOnField[6];
    bool isValid[6];
    // calculate the projection of the four image corners to the ground
    isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
    isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
    isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
    isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

    // calculate a line 15 pixels below the horizon in the image
    const Geometry::Line horizon = Projection::calculateHorizon(*this, cameraInfo);
    Geometry::Line lineBelowHorizon;
    const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
    lineBelowHorizon.direction = horizon.direction;
    lineBelowHorizon.base = horizon.base;
    lineBelowHorizon.base += vertLineDirection * 15.0f;

    // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
    Vector2i beginPoint;
    Vector2i endPoint;
    if(Geometry::getIntersectionPointsOfLineAndRectangle(Vector2i::Zero(), Vector2i(cameraInfo.width - 1, cameraInfo.height - 1), lineBelowHorizon, beginPoint, endPoint))
    {
      isValid[4] = Transformation::imageToRobot(beginPoint.x(), beginPoint.y(), *this, cameraInfo, pointOnField[4]);
      isValid[5] = Transformation::imageToRobot(endPoint.x(), endPoint.y(), *this, cameraInfo, pointOnField[5]);
      if(isValid[4] && isValid[5])
        LINE("representation:CameraMatrix:field", pointOnField[4].x(), pointOnField[4].y(), pointOnField[5].x(), pointOnField[5].y(), 30, Drawings::solidPen, ColorRGBA::yellow);
    }
    else
      isValid[4] = isValid[5] = false;

    // determine the boundary of all the points that were projected to the ground
    Boundaryi boundary(-10000, +10000);
    for(int i = 0; i < 6; ++i)
      if(isValid[i])
      {
        boundary.add(pointOnField[i]);
        const ColorRGBA& color = i < 4 ? ColorRGBA::white : ColorRGBA::yellow;
        CIRCLE("representation:CameraMatrix:field", pointOnField[i].x(), pointOnField[i].y(), 100, 50, Drawings::solidPen, color, Drawings::noBrush, color);
      }

    LINE("representation:CameraMatrix:field", boundary.x.min, boundary.y.min, boundary.x.max, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::red);
    LINE("representation:CameraMatrix:field", boundary.x.max, boundary.y.min, boundary.x.max, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::yellow);
    LINE("representation:CameraMatrix:field", boundary.x.max, boundary.y.max, boundary.x.min, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:CameraMatrix:field", boundary.x.min, boundary.y.max, boundary.x.min, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::white);

    // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
    const int spacing = 100;
    for(int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
    {
      LINE("representation:CameraMatrix:field", xx, boundary.y.min, xx, boundary.y.max, 5, Drawings::solidPen, ColorRGBA::white);
    }
    for(int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
    {
      LINE("representation:CameraMatrix:field", boundary.x.min, yy, boundary.x.max, yy, 5, Drawings::solidPen, ColorRGBA::white);
    }
  } // end complex drawing

  COMPLEX_DRAWING("representation:CameraMatrix:image")
  {
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      Vector2i pointOnField[6];
      bool isValid[6];
      // calculate the projection of the four image corners to the ground
      isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
      isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
      isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
      isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

      // calculate a line 15 pixels below the horizon in the image
      const Geometry::Line horizon = Projection::calculateHorizon(*this, cameraInfo);
      Geometry::Line lineBelowHorizon;
      const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
      lineBelowHorizon.direction = horizon.direction;
      lineBelowHorizon.base = horizon.base;
      lineBelowHorizon.base += vertLineDirection * 15.0f;

      // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
      Vector2f beginPoint = Vector2f::Zero();
      Vector2f endPoint = Vector2f::Zero();
      const Vector2f topRight = Vector2f(cameraInfo.width - 1.f, cameraInfo.height - 1.f);
      if(Geometry::getIntersectionPointsOfLineAndRectangle(Vector2f::Zero(), topRight, lineBelowHorizon, beginPoint, endPoint))
      {
        LINE("representation:CameraMatrix:image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), 3, Drawings::dashedPen, ColorRGBA::white);
        Vector2f pointOnFieldFloat = Vector2f::Zero();
        isValid[4] = Transformation::imageToRobot(beginPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[4].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[4].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
        isValid[5] = Transformation::imageToRobot(endPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[5].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[5].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
      }
      else
        isValid[4] = isValid[5] = false;

      // determine the boundary of all the points that were projected to the ground
      Boundaryi boundary(-10000, +10000);
      for(int i = 0; i < 6; ++i)
        if(isValid[i])
          boundary.add(pointOnField[i]);

      // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
      int spacing = 100;
      for(int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
        if(Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.min)), *this, cameraInfo, beginPoint) &&
           Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.max)), *this, cameraInfo, endPoint))
          LINE("representation:CameraMatrix:image",
               beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(),
               xx == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::white);
      for(int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
        if(Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.min), static_cast<float>(yy)), *this, cameraInfo, beginPoint) &&
           Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.max), static_cast<float>(yy)), *this, cameraInfo, endPoint))
          LINE("representation:CameraMatrix:image",
               beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(),
               yy == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::white);
    }
  } // end complex drawing

  if(Blackboard::getInstance().exists("CameraInfo"))
  {
    const CameraInfo cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);

    if(cameraInfo.camera == CameraInfo::upper)
      PLOT_VEC3("representation:CameraMatrix:upper:", this->translation);
    else
      PLOT_VEC3("representation:CameraMatrix:lower:", this->translation);
  }
}

void RobotCameraMatrix::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:RobotCameraMatrix:image", "drawingOnImage"); // Shows the robot coordinate system
  DECLARE_DEBUG_DRAWING("representation:RobotCameraMatrix:field", "drawingOnField"); // Shows the robot coordinate system

  COMPLEX_DRAWING("representation:RobotCameraMatrix:field")
  {
    CameraInfo cameraInfo;
    if(Blackboard::getInstance().exists("CameraInfo"))
      cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
    Vector2i pointOnField[6];
    bool isValid[6];
    // calculate the projection of the four image corners to the ground
    isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
    isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
    isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
    isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

    // calculate a line 15 pixels below the horizon in the image
    const Geometry::Line horizon = Projection::calculateHorizon(*this, cameraInfo);
    Geometry::Line lineBelowHorizon;
    const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
    lineBelowHorizon.direction = horizon.direction;
    lineBelowHorizon.base = horizon.base;
    lineBelowHorizon.base += vertLineDirection * 15.f;

    // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
    Vector2i beginPoint;
    Vector2i endPoint;
    if(Geometry::getIntersectionPointsOfLineAndRectangle(Vector2i::Zero(), Vector2i(cameraInfo.width - 1, cameraInfo.height - 1), lineBelowHorizon, beginPoint, endPoint))
    {
      isValid[4] = Transformation::imageToRobot(beginPoint.x(), beginPoint.y(), *this, cameraInfo, pointOnField[4]);
      isValid[5] = Transformation::imageToRobot(endPoint.x(), endPoint.y(), *this, cameraInfo, pointOnField[5]);
      if(isValid[4] && isValid[5])
        LINE("representation:CameraMatrix:field", pointOnField[4].x(), pointOnField[4].y(), pointOnField[5].x(), pointOnField[5].y(), 30, Drawings::solidPen, ColorRGBA::yellow);
    }

    // determine the boundary of all the points that were projected to the ground
    Boundaryi boundary(-10000, +10000);
    for(int i = 0; i < 6; ++i)
      if(isValid[i])
      {
        boundary.add(pointOnField[i]);
        const ColorRGBA& color = i < 4 ? ColorRGBA::white : ColorRGBA::yellow;
        CIRCLE("representation:RobotCameraMatrix:field", pointOnField[i].x(), pointOnField[i].y(), 100, 50, Drawings::solidPen, color, Drawings::noBrush, color);
      }

    LINE("representation:RobotCameraMatrix:field", boundary.x.min, boundary.y.min, boundary.x.max, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::red);
    LINE("representation:RobotCameraMatrix:field", boundary.x.max, boundary.y.min, boundary.x.max, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::yellow);
    LINE("representation:RobotCameraMatrix:field", boundary.x.max, boundary.y.max, boundary.x.min, boundary.y.max, 30, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:RobotCameraMatrix:field", boundary.x.min, boundary.y.max, boundary.x.min, boundary.y.min, 30, Drawings::solidPen, ColorRGBA::white);

    // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
    const int spacing = 100;
    for(int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
    {
      LINE("representation:RobotCameraMatrix:field", xx, boundary.y.min, xx, boundary.y.max, 5, Drawings::solidPen, ColorRGBA::white);
    }
    for(int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
    {
      LINE("representation:RobotCameraMatrix:field", boundary.x.min, yy, boundary.x.max, yy, 5, Drawings::solidPen, ColorRGBA::white);
    }
  } // end complex drawing

  COMPLEX_DRAWING("representation:RobotCameraMatrix:image")
  {
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      const CameraInfo cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      Vector2i pointOnField[6];
      bool isValid[6];
      // calculate the projection of the four image corners to the ground
      isValid[0] = Transformation::imageToRobot(0, 0, *this, cameraInfo, pointOnField[0]);
      isValid[1] = Transformation::imageToRobot(cameraInfo.width, 0, *this, cameraInfo, pointOnField[1]);
      isValid[2] = Transformation::imageToRobot(cameraInfo.width, cameraInfo.height, *this, cameraInfo, pointOnField[2]);
      isValid[3] = Transformation::imageToRobot(0, cameraInfo.height, *this, cameraInfo, pointOnField[3]);

      // calculate a line 15 pixels below the horizon in the image
      const Geometry::Line horizon = Projection::calculateHorizon(*this, cameraInfo);
      Geometry::Line lineBelowHorizon;
      const Vector2f vertLineDirection(-horizon.direction.y(), horizon.direction.x());
      lineBelowHorizon.direction = horizon.direction;
      lineBelowHorizon.base = horizon.base;
      lineBelowHorizon.base += vertLineDirection * 15.f;

      // calculate the projection to the ground of the intersection points of the line parallel to the horizon and the image borders
      Vector2f beginPoint;
      Vector2f endPoint;
      if(Geometry::getIntersectionPointsOfLineAndRectangle(Vector2f::Zero(),
          Vector2f(static_cast<float>(cameraInfo.width - 1), static_cast<float>(cameraInfo.height - 1)),
          lineBelowHorizon, beginPoint, endPoint))
      {
        LINE("representation:RobotCameraMatrix:image", beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(), 3, Drawings::dashedPen, ColorRGBA::white);
        Vector2f pointOnFieldFloat;
        isValid[4] = Transformation::imageToRobot(beginPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[4].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[4].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
        isValid[5] = Transformation::imageToRobot(endPoint, *this, cameraInfo, pointOnFieldFloat);
        pointOnField[5].x() = static_cast<int>(std::floor(pointOnFieldFloat.x() + 0.5));
        pointOnField[5].y() = static_cast<int>(std::floor(pointOnFieldFloat.y() + 0.5));
      }
      else
        isValid[4] = isValid[5] = false;

      // determine the boundary of all the points that were projected to the ground
      Boundaryi boundary(-10000, +10000);
      for(int i = 0; i < 6; ++i)
        if(isValid[i])
          boundary.add(pointOnField[i]);

      // fill the bounding rectangle with coordinate system lines (and reproject it to the image)
      const int spacing = 100;
      for(int xx = boundary.x.min - boundary.x.min % spacing + spacing; xx <= boundary.x.max; xx += spacing)
        if(Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.min)), *this, CameraInfo(), beginPoint) &&
           Transformation::robotToImage(Vector2f(static_cast<float>(xx), static_cast<float>(boundary.y.max)), *this, CameraInfo(), endPoint))
          LINE("representation:RobotCameraMatrix:image",
               beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(),
               xx == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::yellow);
      for(int yy = boundary.y.min - boundary.y.min % spacing + spacing; yy <= boundary.y.max; yy += spacing)
        if(Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.min), static_cast<float>(yy)), *this, CameraInfo(), beginPoint) &&
           Transformation::robotToImage(Vector2f(static_cast<float>(boundary.x.max), static_cast<float>(yy)), *this, CameraInfo(), endPoint))
          LINE("representation:RobotCameraMatrix:image",
               beginPoint.x(), beginPoint.y(), endPoint.x(), endPoint.y(),
               yy == 0 ? 3 : 0, Drawings::solidPen, ColorRGBA::yellow);
    }
  } // end complex drawing
}

RobotCameraMatrix::RobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, CameraInfo::Camera camera)
{
  computeRobotCameraMatrix(robotDimensions, headYaw, headPitch, cameraCalibration, camera);
}

void RobotCameraMatrix::computeRobotCameraMatrix(const RobotDimensions& robotDimensions, float headYaw, float headPitch, const CameraCalibration& cameraCalibration, CameraInfo::Camera camera)
{
  *this = RobotCameraMatrix();

  translate(0., 0., robotDimensions.hipToNeckLength);
  rotateZ(headYaw);
  rotateY(headPitch);
  if(camera == CameraInfo::upper)
  {
    translate(robotDimensions.xOffsetNeckToUpperCamera, 0.f, robotDimensions.zOffsetNeckToUpperCamera);
    rotateY(robotDimensions.tiltNeckToUpperCamera + cameraCalibration.cameraRotationCorrections[camera].y());
  }
  else
  {
    translate(robotDimensions.xOffsetNeckToLowerCamera, 0.f, robotDimensions.zOffsetNeckToLowerCamera);
    rotateY(robotDimensions.tiltNeckToLowerCamera + cameraCalibration.cameraRotationCorrections[camera].y());
  }
  rotateX(cameraCalibration.cameraRotationCorrections[camera].x());
  rotateZ(cameraCalibration.cameraRotationCorrections[camera].z());
}

CameraMatrix::CameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  computeCameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
}

void CameraMatrix::computeCameraMatrix(const Pose3f& torsoMatrix, const Pose3f& robotCameraMatrix, const CameraCalibration& cameraCalibration)
{
  static_cast<Pose3f&>(*this) = torsoMatrix;
  rotateY(cameraCalibration.bodyRotationCorrection.y());
  rotateX(cameraCalibration.bodyRotationCorrection.x());
  conc(robotCameraMatrix);
  onRead();
}

void CameraMatrix::onRead()
{
  invPos = Pose3f::inverse();
}
