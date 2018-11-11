/**
 * @file Tools/Math/Projection.cpp
 *
 * A collection of functions that are about projections and perspective and stuff.
 */

#include "Projection.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/BHMath.h"

#include <cmath>


void Projection::computeFieldOfViewInFieldCoordinates(const RobotPose& robotPose, const CameraMatrix& cameraMatrix,
                                                      const CameraInfo& cameraInfo, const FieldDimensions& fieldDimensions,
                                                      std::vector<Vector2f>& p)
{
  if(p.size() < 4)
    p.resize(4);
  const Vector3f vectorToCenter(1, 0, 0);

  RotationMatrix r(cameraMatrix.rotation);
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  Vector3f vectorToCenterWorld = r * vectorToCenter;

  const float a1 = cameraMatrix.translation.x();
  const float a2 = cameraMatrix.translation.y();
  const float a3 = cameraMatrix.translation.z();
  float b1 = vectorToCenterWorld.x(),
  b2 = vectorToCenterWorld.y(),
  b3 = vectorToCenterWorld.z(),
  f = a3 / b3;
  Vector2f pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[0] = robotPose.translation;
  else
    p[0] = (robotPose + pof).translation;

  r = cameraMatrix.rotation;
  r.rotateY(cameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[1] = robotPose.translation;
  else
    p[1] = (robotPose + pof).translation;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(cameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  const float maxDist = std::sqrt(4.f * fieldDimensions.xPosOpponentFieldBorder * fieldDimensions.xPosOpponentFieldBorder +
                                  4.f * fieldDimensions.yPosLeftFieldBorder * fieldDimensions.yPosLeftFieldBorder);
  if(f > 0.f)
    p[2] = robotPose.translation + Vector2f(maxDist, 0).rotate(robotPose.rotation + (-cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    p[2] = (robotPose + pof).translation;

  r = cameraMatrix.rotation;
  r.rotateY(-(cameraInfo.openingAngleHeight / 2));
  r.rotateZ(cameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x();
  b2 = vectorToCenterWorld.y();
  b3 = vectorToCenterWorld.z();
  f = a3 / b3;
  pof = Vector2f(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[3] = robotPose.translation + Vector2f(maxDist, 0).rotate(robotPose.rotation + (cameraInfo.openingAngleWidth / 2) + cameraMatrix.rotation.getZAngle());
  else
    p[3] = (robotPose + pof).translation;
}

Geometry::Line Projection::calculateHorizon(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo)
{
  const RowVector3f bottomRow = cameraMatrix.rotation.bottomRows(1);
  const float r31 = bottomRow.x();
  const float r32 = bottomRow.y();
  const float r33 = bottomRow.z() == 0 ? 0.00001f : bottomRow.z();

  const float v1 = cameraInfo.focalLength;
  const float v2 = cameraInfo.opticalCenter.x();
  const float v3 = cameraInfo.opticalCenter.y();
  float x1 = 0;
  float x2 = static_cast<float>(cameraInfo.width - 1);
  float y1 = (v3 * r33 + r31 * v1 + r32 * v2) / r33;
  float y2 = (v3 * r33 + r31 * v1 - r32 * v2) / r33;

  // Mirror the ends of the horizon, if camera is rotated to the left
  if((cameraMatrix.rotation * Vector3f(0, 0, 1)).z() < 0)
  {
    float t = x1;
    x1 = x2;
    x2 = t;
    t = y1;
    y1 = y2;
    y2 = t;
  }

  Geometry::Line horizon;
  horizon.base.x() = (x1 + x2) / 2.f;
  horizon.base.y() = (y1 + y2) / 2.f;
  horizon.direction.x() = x2 - x1;
  horizon.direction.y() = y2 - y1;
  horizon.normalizeDirection();
  return horizon;
}

bool Projection::calculateBallInImage(const Vector2f& ballOffset, const CameraMatrix& cameraMatrix,
                                      const CameraInfo& cameraInfo, float ballRadius, Geometry::Circle& circle)
{
  const Vector2f offset = ballOffset - cameraMatrix.translation.head<2>();
  const float distance = offset.norm();
  const float height = cameraMatrix.translation.z() - ballRadius;
  const float cameraDistance = std::sqrt(sqr(distance) + sqr(height));
  circle.center = Vector2f(atan2(offset.y(), offset.x()), -atan2(height, distance));
  if(cameraDistance >= ballRadius)
  {
    const float alpha = pi_2 - circle.center.y() - std::acos(ballRadius / cameraDistance);
    const float yBottom = -std::atan2(height + std::cos(alpha) * ballRadius, distance - std::sin(alpha) * ballRadius);
    const float beta = pi_2 - circle.center.y() + std::acos(ballRadius / cameraDistance);
    const float yTop = -std::atan2(height + std::cos(beta) * ballRadius, distance - std::sin(beta) * ballRadius);
    Vector2f top, bottom;
    if(!calculatePointByAngles(Vector2f(circle.center.x(), yTop), cameraMatrix, cameraInfo, top))
      return false;
    if(!calculatePointByAngles(Vector2f(circle.center.x(), yBottom), cameraMatrix, cameraInfo, bottom))
      return false;
    circle.center = (top + bottom) / 2.0f;
    circle.radius = (top - bottom).norm() / 2.0f;
    return true;
  }
  else
    return false;
}

float Projection::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels)
{
  const float xFactor = cameraInfo.focalLength;
  return sizeInReality * xFactor / (sizeInPixels + 0.000001f);
}

float Projection::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality,
                                    float sizeInPixels, float centerX, float centerY)
{
  const float mx = centerX;
  const float my = centerY;
  const float cx = cameraInfo.opticalCenter.x();
  const float cy = cameraInfo.opticalCenter.y();
  const float focalLenPow2 = cameraInfo.focalLenPow2;
  const float sqrImgRadius = (mx - cx) * (mx - cx) + (my - cy) * (my - cy);
  const float imgDistance = std::sqrt(focalLenPow2 + sqrImgRadius);
  return imgDistance * sizeInReality / (sizeInPixels + 0.000001f);
}

float Projection::getSizeByDistance(const CameraInfo& cameraInfo, float sizeInReality, float distance)
{
  const float xFactor = cameraInfo.focalLength;
  return sizeInReality / distance * xFactor;
}

void Projection::calculateAnglesForPoint(const Vector2f& point, const CameraMatrix& cameraMatrix,
                                         const CameraInfo& cameraInfo, Vector2f& angles)
{
  const float factor = cameraInfo.focalLength;
  const Vector3f vectorToPoint(factor, cameraInfo.opticalCenter.x() - point.x(), cameraInfo.opticalCenter.y() - point.y());
  const Vector3f vectorToPointWorld = cameraMatrix.rotation * vectorToPoint;
  angles.x() = std::atan2(vectorToPointWorld.y(), vectorToPointWorld.x());
  angles.y() = std::atan2(vectorToPointWorld.z(), std::sqrt(sqr(vectorToPointWorld.x()) + sqr(vectorToPointWorld.y())));
}

bool Projection::calculatePointByAngles(const Vector2f& angles, const CameraMatrix& cameraMatrix,
                                        const CameraInfo& cameraInfo, Vector2f& point)
{
  const Vector3f vectorToPointWorld(std::cos(angles.x()), std::sin(angles.x()), std::tan(angles.y()));
  const Vector3f vectorToPoint(cameraMatrix.rotation.inverse() * vectorToPointWorld);
  if(vectorToPoint.x() <= 0)
    return false;
  const float scale = cameraInfo.focalLength / vectorToPoint.x();
  point.x() = cameraInfo.opticalCenter.x() - vectorToPoint.y() * scale;
  point.y() = cameraInfo.opticalCenter.y() - vectorToPoint.z() * scale;
  return true;
}
