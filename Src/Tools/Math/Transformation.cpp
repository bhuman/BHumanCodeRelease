/**
 * @file Tools/Math/Transformation.cpp
 *
 * Implements methods for
 * coordinate system transformations.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de.de">Tim Laue</a>
 */

#include "Transformation.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"

using namespace std;

const float MAX_DIST_ON_FIELD = sqrt(10400.f * 10400.f + 7400.f * 7400.f);

Vector2<> Transformation::robotToField(const Pose2D& rp, const Vector2<>& relPos)
{
  return rp * relPos;
}

Vector2<> Transformation::fieldToRobot(const Pose2D& rp, const Vector2<>& fieldCoord)
{
  const float invRotation = -rp.rotation;
  const float s = sin(invRotation);
  const float c = cos(invRotation);
  const float x = rp.translation.x;
  const float y = rp.translation.y;
  return Vector2<>(c * (fieldCoord.x - x) - s * (fieldCoord.y - y), s * (fieldCoord.x - x) + c * (fieldCoord.y - y));
}

bool Transformation::imageToRobot(const float x, const float y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<>& relativePosition)
{
  const float xFactor = cameraInfo.focalLengthInv;
  const float yFactor = cameraInfo.focalLengthInv;
  const Vector3<> vectorToCenter(1.f, (cameraInfo.opticalCenter.x - x) * xFactor,
                                 (cameraInfo.opticalCenter.y - y) * yFactor);
  const Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;
  //Is the point above the horizon ? - return
  if(vectorToCenterWorld.z > -5 * yFactor)
    return false;
  const float a1 = cameraMatrix.translation.x;
  const float a2 = cameraMatrix.translation.y;
  const float a3 = cameraMatrix.translation.z;
  const float b1 = vectorToCenterWorld.x;
  const float b2 = vectorToCenterWorld.y;
  const float b3 = vectorToCenterWorld.z;
  const float f  = a3 / b3;
  relativePosition.x = a1 - f * b1;
  relativePosition.y = a2 - f * b2;
  return std::abs(relativePosition.x) < MAX_DIST_ON_FIELD && std::abs(relativePosition.y) < MAX_DIST_ON_FIELD;
}

bool Transformation::imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<>& relativePosition)
{
  const float xFloat = static_cast<float>(x);
  const float yFloat = static_cast<float>(y);
  return imageToRobot(xFloat, yFloat, cameraMatrix, cameraInfo, relativePosition);
}

bool Transformation::imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<int>& relativePosition)
{
  Vector2<> pointFloat;
  const bool onField = imageToRobot(x, y, cameraMatrix, cameraInfo, pointFloat);
  relativePosition.x = static_cast<int>(std::floor(pointFloat.x + 0.5f));
  relativePosition.y = static_cast<int>(std::floor(pointFloat.y + 0.5f));
  return onField;
}

bool Transformation::imageToRobot(const Vector2<int>& pointInImage, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<>& relativePosition)
{
  return imageToRobot(pointInImage.x, pointInImage.y, cameraMatrix, cameraInfo, relativePosition);
}

bool Transformation::imageToRobotHorizontalPlane(const Vector2<>& pointInImage, float z,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
    Vector2<>& pointOnPlane)
{
  const float xFactor = cameraInfo.focalLengthInv;
  const float yFactor = cameraInfo.focalLengthInv;
  const Vector3<> vectorToCenter(1.f, (cameraInfo.opticalCenter.x - pointInImage.x) * xFactor,
                                 (cameraInfo.opticalCenter.y - pointInImage.y) * yFactor);
  const Vector3<> vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;
  const float a1 = cameraMatrix.translation.x;
  const float a2 = cameraMatrix.translation.y;
  const float a3 = cameraMatrix.translation.z - z;
  const float b1 = vectorToCenterWorld.x;
  const float b2 = vectorToCenterWorld.y;
  const float b3 = vectorToCenterWorld.z;
  if(abs(b3) > 0.00001)
  {
    pointOnPlane.x = (a1 * b3 - a3 * b1) / b3;
    pointOnPlane.y = (a2 * b3 - a3 * b2) / b3;
    return true;
  }
  else
  {
    return false;
  }
}

bool Transformation::robotToImage(const Vector2<>& point, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<float>& pointInImage)
{
  const Vector3<> point3D(point.x, point.y, 0.f);
  return robotToImage(point3D, cameraMatrix, cameraInfo, pointInImage);
}

bool Transformation::robotToImage(const Vector3<>& point, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2<float>& pointInImage)
{
  Vector3<> pointInCam = cameraMatrix.invert() * point;
  if(pointInCam.x < 0)
  {
    return false;
  }
  pointInCam *= cameraInfo.focalLength / pointInCam.x;
  pointInImage = cameraInfo.opticalCenter - Vector2<>(pointInCam.y, pointInCam.z);
  return pointInCam.x > 0;
}

bool Transformation::robotWithCameraRotationToImage(const Vector2<>& point,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
    Vector2<>& pointInImage)
{
  Pose3D cameraRotatedMatrix;
  cameraRotatedMatrix.rotateZ(cameraMatrix.rotation.getZAngle());
  const Vector3<> point3D(point.x, point.y, 0.f);
  return robotToImage(cameraRotatedMatrix * point3D, cameraMatrix, cameraInfo, pointInImage);
}

bool Transformation::imageToRobotWithCameraRotation(const Vector2<int>& pointInImage,
    const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
    Vector2<>& relativePosition)
{
  const bool ret = imageToRobot(pointInImage, cameraMatrix, cameraInfo, relativePosition);
  if(ret)
  {
    Pose3D cameraRotatedMatrix;
    cameraRotatedMatrix.rotateZ(cameraMatrix.rotation.getZAngle());
    const Vector3<> point3D = cameraRotatedMatrix.invert() * Vector3<>(relativePosition.x, relativePosition.y, 0.f);
    relativePosition.x = point3D.x;
    relativePosition.y = point3D.y;
  }
  return ret;
}
