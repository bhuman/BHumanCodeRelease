/**
 * @file Tools/Math/Transformation.cpp
 *
 * Implements methods for coordinate system transformations.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "Transformation.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/RotationMatrix.h"

static constexpr float MAX_DIST_ON_FIELD = 142127.f; // Human soccer field diagonal

Vector2f Transformation::robotToField(const Pose2f& rp, const Vector2f& relPos)
{
  float s = std::sin(rp.rotation);
  float c = std::cos(rp.rotation);
  return Vector2f(relPos.x() * c - relPos.y() * s, relPos.x() * s + relPos.y() * c) + rp.translation;
}

Vector2f Transformation::fieldToRobot(const Pose2f& rp, const Vector2f& fieldCoord)
{
  const float invRotation = -rp.rotation;
  const float s = std::sin(invRotation);
  const float c = std::cos(invRotation);
  const float x = rp.translation.x();
  const float y = rp.translation.y();
  return Vector2f(c * (fieldCoord.x() - x) - s * (fieldCoord.y() - y), s * (fieldCoord.x() - x) + c * (fieldCoord.y() - y));
}

bool Transformation::imageToRobot(const Vector2f& pointInImage, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& relativePosition)
{
  return imageToRobot(pointInImage.x(), pointInImage.y(), cameraMatrix, cameraInfo, relativePosition);
}

bool Transformation::imageToRobot(const float x, const float y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& relativePosition)
{
  const float xFactor = cameraInfo.focalLengthInv;
  const float yFactor = cameraInfo.focalLengthHeightInv;
  const Vector3f vectorToCenter(1.f, (cameraInfo.opticalCenter.x() - x) * xFactor,
                                (cameraInfo.opticalCenter.y() - y) * yFactor);
  const Vector3f vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;
  //Is the point above the horizon ? - return
  if(vectorToCenterWorld.z() > -5 * yFactor)
    return false;
  const float a1 = cameraMatrix.translation.x();
  const float a2 = cameraMatrix.translation.y();
  const float a3 = cameraMatrix.translation.z();
  const float b1 = vectorToCenterWorld.x();
  const float b2 = vectorToCenterWorld.y();
  const float b3 = vectorToCenterWorld.z();
  const float f = a3 / b3;
  relativePosition.x() = a1 - f * b1;
  relativePosition.y() = a2 - f * b2;
  return std::abs(relativePosition.x()) < MAX_DIST_ON_FIELD && std::abs(relativePosition.y()) < MAX_DIST_ON_FIELD;
}

bool Transformation::imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& relativePosition)
{
  const float xFloat = static_cast<float>(x);
  const float yFloat = static_cast<float>(y);
  return imageToRobot(xFloat, yFloat, cameraMatrix, cameraInfo, relativePosition);
}

bool Transformation::imageToRobot(const int x, const int y, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2i& relativePosition)
{
  Vector2f pointFloat;
  const bool onField = imageToRobot(x, y, cameraMatrix, cameraInfo, pointFloat);
  relativePosition.x() = static_cast<int>(std::floor(pointFloat.x() + 0.5f));
  relativePosition.y() = static_cast<int>(std::floor(pointFloat.y() + 0.5f));
  return onField;
}

bool Transformation::imageToRobot(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& relativePosition)
{
  return imageToRobot(pointInImage.x(), pointInImage.y(), cameraMatrix, cameraInfo, relativePosition);
}

bool Transformation::imageToRobotHorizontalPlane(const Vector2f& pointInImage, float z,
                                                 const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2f& pointOnPlane)
{
  const float xFactor = cameraInfo.focalLengthInv;
  const float yFactor = cameraInfo.focalLengthHeightInv;
  const Vector3f vectorToCenter(1.f, (cameraInfo.opticalCenter.x() - pointInImage.x()) * xFactor,
                                (cameraInfo.opticalCenter.y() - pointInImage.y()) * yFactor);
  const Vector3f vectorToCenterWorld = cameraMatrix.rotation * vectorToCenter;
  const float a1 = cameraMatrix.translation.x();
  const float a2 = cameraMatrix.translation.y();
  const float a3 = cameraMatrix.translation.z() - z;
  const float b1 = vectorToCenterWorld.x();
  const float b2 = vectorToCenterWorld.y();
  const float b3 = vectorToCenterWorld.z();
  if(std::abs(b3) > 0.00001)
  {
    pointOnPlane.x() = (a1 * b3 - a3 * b1) / b3;
    pointOnPlane.y() = (a2 * b3 - a3 * b2) / b3;
    return true;
  }
  else
  {
    return false;
  }
}

bool Transformation::robotToImage(const Vector3f& point, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& pointInImage)
{
  Vector3f pointInCam = cameraMatrix.inverse() * point;
  if(pointInCam.x() <= 0)
    return false;

  pointInCam /= pointInCam.x();
  pointInImage = cameraInfo.opticalCenter - pointInCam.tail<2>().cwiseProduct(Vector2f(cameraInfo.focalLength, cameraInfo.focalLengthHeight));
  return pointInCam.x() > 0;
}

bool Transformation::robotToImage(const Vector2f& point, const CameraMatrix& cameraMatrix,
                                  const CameraInfo& cameraInfo, Vector2f& pointInImage)
{
  const Vector3f point3D(point.x(), point.y(), 0.f);
  return robotToImage(point3D, cameraMatrix, cameraInfo, pointInImage);
}

bool Transformation::robotWithCameraRotationToImage(const Vector2f& point, const CameraMatrix& cameraMatrix,
                                                    const CameraInfo& cameraInfo, Vector2f& pointInImage)
{
  Pose3f cameraRotatedMatrix;
  cameraRotatedMatrix.rotateZ(cameraMatrix.rotation.getZAngle());
  const Vector3f point3D(point.x(), point.y(), 0.f);
  return robotToImage(cameraRotatedMatrix * point3D, cameraMatrix, cameraInfo, pointInImage);
}

bool Transformation::imageToRobotWithCameraRotation(const Vector2i& pointInImage, const CameraMatrix& cameraMatrix,
                                                    const CameraInfo& cameraInfo, Vector2f& relativePosition)
{
  const bool ret = imageToRobot(pointInImage, cameraMatrix, cameraInfo, relativePosition);
  if(ret)
  {
    Pose3f cameraRotatedMatrix;
    cameraRotatedMatrix.rotateZ(cameraMatrix.rotation.getZAngle());
    const Vector3f point3D = cameraRotatedMatrix.inverse() * Vector3f(relativePosition.x(), relativePosition.y(), 0.f);
    relativePosition = point3D.head<2>();
  }
  return ret;
}
