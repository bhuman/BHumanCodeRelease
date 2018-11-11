/**
 * @file InImageSizeCalculations.cpp
 *
 * A namespace with methods to help validate perception.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
#include "InImageSizeCalculations.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"

float IISC::getImageBallRadiusByCenter(const Vector2f& center, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatix, const BallSpecification& theBallSpecification)
{
  Vector2f centerPoint;
  if(!Transformation::imageToRobotHorizontalPlane(center, theBallSpecification.radius, theCameraMatix, theCameraInfo, centerPoint))
    return -1.f;

  const Vector2f slightlyRightPoint(center.x() + 1.f, center.y());
  Vector2f slightlyRightField;
  if(!Transformation::imageToRobotHorizontalPlane(slightlyRightPoint, theBallSpecification.radius, theCameraMatix, theCameraInfo, slightlyRightField))
    return -1.f;

  const Vector3f centerPoint3f(centerPoint.x(), centerPoint.y(), theBallSpecification.radius);
  const Vector3f cameraPointVector(centerPoint3f - theCameraMatix.translation);
  const Vector3f slightlyRightField3f(slightlyRightField.x(), slightlyRightField.y(), theBallSpecification.radius);
  const Vector3f rightVector(slightlyRightField3f - centerPoint3f);

  const Vector3f dir(cameraPointVector.cross(rightVector));

  const Vector3f newPoint(centerPoint3f + dir.normalized(theBallSpecification.radius));

  Vector2f newPointInImage;
  if(!Transformation::robotToImage(newPoint, theCameraMatix, theCameraInfo, newPointInImage))
    return -1.f;

  return (center - newPointInImage).norm();
}

float IISC::getImageBallRadiusByLowestPoint(const Vector2f& start, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatix, const BallSpecification& theBallSpecification)
{
  Vector2f startPoint;
  if(!Transformation::imageToRobot(start, theCameraMatix, theCameraInfo, startPoint))
    return -1.f;

  const Vector2f slightlyRightPoint(start.x() + 1.f, start.y());
  Vector2f slightlyRightField;
  if(!Transformation::imageToRobot(slightlyRightPoint, theCameraMatix, theCameraInfo, slightlyRightField))
    return -1.f;

  const Vector3f centerPoint3f(startPoint.x(), startPoint.y(), theBallSpecification.radius);
  const Vector3f cameraPointVector(centerPoint3f - theCameraMatix.translation);
  const Vector3f slightlyRightField3f(slightlyRightField.x(), slightlyRightField.y(), theBallSpecification.radius);
  const Vector3f rightVector(slightlyRightField3f - centerPoint3f);

  const Vector3f dir(rightVector.cross(cameraPointVector));

  const Vector3f ballRayIntersectionOffset(-dir.normalized(theBallSpecification.radius) - Vector3f(0.f, 0.f, -theBallSpecification.radius));

  Vector2f ballPointByStart;
  if(!Transformation::imageToRobotHorizontalPlane(start, ballRayIntersectionOffset.z(), theCameraMatix, theCameraInfo, ballPointByStart))
    return -1.f;

  const Vector3f ballPointByStart3f(ballPointByStart.x(), ballPointByStart.y(), ballRayIntersectionOffset.z());
  const Vector3f highesVisibleBallPoint(ballPointByStart3f + dir.normalized(2.f * theBallSpecification.radius));

  Vector2f highesImagePoint;
  if(!Transformation::robotToImage(highesVisibleBallPoint, theCameraMatix, theCameraInfo, highesImagePoint))
    return -1.f;

  return (start - highesImagePoint).norm() / 2.f;
}

float IISC::calcBallVisibilityInImageByCenter(const Vector2f& center, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatix, const BallSpecification& theBallSpecification, const Angle greenEdge)
{
  Vector2f centerPoint;
  if(!Transformation::imageToRobot(center, theCameraMatix, theCameraInfo, centerPoint))
    return -1.f;

  const Vector2f slightlyRightPoint(center.x() + 1.f, center.y());
  Vector2f slightlyRightField;
  if(!Transformation::imageToRobot(slightlyRightPoint, theCameraMatix, theCameraInfo, slightlyRightField))
    return -1.f;

  const Vector3f centerPoint3f(centerPoint.x(), centerPoint.y(), theBallSpecification.radius);
  const Vector3f cameraPointVector(centerPoint3f - theCameraMatix.translation);
  const Vector3f slightlyRightField3f(slightlyRightField.x(), slightlyRightField.y(), theBallSpecification.radius);
  const Vector3f rightVector(slightlyRightField3f - centerPoint3f);

  const Vector3f dir(rightVector.cross(cameraPointVector));
  const Angle dirAngle(std::asin(dir.normalized(theBallSpecification.radius).z() / theBallSpecification.radius));
  if(greenEdge + dirAngle < pi_2)
    return 1.f;

  const Angle angleDif = greenEdge + dirAngle - pi_2;
  ASSERT(angleDif > 0_deg && angleDif <= 90_deg);
  return 0.5f + 0.5f * std::cos(angleDif);
}

bool IISC::calcPossibleVisibleBallByLowestPoint(const Vector2f& start, Geometry::Circle& circle, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatix, const BallSpecification& theBallSpecification, const Angle greenEdge)
{
  Vector2f startPoint;
  if(!Transformation::imageToRobot(start, theCameraMatix, theCameraInfo, startPoint))
    return false;

  const Vector2f slightlyRightPoint(start.x() + 1.f, start.y());
  Vector2f slightlyRightField;
  if(!Transformation::imageToRobot(slightlyRightPoint, theCameraMatix, theCameraInfo, slightlyRightField))
    return false;

  const Vector3f centerPoint3f(startPoint.x(), startPoint.y(), theBallSpecification.radius);
  const Vector3f cameraPointVector(centerPoint3f - theCameraMatix.translation);
  const Vector3f slightlyRightField3f(slightlyRightField.x(), slightlyRightField.y(), theBallSpecification.radius);
  const Vector3f rightVector(slightlyRightField3f - centerPoint3f);

  const Vector3f dir(rightVector.cross(cameraPointVector));

  const Angle dirAngle(std::asin(dir.normalized(theBallSpecification.radius).z() / theBallSpecification.radius));

  const Angle rotateAngle(greenEdge + dirAngle < pi_2 ? 0_rad : Angle(greenEdge + dirAngle - pi_2));
  const float visiblePercentage = 0.5f + 0.5f * std::cos(rotateAngle);
  const AngleAxisf rotationAxis(-rotateAngle, Vector3f(dir.y(), dir.x(), 0.f).normalized());
  const Vector3f useDir(RotationMatrix(rotationAxis) * dir.normalized(theBallSpecification.radius));

  const Vector3f ballRayIntersectionOffset(-useDir - Vector3f(0.f, 0.f, -theBallSpecification.radius));

  Vector2f ballPointByStart;
  if(!Transformation::imageToRobotHorizontalPlane(start, ballRayIntersectionOffset.z(), theCameraMatix, theCameraInfo, ballPointByStart))
    return false;

  const Vector3f ballPointByStart3f(ballPointByStart.x(), ballPointByStart.y(), ballRayIntersectionOffset.z());
  const Vector3f highesVisibleBallPoint(ballPointByStart3f + dir.normalized((2.f * theBallSpecification.radius) * visiblePercentage));

  Vector2f highesImagePoint;
  if(!Transformation::robotToImage(highesVisibleBallPoint, theCameraMatix, theCameraInfo, highesImagePoint))
    return false;

  const float visibleDiameter = (start - highesImagePoint).norm();

  circle.radius = (visibleDiameter / visiblePercentage) / 2.f;
  circle.center = highesImagePoint + (start - highesImagePoint).normalized(circle.radius);

  return true;
}

float IISC::getImageLineDiameterByLowestPoint(const Vector2f& start, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatix, const FieldDimensions& theFieldDimensions)
{
  return getImageDiameterByLowestPointAndFieldDiameter(theFieldDimensions.fieldLinesWidth, start, theCameraInfo, theCameraMatix);
}

float IISC::getImageGoalPostFootWidthDiameterByFootMiddlePoint(const Vector2f& midlle, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix, const FieldDimensions& theFieldDimensions)
{
  return getHorizontalImageDiameterByMiddlePointAndFieldDiameter(theFieldDimensions.goalPostRadius * 2.f, midlle, theCameraInfo, theCameraMatrix);
}

float IISC::getImagePenaltyMarkDiameterByLowestPoint(const Vector2f& start, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix, const FieldDimensions& theFieldDimensions)
{
  return getImageDiameterByLowestPointAndFieldDiameter(theFieldDimensions.penaltyMarkSize, start, theCameraInfo, theCameraMatrix);
}

bool IISC::calculateImagePenaltyMeasurementsByCenter(const Vector2f& center, float& length, float& height, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix, const FieldDimensions& theFieldDimensions)
{
  height = getImageDiameterByLowestPointAndFieldDiameter(theFieldDimensions.penaltyMarkSize, center, theCameraInfo, theCameraMatrix);
  length = getHorizontalImageDiameterByMiddlePointAndFieldDiameter(theFieldDimensions.penaltyMarkSize, center, theCameraInfo, theCameraMatrix);
  return height >= 0.f && length >= 0.f;
}

float IISC::getImageDiameterByLowestPointAndFieldDiameter(const float fieldDiameter, const Vector2f& start, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix)
{
  Vector2f point;
  if(!Transformation::imageToRobot(start, theCameraMatrix, theCameraInfo, point))
    return -1.f;

  point.normalize(point.norm() + fieldDiameter);

  Vector2f newImagePoint;
  if(!Transformation::robotToImage(point, theCameraMatrix, theCameraInfo, newImagePoint))
    return -1.f;

  return (newImagePoint - start).norm();
}

float IISC::getHorizontalImageDiameterByMiddlePointAndFieldDiameter(const float fieldDiameter, const Vector2f& middle, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix)
{
  Vector2f point;
  if(!Transformation::imageToRobot(middle, theCameraMatrix, theCameraInfo, point))
    return -1.f;

  point += point.normalized(fieldDiameter / 2.f).rotateLeft();

  Vector2f newImagePoint;
  if(!Transformation::robotToImage(point, theCameraMatrix, theCameraInfo, newImagePoint))
    return -1.f;

  return 2.f * (newImagePoint - middle).norm();
}
