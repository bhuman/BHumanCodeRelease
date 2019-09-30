/**
 * @file BodyContourProvider.cpp
 * This file implements a module that provides the contour of the robot's body in the image.
 * The contour can be used to exclude the robot's body from image processing.
 * @author Thomas Röfer
 */

#include "BodyContourProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BodyContourProvider::update(BodyContour& bodyContour)
{
  DECLARE_DEBUG_DRAWING3D("module:BodyContourProvider:contour", "robot");

  bodyContour.cameraResolution.x() = theCameraInfo.width;
  bodyContour.cameraResolution.y() = theCameraInfo.height;
  bodyContour.lines.clear();

  robotCameraMatrixInverted = theRobotCameraMatrix.inverse();
  add(Pose3f(), torso, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::bicepsLeft], shoulder, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::bicepsRight], shoulder, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::bicepsLeft], upperArm, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::bicepsRight], upperArm, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::foreArmLeft], lowerArm, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::foreArmRight], lowerArm, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::foreArmLeft], lowerArm2, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::foreArmRight], lowerArm2, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::thighLeft], upperLeg1, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::thighRight], upperLeg1, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::thighLeft], upperLeg2, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::thighRight], upperLeg2, -1, bodyContour);
  add(theRobotModel.limbs[Limbs::footLeft], foot, 1, bodyContour);
  add(theRobotModel.limbs[Limbs::footRight], foot, -1, bodyContour);
}

void BodyContourProvider::add(const Pose3f& origin, const std::vector<Vector3f>& c, float sign,
                              BodyContour& bodyContour)
{
  Vector2i q1,
           q2;
  Vector3f p1 = origin * Vector3f(c[0].x(), c[0].y() * sign, c[0].z());
  bool valid1 = calculatePointInImage(p1, q1);

  for(unsigned i = 1; i < c.size(); ++i)
  {
    Vector3f p2 = origin * Vector3f(c[i].x(), c[i].y() * sign, c[i].z());
    bool valid2 = calculatePointInImage(p2, q2);

    if(valid1 && valid2 &&
       (q1.y() < theCameraInfo.height || q2.y() < theCameraInfo.height) &&
       (q1.x() >= 0 || q2.x() >= 0) &&
       (q1.x() < theCameraInfo.width || q2.x() < theCameraInfo.width))
      bodyContour.lines.push_back(BodyContour::Line(q1, q2));

    q1 = q2;
    valid1 = valid2;

    COMPLEX_DRAWING3D("module:BodyContourProvider:contour")
    {
      LINE3D("module:BodyContourProvider:contour", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z(), 1, ColorRGBA(255, 0, 0));
      p1 = p2;
    }
  }
}

bool BodyContourProvider::calculatePointInImage(const Vector3f& pointInWorld, Vector2i& pointInImage) const
{
  Vector3f pointInCamera(robotCameraMatrixInverted * pointInWorld);
  if(pointInCamera.x() > 1.0f)
  {
    pointInCamera /= pointInCamera.x();
    Vector2f corrected(theCameraInfo.opticalCenter - pointInCamera.tail<2>().cwiseProduct(Vector2f(theCameraInfo.focalLength, theCameraInfo.focalLengthHeight)));
    pointInImage = theImageCoordinateSystem.fromCorrectedRobot(corrected).cast<int>();
    return (pointInImage.x() >= 0 || corrected.x() < theCameraInfo.height)
           && (pointInImage.x() < theCameraInfo.height || corrected.x() >= 0);
  }
  else
    return false;
}

MAKE_MODULE(BodyContourProvider, perception)
