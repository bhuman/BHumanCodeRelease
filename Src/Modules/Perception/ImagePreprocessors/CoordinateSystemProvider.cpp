/**
 * @file CoordinateSystemProvider.cpp
 * This file implements a module that provides a coordinate system in image coordinates
 * that is parallel to the ground and compensates for distortions resulting from the
 * rolling shutter.
 * @author Thomas Röfer
 */

#include "CoordinateSystemProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Math/Projection.h"

MAKE_MODULE(CoordinateSystemProvider, perception)

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.cameraInfo = theCameraInfo;

  Geometry::Line horizon = Projection::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.col(0) = horizon.direction;
  imageCoordinateSystem.rotation.col(1) = Vector2f(-horizon.direction.y(), horizon.direction.x());
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  calcOffset(prevCameraMatrix, theCameraMatrix, prevCameraMatrixOffset, imageCoordinateSystem.offset);
  calcOffset(prevRobotCameraMatrix, theRobotCameraMatrix, prevRobotCameraMatrixOffset, imageCoordinateSystem.robotOffset);
  prevCameraMatrix = theCameraMatrix;
  prevRobotCameraMatrix = theRobotCameraMatrix;

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theJointSensorData.timestamp - prevTimestamp);
  prevTimestamp = theJointSensorData.timestamp;
}

void CoordinateSystemProvider::calcOffset(const Pose3f& prevPose, const Pose3f& currentPose, Vector2f& prevOffset, Vector2f& offset)
{
  RotationMatrix r(currentPose.rotation.inverse() * prevPose.rotation);

  Vector2f rawOffset(r.getZAngle(), r.getYAngle());
  // Reject calculated offset if velocity direction changed.
  offset.x() = rawOffset.x() * prevOffset.x() < 0 ? 0 : rawOffset.x();
  offset.y() = rawOffset.y() * prevOffset.y() < 0 ? 0 : rawOffset.y();
  prevOffset = rawOffset;
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const
{
  if(abTimeDiff)
  {
    const CameraTiming& cameraTiming = cameraTimings[theCameraInfo.camera];
    float timeDiff = static_cast<float>(static_cast<int>(abTimeDiff)) * 0.001f; // in seconds
    float timeDiff2 = static_cast<float>(static_cast<int>(theFrameInfo.time - theJointSensorData.timestamp)) * 0.001f; // in seconds
    a = (timeDiff2 - cameraTiming.imageRecordingTime - cameraTiming.imageRecordingDelay) / timeDiff;
    b = cameraTiming.imageRecordingTime / theCameraInfo.height / timeDiff;
  }
  else
    a = b = 0;
}
