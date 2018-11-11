/**
 * @file CoordinateSystemProvider.cpp
 * This file implements a module that provides a coordinate system in image coordinates
 * that is parallel to the ground and compensates for distortions resulting from the
 * rolling shutter.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CoordinateSystemProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Math/Projection.h"

MAKE_MODULE(CoordinateSystemProvider, perception)

CoordinateSystemProvider::CoordinateSystemProvider()
{
  memset(cameraMatrixPrevTimeStamp, 0, sizeof(cameraMatrixPrevTimeStamp));
}

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.cameraInfo = theCameraInfo;

  Geometry::Line horizon = Projection::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.col(0) = horizon.direction;
  imageCoordinateSystem.rotation.col(1) = Vector2f(-horizon.direction.y(), horizon.direction.x());
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixPrev[theCameraInfo.camera];
  RotationMatrix r(theCameraMatrix.rotation.inverse() * cmPrev.rotation);

  Vector2f offset(r.getZAngle(), r.getYAngle());
  // Reject calculated offset if velocity direction changed.
  imageCoordinateSystem.offset.x() = offset.x() * prevOffset.x() < 0 ? 0 : offset.x();
  imageCoordinateSystem.offset.y() = offset.y() * prevOffset.y() < 0 ? 0 : offset.y();
  prevOffset = offset;

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theJointSensorData.timestamp - cameraMatrixPrevTimeStamp[theCameraInfo.camera]);
  cameraMatrixPrev[theCameraInfo.camera] = theCameraMatrix;
  cameraMatrixPrevTimeStamp[theCameraInfo.camera] = theJointSensorData.timestamp;
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const
{
  if(abTimeDiff)
  {
    float timeDiff = (float) int(abTimeDiff) * 0.001f; // in seconds
    float timeDiff2 = (float) int(theFrameInfo.time - theJointSensorData.timestamp) * 0.001f; // in seconds
    a = (timeDiff2 - imageRecordingTime - imageRecordingDelay) / timeDiff;
    b = imageRecordingTime / theCameraInfo.height / timeDiff;
  }
  else
    a = b = 0;
}
