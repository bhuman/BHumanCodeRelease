/**
 * @file CoordinateSystemProvider.cpp
 * This file implements a module that provides a coordinate system in image coordinates
 * that is parallel to the ground and compensates for distortions resulting from the
 * rolling shutter.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "CoordinateSystemProvider.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/RotationMatrix.h"

MAKE_MODULE(CoordinateSystemProvider, perception)

CoordinateSystemProvider::CoordinateSystemProvider()
{
  memset(cameraMatrixPrevTimeStamp, 0, sizeof(cameraMatrixPrevTimeStamp));
}

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.col(0) = horizon.direction;
  imageCoordinateSystem.rotation.col(1) = Vector2f(-horizon.direction.y(), horizon.direction.x());
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixPrev[theCameraInfo.camera];
  RotationMatrix r(theCameraMatrix.rotation.inverse() * cmPrev.rotation);

  Vector2f offset(r.getZAngle(), r.getYAngle());
  if((offset.x() - prevOffset.x()) * (prevOffset.x() - prevPrevOffset.x()) < 0)
    imageCoordinateSystem.offset.x() = 0;
  else
    imageCoordinateSystem.offset.x() = offset.x();
  if((offset.y() - prevOffset.y()) * (prevOffset.y() - prevPrevOffset.y()) < 0)
    imageCoordinateSystem.offset.y() = 0;
  else
    imageCoordinateSystem.offset.y() = offset.y();
  prevPrevOffset = prevOffset;
  prevOffset = offset;

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theJointSensorData.timestamp - cameraMatrixPrevTimeStamp[theCameraInfo.camera]);
  cameraMatrixPrev[theCameraInfo.camera] = theCameraMatrix;
  cameraMatrixPrevTimeStamp[theCameraInfo.camera] = theJointSensorData.timestamp;

  COMPLEX_IMAGE("corrected")
  {
    correctedImage.setResolution(theCameraInfo.width / 2, theCameraInfo.height);
    memset(correctedImage[0], 0, (theCameraInfo.width / 2) * theCameraInfo.height * sizeof(PixelTypes::YUYVPixel));
    int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y();
    for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
      for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y(); yDest <= yDest2; ++yDest)
      {
        int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x() / 2;
        for(int xSrc = 0; xSrc < theCameraInfo.width; xSrc += 2)
        {
          for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x() / 2; xDest <= xDest2; ++xDest)
          {
            correctedImage[yDest + int(theCameraInfo.opticalCenter.y() + 0.5f)][xDest + int(theCameraInfo.opticalCenter.x() + 0.5f) / 2].color = (theImage[ySrc / 2] + theImage.width * (ySrc & 1))[xSrc / 2].color;
          }
        }
      }
    SEND_DEBUG_IMAGE("corrected", correctedImage);
  }

  COMPLEX_IMAGE("horizonAligned")
  {
    horizonAlignedImage.setResolution(theCameraInfo.width / 2, theCameraInfo.height);
    memset(horizonAlignedImage[0], 0, (theCameraInfo.width / 2) * theCameraInfo.height * sizeof(PixelTypes::YUVPixel));
    for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
      for(int xSrc = 0; xSrc < theCameraInfo.width; xSrc += 2)
      {
        Vector2f corrected(imageCoordinateSystem.toCorrected(Vector2i(xSrc, ySrc)));
        corrected.x() -= theCameraInfo.opticalCenter.x();
        corrected.y() -= theCameraInfo.opticalCenter.y();
        const Vector2f& horizonAligned(imageCoordinateSystem.toHorizonAligned(corrected));

        horizonAlignedImage[int(horizonAligned.y() + theCameraInfo.opticalCenter.y() + 0.5f)][int(horizonAligned.x() + theCameraInfo.opticalCenter.x() + 0.5f) / 2].color = (theImage[ySrc / 2] + theImage.width * (ySrc & 1))[xSrc / 2].color;
      }
    SEND_DEBUG_IMAGE("horizonAligned", horizonAlignedImage);
  }
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const
{
  if(abTimeDiff)
  {
    float timeDiff = (float) int(abTimeDiff) * 0.001f; // in seconds
    float timeDiff2 = (float) int(theFrameInfo.time - theJointSensorData.timestamp) * 0.001f; // in seconds
    a = (timeDiff2 - imageRecordingTime - imageRecordingDelay) / timeDiff;
    b = imageRecordingTime / theImage.height / timeDiff;
  }
  else
    a = b = 0;
}
