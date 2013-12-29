/**
* @file CoordinateSystemProvider.cpp
* This file implements a module that provides a coordinate system in image coordinates
* that is parallel to the ground and compensates for distortions resulting from the
* rolling shutter.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "CoordinateSystemProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

void CoordinateSystemProvider::update(ImageCoordinateSystem& imageCoordinateSystem)
{
  imageCoordinateSystem.setCameraInfo(theCameraInfo);

  Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);
  imageCoordinateSystem.origin = horizon.base;
  imageCoordinateSystem.rotation.c[0] = horizon.direction;
  imageCoordinateSystem.rotation.c[1] = Vector2<>(-horizon.direction.y, horizon.direction.x);
  imageCoordinateSystem.invRotation = imageCoordinateSystem.rotation.transpose();

  const CameraMatrix& cmPrev = cameraMatrixPrev[theCameraInfo.camera];
  RotationMatrix r(theCameraMatrix.rotation.transpose() * cmPrev.rotation);
  imageCoordinateSystem.offset = Vector2<>(r.getZAngle(), r.getYAngle());

  calcScaleFactors(imageCoordinateSystem.a, imageCoordinateSystem.b, theFilteredJointData.timeStamp - cameraMatrixPrevTimeStamp[theCameraInfo.camera]);
  cameraMatrixPrev[theCameraInfo.camera] = theCameraMatrix;
  cameraMatrixPrevTimeStamp[theCameraInfo.camera] = theFilteredJointData.timeStamp;

  COMPLEX_DEBUG_IMAGE(corrected,
  {
    INIT_DEBUG_IMAGE_BLACK(corrected, theCameraInfo.width, theCameraInfo.height);
    int yDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, 0).y;
    for(int ySrc = 0; ySrc < theImage.height; ++ySrc)
      for(int yDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).y; yDest <= yDest2; ++yDest)
      {
        int xDest = -imageCoordinateSystem.toCorrectedCenteredNeg(0, ySrc).x;
        for(int xSrc = 0; xSrc < theImage.width; ++xSrc)
        {
          for(int xDest2 = -imageCoordinateSystem.toCorrectedCenteredNeg(xSrc, ySrc).x; xDest <= xDest2; ++xDest)
          {
            DEBUG_IMAGE_SET_PIXEL_YUV(corrected, xDest + int(theCameraInfo.opticalCenter.x + 0.5f),
                                                 yDest + int(theCameraInfo.opticalCenter.y + 0.5f),
                                                 theImage[ySrc][xSrc].y,
                                                 theImage[ySrc][xSrc].cb,
                                                 theImage[ySrc][xSrc].cr);
          }
        }
      }
    SEND_DEBUG_IMAGE(corrected);
  });

  COMPLEX_DEBUG_IMAGE(horizonAligned,
  {
    INIT_DEBUG_IMAGE_BLACK(horizonAligned, theCameraInfo.width, theCameraInfo.height);
    for(int ySrc = 0; ySrc < theCameraInfo.height; ++ySrc)
      for(int xSrc = 0; xSrc < theCameraInfo.width; ++xSrc)
      {
        Vector2<> corrected(imageCoordinateSystem.toCorrected(Vector2<int>(xSrc, ySrc)));
        corrected.x -= theCameraInfo.opticalCenter.x;
        corrected.y -= theCameraInfo.opticalCenter.y;
        const Vector2<>& horizonAligned(imageCoordinateSystem.toHorizonAligned(corrected));

        DEBUG_IMAGE_SET_PIXEL_YUV(horizonAligned, int(horizonAligned.x + theCameraInfo.opticalCenter.x + 0.5f),
                                                  int(horizonAligned.y + theCameraInfo.opticalCenter.y + 0.5f),
                                                  theImage[ySrc][xSrc].y,
                                                  theImage[ySrc][xSrc].cb,
                                                  theImage[ySrc][xSrc].cr);
      }
    SEND_DEBUG_IMAGE(horizonAligned);
  });
}

void CoordinateSystemProvider::calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const
{
  if(abTimeDiff)
  {
    float timeDiff = (float) int(abTimeDiff) * 0.001f; // in seconds
    float timeDiff2 = (float) int(theFrameInfo.time - theFilteredJointData.timeStamp) * 0.001f; // in seconds
    a = (timeDiff2 - imageRecordingTime - imageRecordingDelay) / timeDiff;
    b = imageRecordingTime / theImage.height / timeDiff;
  }
  else
    a = b = 0;
}

MAKE_MODULE(CoordinateSystemProvider, Perception)
