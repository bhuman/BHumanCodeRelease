/**
 * @file CoordinateSystemProvider.h
 * This file declares a module that provides coordinate systems.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/ImageProcessing/Image.h"

namespace CoordinateSystemProviderModule
{
  STREAMABLE(CameraTiming,
  {,
    (float) imageRecordingTime, /**< Time the camera requires to take an image (in s, for motion compensation, may depend on exposure). */
    (float) imageRecordingDelay, /**< Delay after the camera took an image (in s, for motion compensation). */
  });
}

MODULE(CoordinateSystemProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FrameInfo),
  REQUIRES(JointSensorData), // for timestamp only
  REQUIRES(RobotCameraMatrix),
  PROVIDES(ImageCoordinateSystem),
  LOADS_PARAMETERS(
  {
    using CameraTiming = CoordinateSystemProviderModule::CameraTiming,

    (ENUM_INDEXED_ARRAY(CameraTiming, CameraInfo::Camera)) cameraTimings, /** The timings of both cameras. */
  }),
});

class CoordinateSystemProvider : public CoordinateSystemProviderBase
{
  /**
   * Updates the image coordinate system provided by this module.
   */
  void update(ImageCoordinateSystem& imageCoordinateSystem) override;

  /**
   * The method calculates the angular offset between two camera poses
   * @param prevPose The previous camera pose
   * @param currentPose The current camera pose
   * @param prevOffset The previous angular offset (is overwritten)
   * @param offset The resulting angular offset
   */
  void calcOffset(const Pose3f& prevPose, const Pose3f& currentPose, Vector2f& prevOffset, Vector2f& offset);

  /**
   * The method calculates the scaling factors for the distored image.
   * @param a The constant part of the equation for motion distortion will be returned here.
   * @param b The linear part of the equation for motion distortion will be returned here.
   */
  void calcScaleFactors(float& a, float& b, unsigned int abTimeDiff) const;

  CameraMatrix prevCameraMatrix;
  RobotCameraMatrix prevRobotCameraMatrix;
  Vector2f prevCameraMatrixOffset = Vector2f::Zero();
  Vector2f prevRobotCameraMatrixOffset = Vector2f::Zero();
  unsigned prevTimestamp = 0;
};
