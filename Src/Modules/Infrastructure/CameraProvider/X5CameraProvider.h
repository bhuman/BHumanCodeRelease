/**
 * @file X5CameraProvider.h
 *
 * This file declares a module that provides camera images on the K1 robot
 * using Rico Tilgner's \c cam_stream service.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JPEGImageRequest.h"
#ifdef TARGET_ROBOT
#include <cam_stream_receiver.hpp>
#endif

MODULE(X5CameraProvider,
{,
  REQUIRES(CameraImage),
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES(FrameInfo),
  USES(JPEGImageRequest),
  LOADS_PARAMETERS(
  {,
    (std::string) ip, /**< The ip address of the host the camera service runs on. */
    (short) port, /**< The ip port of the service. */
    (unsigned char) channel, /**< 0: left camera, 1: right camera. */
    (int) maxWaitForImage, /**< Timeout in ms for waiting for new images after the camera was just set up. */
    (int) minWaitForImage, /**< Timeout in ms for waiting for new images when the camera was previously working. */
  }),
});

class X5CameraProvider : public X5CameraProviderBase
{
  DECLARE_SYNC;
  thread_local static X5CameraProvider* theInstance; /**< Points to the only instance of this class in this thread or is nullptr if there is none. */
  Semaphore imageSignal; /**< A signal used for announcing that a new image has arrived to waitForFrameData(). */
  CameraInfo cameraInfo; /**< The intrinsic calibration of the color camera. */
  CameraIntrinsics cameraIntrinsics; /**< The intrinsic calibration of the cameras. */
  CameraResolutionRequest cameraResolutionRequest; /**< The resolution request for the cameras.  */
#ifdef TARGET_ROBOT
  cam_stream::CamStreamReceiver receiver; /**< The receiver for data from the camera service. */
  cam_stream::CamFrame frame; /**< The latest frame received. */
  unsigned lastImageTimestamp = 0; /**< The timestamp of the last image received. */
  bool intrinsicsUpdated = false; /**< Were the camera intrinsics and camera info updated with the values received from the camera? */
#endif

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(CameraImage& theCameraImage) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraInfo The representation updated.
   */
  void update(CameraInfo& theCameraInfo) override {theCameraInfo = cameraInfo;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraIntrinsics The representation updated.
   */
  void update(CameraIntrinsics& theCameraIntrinsics) override {theCameraIntrinsics = cameraIntrinsics;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {theFrameInfo.time = theCameraImage.timestamp;}

  /** Read the camera intrinsics from a configuration file. */
  bool readCameraIntrinsics();

  /** Read the initial camera resolution from a configuration file. */
  bool readCameraResolution();

  /** Setup the camera information. */
  void updateCameraInfo();

public:
  X5CameraProvider();
  ~X5CameraProvider();

  /**
   * The method returns whether a new image is available.
   * @return Is an new image available?
   */
  static bool isFrameDataComplete();

  /** The method waits for a new image. */
  static void waitForFrameData();
};
