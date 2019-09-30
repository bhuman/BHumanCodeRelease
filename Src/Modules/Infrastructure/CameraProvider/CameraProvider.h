/**
 * @file CameraProvider.h
 *
 * This file declares a module that handles the communication with the two
 * cameras. This implementation starts a separate thread to avoid blocking
 * calls to slow down processing.
 *
 * @author Colin Graf
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/Camera.h"
#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Configuration/AutoExposureWeightTable.h"
#include "Representations/Configuration/CameraIntrinsics.h"
#include "Representations/Configuration/CameraResolutionRequest.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Tools/Module/Module.h"

#include "Tools/Math/Random.h"
#include "Tools/Md5.h"
#include "Tools/RingBuffer.h"

class NaoCamera;

MODULE(CameraProvider,
{,
  USES(AutoExposureWeightTable),
  REQUIRES(CameraResolutionRequest),
  REQUIRES(CameraSettings),
  REQUIRES(CameraImage),
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraStatus),
  PROVIDES_WITHOUT_MODIFY(JPEGImage),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(1000) maxWaitForImage, /** Timeout in ms for waiting for new images. */
    (int)(2000) resetDelay, /** Timeout in ms for resetting camera without image. */
  }),
});

class CameraProvider : public CameraProviderBase
{
  static thread_local CameraProvider* theInstance; /**< Points to the only instance of this class in this thread or is 0 if there is none. */

  CameraInfo::Camera whichCamera;
  NaoCamera* camera = nullptr;
  CameraInfo cameraInfo;
  CameraIntrinsics cameraIntrinsics;
  CameraResolutionRequest cameraResolutionRequest;
  CameraResolutionRequest::Resolutions lastResolutionRequest = CameraResolutionRequest::noRequest;
  volatile bool cameraOk = true;
#ifdef CAMERA_INCLUDED
  static Semaphore performingReset;
  static bool resetPending;
  RingBuffer<std::string, 120> rowBuffer;
  unsigned int currentRow = 0, timestampLastRowChange = 0;
  std::string headName;
  unsigned int lastImageTimestamp = 0;
  unsigned long long lastImageTimestampLL = 0;
#endif

  Thread thread;
  Semaphore takeNextImage;
  Semaphore imageTaken;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(CameraImage& theCameraImage) override;

  void update(CameraInfo& cameraInfo) override;
  void update(CameraIntrinsics& cameraIntrinsics) override {cameraIntrinsics = this->cameraIntrinsics;}
  void update(CameraStatus& cameraStatus) override;
  void update(FrameInfo& frameInfo) override {frameInfo.time = theCameraImage.timestamp;}
  void update(JPEGImage& jpegImage) override;

  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCamera();

  void takeImages();

public:
  CameraProvider();
  ~CameraProvider();

  /**
   * The method returns whether a new image is available.
   * @return Is an new image available?
   */
  static bool isFrameDataComplete();

  /**
   * The method waits for a new image.
   */
  static void waitForFrameData();
};
