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
#include "Framework/Module.h"

#include "Math/Random.h"
#include "Math/RingBuffer.h"

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
  LOADS_PARAMETERS(
  {,
    (unsigned) maxWaitForImage, /**< Timeout in ms for waiting for new images. */
    (int) resetDelay, /**< Timeout in ms for resetting camera without image. */
    (int) jpegQuality, /**< The quality of the JPEG compressing (0 = bad ... 100 = very good). */
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
  CameraResolutionRequest::Resolutions lastResolutionRequest = CameraResolutionRequest::defaultRes;
  volatile bool cameraOk = true;
#ifdef TARGET_ROBOT
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
