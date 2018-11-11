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
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/CameraResolution.h"
#include "Representations/Infrastructure/CameraSettings.h"
#include "Representations/Infrastructure/CameraStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Module/Module.h"

class NaoCamera;

MODULE(CameraProvider,
{,
  USES(CameraResolutionRequest),
  USES(AutoExposureWeightTable),
  REQUIRES(CameraSettings),
  REQUIRES(Image),
  REQUIRES(RobotInfo),
  PROVIDES_WITHOUT_MODIFY(Image),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES(CameraResolution),
  PROVIDES(CameraStatus),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(1000) maxWaitForImage, /** Timeout in ms for waiting for new images. */
  }),
});

class CameraProvider : public CameraProviderBase
{
  static thread_local CameraProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* upperCamera = nullptr;
  NaoCamera* lowerCamera = nullptr;
  NaoCamera* currentImageCamera = nullptr;
  CameraInfo upperCameraInfo;
  CameraInfo lowerCameraInfo;
  CameraIntrinsics cameraIntrinsics;
  CameraResolution cameraResolution;
  volatile bool camerasOk = true;
#ifdef CAMERA_INCLUDED
  unsigned int lastImageTimeStamp = 0;
  unsigned long long lastImageTimeStampLL = 0;
#endif

  Thread thread;
  Semaphore takeNextImage;
  Semaphore imageTaken;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theImage The representation updated.
   */
  void update(Image& theImage) override;

  void update(CameraInfo& cameraInfo) override;
  void update(CameraIntrinsics& cameraIntrinsics) override {cameraIntrinsics = this->cameraIntrinsics;}
  void update(CameraResolution& cameraResolution) override {cameraResolution = this->cameraResolution;}
  void update(CameraStatus& cameraStatus) override;
  void update(FrameInfo& frameInfo) override {frameInfo.time = theImage.timeStamp;}

  void useImage(bool isUpper, unsigned timestamp, CameraInfo& cameraInfo, Image& image, NaoCamera* naoCam);

  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCameras();

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
