/**
 * @file CameraProvider.h
 * This file declares a module that provides camera images.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Platform/Camera.h"
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
  USES(CameraIntrinsicsNext),
  USES(CameraResolutionRequest),
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
    (unsigned)(10000) maxDelayAfterInit, /**< Maximum delay until image is received after camera was initialized. */
    (unsigned)(2000) notOkDelay, /** How long after first camera reset to report that camera is not ok. */
  }),
});

class CameraProvider : public CameraProviderBase
{
private:
  static thread_local CameraProvider* theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* upperCamera = nullptr;
  NaoCamera* lowerCamera = nullptr;
  NaoCamera* currentImageCamera = nullptr;
  CameraInfo upperCameraInfo;
  CameraInfo lowerCameraInfo;
  CameraIntrinsics cameraIntrinsics;
  CameraResolution cameraResolution;
  float cycleTime;
#ifdef CAMERA_INCLUDED
  unsigned int upperImageReceived;
  unsigned int lowerImageReceived;
  unsigned int lastCameraReset = 0;
  unsigned int timeWhenCamerasWereOk = 0;
  unsigned int lastImageTimeStamp = 0;
  unsigned long long lastImageTimeStampLL = 0;
#endif

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
  void waitForFrameData2();

private:
  void update(Image& image);
  void update(FrameInfo& frameInfo);
  void update(CameraInfo& cameraInfo);
  void update(CameraIntrinsics& cameraIntrinsics);
  void update(CameraResolution& cameraResolution);
  void update(CameraStatus& cameraStatus);

  void useImage(bool isUpper, CameraInfo& cameraInfo, Image& image, NaoCamera* naoCam, CameraSettings::CameraSettingCollection& settings, unsigned stamp);

  bool readCameraIntrinsics();
  bool readCameraResolution();

  bool processResolutionRequest();

  void setupCameras();
};
