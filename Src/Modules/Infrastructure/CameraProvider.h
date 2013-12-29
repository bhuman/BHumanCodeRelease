/**
* @file CameraProvider.h
* This file declares a module that provides camera images.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#pragma once

class NaoCamera;

#include "Tools/Module/Module.h"
#include "Platform/Camera.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(CameraProvider)
  REQUIRES(CameraSettings)
  REQUIRES(Image)
  PROVIDES_WITH_OUTPUT(Image)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(FrameInfo)
  PROVIDES_WITH_MODIFY(CognitionFrameInfo)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(CameraInfo)
END_MODULE

class CameraProvider : public CameraProviderBase
{
private:
  static PROCESS_WIDE_STORAGE(CameraProvider) theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  NaoCamera* upperCamera;
  NaoCamera* lowerCamera;
  NaoCamera* currentImageCamera;
  CameraInfo upperCameraInfo;
  CameraInfo lowerCameraInfo;
  float cycleTime;
#ifdef CAMERA_INCLUDED
  unsigned int imageTimeStamp;
  unsigned int otherImageTimeStamp;
  unsigned int lastImageTimeStamp;
  unsigned long long lastImageTimeStampLL;
#endif

  void update(Image& image);
  void update(FrameInfo& frameInfo);
  void update(CognitionFrameInfo& cognitionFrameInfo);
  void update(CameraInfo& cameraInfo);

public:
  /**
  * Default constructor.
  */
  CameraProvider();

  /**
  * Destructor.
  */
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
};
