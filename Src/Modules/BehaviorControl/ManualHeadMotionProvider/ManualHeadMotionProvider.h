#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(ManualHeadMotionProvider,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  PROVIDES(HeadMotionRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(0) xImg,
    (int)(0) yImg,
    (CameraInfo::Camera)(CameraInfo::lower) camera,
  }),
});

class ManualHeadMotionProvider: public ManualHeadMotionProviderBase
{
public:
  /**
   * The update method to generate the head joint angles from desired head motion.
   */
  void update(HeadMotionRequest& headMotionRequest) override;

private:
  int currentX = 0;
  int currentY = 0;
};
