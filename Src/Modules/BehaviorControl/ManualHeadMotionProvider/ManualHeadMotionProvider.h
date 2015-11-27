#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/Perception/CameraMatrix.h"
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
    ((CameraInfo) Camera)(lower) camera,
  }),
});

class ManualHeadMotionProvider: public ManualHeadMotionProviderBase
{
public:
  ManualHeadMotionProvider();

  /**
   * The update method to generate the head joint angles from desired head motion.
   */
  void update(HeadMotionRequest& headMotionRequest);

private:
  int currentX;
  int currentY;
};
