#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/Streamable.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(ManualHeadMotionProvider)
  REQUIRES(CameraMatrix)
  REQUIRES(CameraInfo)
  PROVIDES_WITH_MODIFY(HeadMotionRequest)
  DEFINES_PARAMETER(int, xImg, 0)
  DEFINES_PARAMETER(int, yImg, 0)
  DEFINES_PARAMETER(CameraInfo, Camera, camera, lower)
END_MODULE

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
