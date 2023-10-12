/**
 * @file NaoQiImageDetector.h
 * This file declares a module that checks if the correct image was used to flash the robot.
 * Otherwise no sensor data can be read and all values are 0.
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/NaoQiImageInfo.h"
#include "Framework/Module.h"

MODULE(NaoQiImageDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(JointAngles),
  PROVIDES(NaoQiImageInfo),
  DEFINES_PARAMETERS(
  {,
    (unsigned int)(101000) startTime, /**< Start detection after this timestamp is reached. 1 second after software start. */
    (int)(20000) wrongImageTime, /**< Repeat warning sound every 10 seconds. */
    (Rangef)(-0.001f, 0.001f) valueRange, /**< Sensordata must be within this range to trigger the warning. */
  }
  ),
});

class NaoQiImageDetector : public NaoQiImageDetectorBase
{
  void update(NaoQiImageInfo& theNaoQiImageInfo) override;

  unsigned int lastSirene = 0;
};
