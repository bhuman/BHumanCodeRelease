/**
 * @file GroundContactDetector.h
 * Declaration of a module that detects ground contact based on FSR measurements.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FsrData.h"
#include "Framework/Module.h"

MODULE(GroundContactDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FsrData),
  PROVIDES(GroundContactState),
  LOADS_PARAMETERS(
  {,
    (int) maxTimeWithoutPressure, /**< Maximum time allowed without minimum pressure before losing contact in ms. */
    (int) minTimeWithPressure, /**< Minimum time required with minimum pressure to regain contact in ms. */
    (int) minTimeSay, /**< Minimum interval in milliseconds to say something.. */
  }),
});

class GroundContactDetector : public GroundContactDetectorBase
{
private:
  unsigned lastTimeWithPressure = 0; /**< Last time when there was enough pressure while ground contact is still assumed (in ms). */
  unsigned lastTimeWithoutPressure = 0; /**< Last time when there wasn't enough pressure while ground contact is not yet assumed (in ms). */
  unsigned lastTimeSay = 0; /**< Last time when something was queued to say. */
  unsigned lastTimeGround = 0; /**< Last time when we wanted to say 'Ground'. */
  unsigned lastTimeHigh = 0; /**< Last time when we wanted to say 'High'. */

  void update(GroundContactState& groundContactState) override;
};
