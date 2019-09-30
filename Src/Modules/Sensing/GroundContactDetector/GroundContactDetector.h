/**
 * @file GroundContactDetector.h
 * Declaration of a module that detects ground contact based on FSR measurements.
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"

MODULE(GroundContactDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  REQUIRES(InertialSensorData),
  PROVIDES(GroundContactState),
  LOADS_PARAMETERS(
  {,
    (float) minPressureToKeepContact, /**< Minimum pressure on both feet for ground contact in kg. */
    (float) minPressureToRegainContact, /**< Minimum pressure on both feet to regain ground contact in kg. */
    (float) minPressurePerFootToRegainContact, /**< Minimum pressure on both feet to regain ground contact in kg. */
    (Angle) maxGyroYToRegainContact, /**< Maximum y gyro value considered as "not moving". */
    (int) maxTimeWithoutPressure, /**< Maxmimum time allowed without minimum pressure before losing contact in ms. */
    (int) minTimeWithPressure, /**< Minimum time required with minimum pressure to regain contact in ms. */
  }),
});

class GroundContactDetector : public GroundContactDetectorBase
{
private:
  unsigned lastTimeWithPressure = 0; /**< Last time when there was enough pressure while ground contact is still assumed (in ms). */
  unsigned lastTimeWithoutPressure = 0; /**< Last time when there wasn't enough pressure while ground contact is not yet assumed (in ms). */

  void update(GroundContactState& groundContactState) override;
};
