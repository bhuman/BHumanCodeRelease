/**
 * @file BoosterFsrSensorDataProvider.h
 *
 * This file declares a module that computes fake FSR readings
 * from the deviations of the joints.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/ModifiedJointRequest.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Math/RingBuffer.h"

MODULE(BoosterFsrSensorDataProvider,
{,
  REQUIRES(JointAngles),
  REQUIRES(JointPlay),
  USES(JointRequest),
  REQUIRES(MassCalibration),
  USES(ModifiedJointRequest),
  PROVIDES(FsrSensorData),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(7) motionDelay, /**< The delay between requests and measurements in number of frames. */
    (Rangea)(0_deg, 1_deg) baseRange, /**< The minimum range for accumulated deviations. */
    (Angle)(3.5_deg) maxAngle, /**< The assumed maximum deviations of ankle joints. */
  }),
});

class BoosterFsrSensorDataProvider : public BoosterFsrSensorDataProviderBase
{
  RingBuffer<std::array<Angle, Joints::numOfJoints>> buffer; /**< Buffer for requested joint offsets. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFsrSensorData The representation updated.
   */
  void update(FsrSensorData& theFsrSensorData) override;

public:
  BoosterFsrSensorDataProvider() {buffer.reserve(motionDelay);}
};
