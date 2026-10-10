/**
 * @file SensorDataHistory.h
 * Declaration of struct SensorDataHistory.
 * It contains a history the most important motion representations,
 * so perception can use the best suitable data.
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Pose2f.h"
#include "Math/RingBuffer.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/MotionControl/OdometryData.h"

STREAMABLE(SensorDataHistory,
{
  STREAMABLE(SensorData,
  {,
    (JointAngles) jointAngles,
    (TorsoMatrix) torsoMatrix,
    (OdometryData) odometryData,
    (unsigned) timestamp,
  }),

  (std::vector<SensorData>) sensorData,
});
