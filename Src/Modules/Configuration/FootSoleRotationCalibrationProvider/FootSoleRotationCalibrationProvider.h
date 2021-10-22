/**
 * @file FootSoleRotationCalibrationProvider.h
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations//Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Module.h"

MODULE(FootSoleRotationCalibrationProvider,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(FootSupport),
  REQUIRES(InertialSensorData),
  REQUIRES(InertialData),
  REQUIRES(GyroState),
  REQUIRES(MotionInfo),
  REQUIRES(FrameInfo),
  REQUIRES(RobotModel),
  USES(FootSoleRotationCalibration),
  USES(MotionRequest),
  PROVIDES(FootSoleRotationCalibration),
  DEFINES_PARAMETERS(
  {,
    (Angle)(0.2_deg) maxTorsoDifference,
    (Angle)(2_deg) maxOffsetDifference,
    (Angle)(0.2_deg) maxGyroMean,
    (Angle)(0.1_deg) maxLeftRightFootRotationDifference,
  }),
});

class FootSoleRotationCalibrationProvider : public FootSoleRotationCalibrationProviderBase
{
private:

  unsigned serialNumbeFootSoleRotationCalibration = 0;
  int usedMeasurements = 0;
  void update(FootSoleRotationCalibration& footSoleRotationCalibration) override;

public:
  FootSoleRotationCalibrationProvider();
};
