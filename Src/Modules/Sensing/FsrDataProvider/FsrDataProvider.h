/**
 * @file FsrDataProvider.h
 * This modules calibrates the fsr sensor data
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/FootSupport.h"

MODULE(FsrDataProvider,
{,
  USES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  USES(MotionInfo),
  PROVIDES(FsrData),
  LOADS_PARAMETERS(
  {,
    (Rangef) minPressure, /**< Minimum pressure assumed. */
    (Rangef) badMinPressureTimewindow,
    (float) minPressurePercent,
    (float) maxPressure, /**< Maximum pressure assumed. */
    (Rangef) minPressureRange, /**< Min and Max value for minPressure. */
    (Rangef) minPressureInterpolationValues, /**< Min and Max interpolation range for minPressure, based on the max single foot sum pressure. */
    (int) numOfSupportSwitches, /**< Update the lowest measured FSRs after this many foot support switches.*/
    (float) maxTimeBetweenSupportSwitches, /**< Last step duration was lower than this max time. */
    (float) minTimeBetweenSupportSwitches, /**< Last step duration was higher than this min time. */
    (float) maxTimeLegNoPressureForCalibration, /**< A leg is only allowed to have max this time no pressure before the next calibration is postponed. */
    (int) highestPressureUpdateTime, /**< Update the highestPressure after so much time is past. */
    (float) minSingleFSRPressureForPredictedSwitchFactor, /**< The forward and backward FSRs must measure the value "minPressure times this factor", to allow a foot support switch prediction. */
    (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, FsrSensors::FsrSensor), Legs::Leg)) lowestPressure, /**< Current min measured pressure. */
    (bool) isCalibrated, /**< Is the FsrData calibrated? */
  }),
});

class FsrDataProvider : public FsrDataProviderBase
{
  void update(FsrData& theFsrData) override;

  float highestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured so far per FSR. */
  float newHighestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured in the last <highestPressureUpdateTime>/1000 seconds per FSR. */

  float newLowestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Lowest pressure measured so far since the last update. */
  float maxFootSumPressure = 0.f; /**< Sum of the last max measured pressures. */
  float maxFootPressureCurrent; /**< Current max measured pressure. */

  float highestLegSumPressure[Legs::numOfLegs];
  float newHighestLegSumPressure[Legs::numOfLegs];

  int supportSwitchCounter = 0; /**< Number of foot support switches since the last update of the min pressure. */
  unsigned int lastSupportSwitch = 0; /**< Timestamp of last support switch. */
  unsigned int updatePressureTimestamp = 0; /**< Timestamp of last highest pressure update. */

  unsigned int originalHasPressure[Legs::numOfLegs];

public:
  FsrDataProvider();
};
