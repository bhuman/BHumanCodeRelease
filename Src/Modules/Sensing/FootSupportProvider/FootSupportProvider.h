/**
 * @file FootSupportProvider.h
 *
 * This file declares a module that provides an abstract distribution of
 * how much each foot supports the weight of the robot.
 *
 * The code is based on parts of the class BodyModel from the code of the
 * team UNSW Australia.
 *
 * @author Thomas Röfer
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/EnumIndexedArray.h"

MODULE(FootSupportProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  USES(MotionInfo),
  REQUIRES(RobotDimensions),
  PROVIDES(FootSupport),
  LOADS_PARAMETERS(
  {,
    (Rangef) minPressureRange, /**< Min and Max value for minPressure. */
    (Rangef) minPressureInterpolationValues, /**< Min and Max interpolation range for minPressure, based on the max single foot sum pressure. */
    (float) minPressure, /**< Minimum pressure assumed. */
    (float) maxPressure, /**< Maximum pressure assumed. */
    (float) outerWeight, /**< Weights for outer FSRs. */
    (float) innerWeight, /**< Weights for inner FSRs. */
    (int) highestPressureUpdateTime, /**< Update the highestPressure after so much time is past. */
    (float) minSingleFSRPressureForPredictedSwitchFactor, /**< The forward and backward FSRs must measure the value "minPressure times this factor", to allow a foot support switch prediction. */
    (int) numOfSupportSwitches, /**< Update the lowest measured FSRs after this many foot support swithes.*/
    (float) maxTimeBetweenSupportSwitches, /**< Last step duration was lower than this max time. */
    (float) minTimeBetweenSupportSwitches, /**< Last step duration was higher than this min time. */
    (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, FsrSensors::FsrSensor), Legs::Leg)) lowestPressure, /**< Lowest overall pressure for each FSR sensor. */
  }),
});

class FootSupportProvider : public FootSupportProviderBase
{
  float weights[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Weights for the individual FSRs. */
  float highestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured so far per FSR. */
  float newHighestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured in the last <highestPressureUpdateTime>/1000 seconds per FSR. */

  float newLowestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Lowest pressure measured so far since the last update. */
  float maxFootSumPressure; /**< Sum of the last max measured pressures. */
  float maxFootPressureCurrent; /**< Current max measured pressure. */

  unsigned int updatePressureTimestamp; /**< Timestamp of last highest pressure update. */
  unsigned int lastSupportSwitch; /**< Timestamp of last support switch. */
  int supportSwitchCounter; /**< Number of foot support switches since the last update of the min pressure. */
  float lastSupport; /**< Last support value. */
  float lastSupportWithPressure; /**< Last support measurment, when the feet had enough pressure. */

  RingBuffer<bool, 4> leftFootPressureBuffer; /**< Ring buffer for the currents for every joint. */
  RingBuffer<bool, 4> rightFootPressureBuffer; /**< Ring buffer for the currents for every joint. */

  void update(FootSupport& theFootSupport) override;

public:
  FootSupportProvider();
};
