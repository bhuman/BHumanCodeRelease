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
 */

#pragma once

#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Module.h"
#include "Tools/RobotParts/Legs.h"

MODULE(FootSupportProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FsrSensorData),
  PROVIDES(FootSupport),
  DEFINES_PARAMETERS(
  {,
    (float)(0.1f) minPressure, /**< Minimum pressure assumed. */
    (float)(5.0f) maxPressure, /**< Maximum pressure assumed. */
    (float)(0.8f) outerWeight, /**< Weights for outer FSRs. */
    (float)(0.3f) innerWeight, /**< Weights for inner FSRs. */
    (int)(10000) highestPressureUpdateTime, /**< Update the highestPressure after so much time is past. */
    (float)(0.5f) thresholdHighChangeWeight, /**< Current support foot weight minimum to detect a support foot switch if the change is fast. */
    (float)(0.45f) thresholdLowChangeWeight, /**< Current support foot weight minimum to detect a support foot switch if the change is slow. */
    (float)(0.3f) thresholdHighChangeVel, /**< Definition of a high support foot weight velocity. */
    (float)(0.15f) thresholdLowChangeVel, /**< Definition of a slow support foot weight velocity. */
    (float)(0.5f) resetMinMaxCheckThreshold, /**< Only do a prediction, if the weight on the support foot was at least this high since the last support switch. */
  }),
});

class FootSupportProvider : public FootSupportProviderBase
{
  float weights[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Weights for the individual FSRs. */
  float highestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured so far per FSR. */
  float newHighestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured in the last <highestPressureUpdateTime>/1000 seconds per FSR. */
  unsigned int updatePressureTimestamp; /** Timestamp of last highest pressure update. */
  float lastSupport; /** Last support value. */
  float lastLastSupport; /** Second last support value. */
  bool wasOverMaxThreshold; /** Was support value above a min positive value. */
  bool wasOverMinThreshold; /** Was support value above a min negative value. */

  void update(FootSupport& theFootSupport) override;

public:
  FootSupportProvider();
};
