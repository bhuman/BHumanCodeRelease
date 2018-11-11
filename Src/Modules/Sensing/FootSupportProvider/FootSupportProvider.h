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
#include "Tools/Module/Module.h"
#include "Tools/RobotParts/Legs.h"

MODULE(FootSupportProvider,
{,
  REQUIRES(FsrSensorData),
  PROVIDES(FootSupport),
  DEFINES_PARAMETERS(
  {,
    (float)(0.1f) minPressure, /**< Minimum pressure assumed. */
    (float)(5.0f) maxPressure, /**< Maximum pressure assumed. */
    (float)(0.8f) outerWeight, /**< Weights for outer FSRs. */
    (float)(0.3f) innerWeight, /**< Weights for inner FSRs. */
  }),
});

class FootSupportProvider : public FootSupportProviderBase
{
  float weights[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Weights for the individual FSRs. */
  float highestPressure[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Highest pressure measured so far per FSR. */

  void update(FootSupport& theFootSupport) override;

public:
  FootSupportProvider();
};
