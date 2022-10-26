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

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"
#include "Math/RingBuffer.h"
#include "RobotParts/Legs.h"
#include "Streaming/EnumIndexedArray.h"

MODULE(FootSupportProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FsrData),
  USES(MotionInfo),
  REQUIRES(RobotDimensions),
  PROVIDES(FootSupport),
  LOADS_PARAMETERS(
  {,
    (float) maxPressure, /**< Maximum pressure assumed. */
    (float) outerWeight, /**< Weights for outer FSRs. */
    (float) innerWeight, /**< Weights for inner FSRs. */
  }),
});

class FootSupportProvider : public FootSupportProviderBase
{
  void update(FootSupport& theFootSupport) override;

  float weights[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Weights for the individual FSRs. */
  float lastSupport; /**< Last support value. */
  float lastSupportWithPressure; /**< Last support measurment, when the feet had enough pressure. */
  unsigned int lastSupportSwitch = 0; /**< Timestamp of last support switch. */

public:
  FootSupportProvider();
};
