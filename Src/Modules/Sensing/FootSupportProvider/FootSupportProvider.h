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
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/SolePressureState.h"
#include "Framework/Module.h"
#include "RobotParts/Legs.h"

MODULE(FootSupportProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(SolePressureState),
  PROVIDES(FootSupport),
  LOADS_PARAMETERS(
  {,
    (float) outerWeight, /**< Weights for outer FSRs. */
    (float) innerWeight, /**< Weights for inner FSRs. */
    (float) currentSupportMaxPressure, /**< Max support % weight to allow a support switch prediction. */
    (float) currentSwingMinPressure, /**< Min swing % weight to allow a support switch prediction. */
    (float) predictionFactor, /**< How many frames into the future we want to predict. */
    (float) minSingleFSRPressureForPredictedSwitchFactor, /**< The forward and backward FSRs must measure the value "minPressure times this factor", to allow a foot support switch prediction. */
  }),
});

class FootSupportProvider : public FootSupportProviderBase
{
  void update(FootSupport& theFootSupport) override;

  float weights[Legs::numOfLegs][FsrSensors::numOfFsrSensors]; /**< Weights for the individual FSRs. */
  float lastSupport; /**< Last support value. */
  float lastSupportWithPressure; /**< Last support measurement, when the feet had enough pressure. */
  unsigned int lastSupportSwitch = 0; /**< Timestamp of last support switch. */

public:
  FootSupportProvider();
};
