/**
 * @file RobotModelProvider.h
 *
 * This file declares a module that provides information about the current state of the robot's limbs.
 *
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#pragma once

#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"

MODULE(RobotModelProvider,
{,
  REQUIRES(JointAngles),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  PROVIDES(RobotModel),
});

/**
 * @class RobotModelProvider
 *
 * A module for computing the current state of the robot's limbs
 */
class RobotModelProvider: public RobotModelProviderBase
{
  /** Executes this module
   * @param robotModel The data structure that is filled by this module
   */
  void update(RobotModel& robotModel) override;
};
