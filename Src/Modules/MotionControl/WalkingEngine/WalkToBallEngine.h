/**
 * @file WalkToBallEngine.h
 *
 * This file declares a module that provides a walk to ball engine.
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"

MODULE(WalkToBallEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FootOffset),
  REQUIRES(RobotModel),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkToPoseGenerator),
  PROVIDES(WalkToBallGenerator),
});

class WalkToBallEngine : public WalkToBallEngineBase
{
  void update(WalkToBallGenerator& walkToBallGenerator) override;
};
