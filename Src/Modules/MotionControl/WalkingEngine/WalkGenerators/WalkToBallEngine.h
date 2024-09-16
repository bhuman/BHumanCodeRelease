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
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Representations/MotionControl/WalkToPoseGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Framework/Module.h"

MODULE(WalkToBallEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkStepData),
  REQUIRES(WalkToPoseGenerator),
  PROVIDES(WalkToBallGenerator),
  DEFINES_PARAMETERS(
  {,
    (int)(750) minTimeSinceBallSeen, // 750 ms -> 3 walk steps
  }),
});

class WalkToBallEngine : public WalkToBallEngineBase
{
  RingBuffer<Angle, 2> lastExecutedStepRotation; /**< Step rotation of the last executed steps. */
  RingBuffer<Angle, 2> lastLeftOverRotation; /**< The last executed rotations if a zero step would have been executed. */
  Angle tempLastLeftOverRotation = 0_deg; /**< If a zero step would be executed now, this would be the executed step rotation. */
  unsigned lastWalkStepUpdate = 0; /**< Timestamp of last WalkStepData update. */
  float rotationReductionPerDirectionChange; /**< 1 / (Size of RingBuffer + 1) */

  void update(WalkToBallGenerator& walkToBallGenerator) override;

public:
  WalkToBallEngine();
};
