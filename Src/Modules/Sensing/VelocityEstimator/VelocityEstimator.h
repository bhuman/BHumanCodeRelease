/**
 * @file VelocityEstimator.h
 * Declaration of module VelocityEstimator.
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/VelocityEstimation.h"

MODULE(VelocityEstimator,
{,
  REQUIRES(OdometryDataPreview),
  PROVIDES(VelocityEstimation),
});

/**
 * @class VelocityEstimator
 * A module that provides the (estimated) position and velocity of the inertia board.
 */
class VelocityEstimator : public VelocityEstimatorBase
{
private:
  RingBuffer<Pose2f, 125> lastEstimations;
  void update(VelocityEstimation& theVelocityEstimation) override;
};
