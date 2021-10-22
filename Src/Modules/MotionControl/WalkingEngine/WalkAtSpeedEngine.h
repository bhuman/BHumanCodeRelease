/**
 * @file WalkAtSpeedEngine.h
 *
 * This file declares a module that provides walk generators.
 *
 * @author Arne Hasslebring
 */

#pragma once

#include "Representations/MotionControl/WalkAtAbsoluteSpeedGenerator.h"
#include "Representations/MotionControl/WalkAtRelativeSpeedGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Tools/Module/Module.h"

MODULE(WalkAtSpeedEngine,
{,
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkGenerator),
  PROVIDES(WalkAtAbsoluteSpeedGenerator),
  PROVIDES(WalkAtRelativeSpeedGenerator),
});

class WalkAtSpeedEngine : public WalkAtSpeedEngineBase
{
  void update(WalkAtAbsoluteSpeedGenerator& walkAtAbsoluteSpeedGenerator) override;
  void update(WalkAtRelativeSpeedGenerator& walkAtRelativeSpeedGenerator) override;

  /**
   * Creates a phase to walk with the requested velocity.
   * @param velocity The requested velocity (mm/s), is interpreted component-wise.
   * @param lastPhase The previous phase.
   * @return The new walk phase.
   */
  std::unique_ptr<MotionPhase> createPhase(const Pose2f& walkTarget, const Pose2f& walkSpeed, const MotionPhase& lastPhase) const;
};
