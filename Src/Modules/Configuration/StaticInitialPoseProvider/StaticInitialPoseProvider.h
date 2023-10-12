/**
 * @file StaticInitialPoseProvider.h
 *
 * Provides static pose information from the config file
 *
 * @author Tim Haß
 * @author Nicole Schrader
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Representations/Infrastructure/GameState.h"

STREAMABLE(PoseVariation,
{,
  (std::vector<Pose2f>) poseVaria, /**< One pose variation > */
});

MODULE(StaticInitialPoseProvider,
{,
  REQUIRES(GameState),
  PROVIDES(StaticInitialPose),
  LOADS_PARAMETERS(
  {,
    (bool) isActive,     /**< Is this function activated > */
    (int) loadVariation, /**< Which variation should be loaded - starts at 0 > */
    (std::vector<PoseVariation>) poseVariations, /**< List of pose variations for testing > */
  }),
});

class StaticInitialPoseProvider : public StaticInitialPoseProviderBase
{
private:
  void update(StaticInitialPose& staticInitialPose) override;
  void setRobotsInSimulator(const Pose2f& staticInitialPose);
};
