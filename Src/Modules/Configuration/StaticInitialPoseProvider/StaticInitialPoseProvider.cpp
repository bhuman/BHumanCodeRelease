/**
 * @file StaticInitialPoseProvider.cpp
 *
 * Sets the positions from the config-file into the representation StaticInitialPose
 *
 * @author Tim Haß
 * @author Nicole Schrader
 */

#include "StaticInitialPoseProvider.h"

MAKE_MODULE(StaticInitialPoseProvider);

void StaticInitialPoseProvider::update(StaticInitialPose& staticInitialPose)
{
  DECLARE_DEBUG_RESPONSE("module:StaticInitialPoseProvider");
  MODIFY("module:StaticInitialPoseProvider:loadVariation", loadVariation);
  staticInitialPose.isActive = isActive;
  if(isActive && static_cast<size_t>(loadVariation) < poseVariations.size())
  {
    const PoseVariation& poses = poseVariations[loadVariation];
    staticInitialPose.staticPoseOnField = poses.poseVaria[theGameState.playerNumber - Settings::lowestValidPlayerNumber];
    staticInitialPose.jump = false;
    DEBUG_RESPONSE_ONCE("module:StaticInitialPoseProvider")
    {
      setRobotsInSimulator(poses.poseVaria[theGameState.playerNumber - Settings::lowestValidPlayerNumber]);
      staticInitialPose.jump = true;
      ++loadVariation;
    }
  }
}

void StaticInitialPoseProvider::setRobotsInSimulator([[maybe_unused]] const Pose2f& staticInitialPose)
{
  OUTPUT(idConsole, text, "mv " << staticInitialPose.translation.x() << " " << staticInitialPose.translation.y() << " 300 0 0 " << staticInitialPose.rotation);
}
