/**
 * @file StaticInitialPoseProvider.cpp
 *
 * Sets the positions from the configuration file into the representation StaticInitialPose
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
    ASSERT(poseVariations.size() > 0);
    const PoseVariation& poses = poseVariations[loadVariation];
    staticInitialPose.staticPoseOnField = poses.poseVaria[theGameState.playerNumber - Settings::lowestValidPlayerNumber];
    staticInitialPose.jump = false;
    DEBUG_RESPONSE_ONCE("module:StaticInitialPoseProvider")
    {
      setRobotsInSimulator(poses.poseVaria[theGameState.playerNumber - Settings::lowestValidPlayerNumber]);
      staticInitialPose.jump = true;
      loadVariation = (loadVariation + 1) % poseVariations.size();
    }
  }
}

void StaticInitialPoseProvider::setRobotsInSimulator([[maybe_unused]] const Pose2f& staticInitialPose)
{
  OUTPUT(idConsole, text, "mv " << staticInitialPose.translation.x() << " " << staticInitialPose.translation.y() << " "
         << theRobotDimensions.simOriginHeight << " 0 0 " << staticInitialPose.rotation);
}
