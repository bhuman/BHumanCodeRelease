/**
 * @file StaticInitialPoseProvider.cpp
 *
 * Sets the positions from the config-file into the representation StaticInitialPose
 *
 * @author Tim Haß
 * @author Nicole Schrader
 */

#include "StaticInitialPoseProvider.h"

MAKE_MODULE(StaticInitialPoseProvider, infrastructure)

void StaticInitialPoseProvider::update(StaticInitialPose& staticInitialPose)
{
  DECLARE_DEBUG_RESPONSE("module:StaticInitialPoseProvider");
  MODIFY("module:StaticInitialPoseProvider:loadVariation", loadVariation);
  staticInitialPose.isActive = isActive;
  if(isActive && static_cast<size_t>(loadVariation) < poseVariations.size())
  {
    const PoseVariation& poses = poseVariations[loadVariation];
    staticInitialPose.staticPoseOnField = poses.poseVaria[theRobotInfo.number - 1];
    DEBUG_RESPONSE_ONCE("module:StaticInitialPoseProvider") setRobotsInSimulator(staticInitialPose);
  }
}

void StaticInitialPoseProvider::setRobotsInSimulator(const StaticInitialPose& staticInitialPose)
{
  const Vector2f& trans = staticInitialPose.staticPoseOnField.translation;
  const Angle rot = staticInitialPose.staticPoseOnField.rotation >= 0
                    ? staticInitialPose.staticPoseOnField.rotation - pi
                    : staticInitialPose.staticPoseOnField.rotation + pi;
  OUTPUT(idConsole, text, "mv " << -trans.x() << " " << -trans.y() << " 300 0 0 " << rot);
}
