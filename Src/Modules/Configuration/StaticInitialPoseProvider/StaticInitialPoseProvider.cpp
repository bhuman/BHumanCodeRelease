/**
 * @file StaticInitialPoseProvider.cpp
 *
 * Sets the positions from the config-file into the representation StaticInitialPose
 *
 * @author Tim Haß
 * @author Nicole Schrader
 */

#include "StaticInitialPoseProvider.h"

MAKE_MODULE(StaticInitialPoseProvider, infrastructure);

void StaticInitialPoseProvider::update(StaticInitialPose& staticInitialPose)
{
  DECLARE_DEBUG_RESPONSE("module:StaticInitialPoseProvider");
  MODIFY("module:StaticInitialPoseProvider:loadVariation", loadVariation);
  staticInitialPose.isActive = isActive;
  if(isActive && static_cast<size_t>(loadVariation) < poseVariations.size())
  {
    const PoseVariation& poses = poseVariations[loadVariation];
    staticInitialPose.staticPoseOnField = poses.poseVaria[theRobotInfo.number - 1];
    staticInitialPose.jump = false;
    DEBUG_RESPONSE_ONCE("module:StaticInitialPoseProvider")
    {
      setRobotsInSimulator(poses.poseVaria[theRobotInfo.number - 1]);
      staticInitialPose.jump = true;
      ++loadVariation;
    }
  }
}

void StaticInitialPoseProvider::setRobotsInSimulator(const Pose2f& staticInitialPose)
{
  const Angle rotation = staticInitialPose.rotation >= 0
                    ? staticInitialPose.rotation - pi
                    : staticInitialPose.rotation + pi;
  OUTPUT(idConsole, text, "mv " << -staticInitialPose.translation.x() << " " << -staticInitialPose.translation.y() << " 300 0 0 " << rotation);
}
