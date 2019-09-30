#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Pose3f.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/ArmKeyPoseEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Sensing/RobotModel.h"

MODULE(ArmKeyPoseEngine,
{,
  USES(MotionInfo),
  REQUIRES(ArmMotionSelection),
  REQUIRES(ArmMotionRequest),
  REQUIRES(JointAngles),
  REQUIRES(StiffnessSettings),
  REQUIRES(JointLimits),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  PROVIDES(ArmKeyPoseEngineOutput),
  LOADS_PARAMETERS(
  {,
    (std::vector<Pose3f>) allLeftPoses,
    (std::vector<Pose3f>) allRightPoses,
    (float)(1000.f) squardDisToSwitchFrame,
    (int)(100) stiffness,
  }),
});

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
class ArmKeyPoseEngine : public ArmKeyPoseEngineBase
{
public:
  void update(ArmKeyPoseEngineOutput& armKeyPoseEngineOutput) override;

  ArmKeyPoseEngine();

private:
  const ArmKeyPoseRequest& theArmKeyPoseRequest = theArmMotionRequest.armKeyPoseRequest;
  Pose3f allPoses[Arms::numOfArms][ArmKeyPoseRequest::numOfArmKeyPoseIds];
  ArmKeyPoseRequest::ArmKeyPoseMotionId lastMotionId[Arms::numOfArms] = { ArmKeyPoseRequest::nullPose, ArmKeyPoseRequest::nullPose};
  int lastMotionFrame[Arms::numOfArms] = {1, 1};

  void updateArm(Arms::Arm arm, ArmKeyPoseEngineOutput& armKeyPoseEngineOutput);
  const Pose3f& getPose(Arms::Arm arm, ArmKeyPoseEngineOutput& armKeyPoseEngineOutput);
};
