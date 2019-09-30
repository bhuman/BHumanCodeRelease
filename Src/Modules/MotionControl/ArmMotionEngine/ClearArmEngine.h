#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/ClearArmEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/ArmKeyPoseEngineOutput.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"

MODULE(ClearArmEngine,
{,
  REQUIRES(JointAngles),
  REQUIRES(ArmKeyPoseEngineOutput),
  REQUIRES(ArmMotionSelection),
  REQUIRES(JointLimits),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  PROVIDES(ClearArmEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(bit(ArmMotionSelection::keyPoseS)) motionsToClear,
    (int)(80.f) stiffnesBySelfOutput,
    (Vector3f)(Vector3f(-4.f, 156.f, -26.f)) leftArmNullPosition,
    (Vector3f)(Vector3f(-4.f, -156.f, -26.f)) rightArmNullPosition,
    (float)(sqr(40)) squaredHandDisToBeCleared,
    (Angle)(3_deg) shoulderPitchLRSideThreshholdVariance,
    (Angle)(90_deg) shoulderPitchLRSideThreshhold,
    (float)(sqr(50)) squaredDisToStopCD,
  }),
});

/**
 * Module that provides a motion which moves the arm to a position to enable easy execution of any
 * other arm motion action.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
class ClearArmEngine : public ClearArmEngineBase
{
public:
  void update(ClearArmEngineOutput& clearArmEngineOutput) override;

private:
  JointRequest assertedFalse;
  void updateArm(Arms::Arm arm, ClearArmEngineOutput& clearArmEngineOutput);
  void clearArm(Arms::Arm arm, const bool clearOverLeg, ClearArmEngineOutput& clearArmEngineOutput);
  void clearMotion(Arms::Arm arm, ClearArmEngineOutput& clearArmEngineOutput);
  bool motionWasCleaned[Arms::numOfArms] = { false, false };
  Vector3f lastTargetHandPos[Arms::numOfArms];

  Vector3f calcTargetHandPostition(const Arms::Arm arm, const JointAngles& jointAngles) const;
  const JointRequest& getInputJointRequest(const Arms::Arm arm) const;
};
