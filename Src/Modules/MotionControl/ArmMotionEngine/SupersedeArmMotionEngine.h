#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/MotionControl/ArmKeyPoseEngineOutput.h"
#include "Representations/MotionControl/ArmMotionSelection.h"
#include "Representations/MotionControl/ClearArmEngineOutput.h"
#include "Representations/MotionControl/SupersedeArmMotionEngineOutput.h"
#include "Representations/Sensing/BodyBoundary.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(SupersedeArmMotionEngine,
{,
  REQUIRES(ArmMotionSelection),
  REQUIRES(ArmKeyPoseEngineOutput),
  REQUIRES(ClearArmEngineOutput),
  REQUIRES(RobotDimensions),
  REQUIRES(JointLimits),
  REQUIRES(JointAngles),
  REQUIRES(BodyBoundary),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(SupersedeArmMotionEngineOutput),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(bit(ArmMotionSelection::keyPoseS) | bit(ArmMotionSelection::clearS)) motionsToSupersede,

    (float)(0.5f) minShoulderPortionOfLowerArmPotential,
    (float)(0.5f) defaultSpeedPercentate,
    (float)(0.2f) refSpeed,

    (float)(70.f) maxDisToFeelGoundPotential,

    (Angle)(90_deg) shoulderPitchLRSideThreshhold,
    (Angle)(5_deg) shoulderPitchLRSideThreshholdVariance,
    (float)(0.2f) constLRSidePenalty,
    (float)(0.4f) addLRSidePenaltyByFalseBFSide,
    (float)(50.f) minDisForAddSidePenalty,
    (float)(200.f) minDisForMaxAddSidePenalty,
    (float)(0.3f) maxAddSidePenaltyByDis,

    (float)(10.f) additionalDistancsToHold,
    (float)(90.f) maxDistanceToFeelObjects,

    (bool)(true) handleLooseJoints,
    (Angle)(20_deg) offsetStopSupersedeByLooseJointsHandling,

    (float)(0.0001f) smallNumber,
  }),
});

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
class SupersedeArmMotionEngine : public SupersedeArmMotionEngineBase
{
public:
  void update(SupersedeArmMotionEngineOutput& supersedeOutput) override;

private:
  JointRequest assertedFalse;
  void updateArm(Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput);
  const JointRequest& getInputJointRequest(const Arms::Arm arm) const;

  void interpolate(const Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput) const;

  Vector3f calcGroundPotential(const Vector3f& onPointInRobot) const;
  float calcSidePotential(const Arms::Arm arm, const float disToTarget, const SupersedeArmMotionEngineOutput& supersedeOutput, const Vector3f& handPosition, const Vector3f& targetHandPosition) const;

  void calcPotentialOnLowerArm(const Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput) const;
  inline bool shouldSupersede(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const;
  inline Vector3f calcTargetHandPostition(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const;
  inline Vector3f calcTargetElbowPosition(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const;
  inline Vector3f calcHandPostition(const Arms::Arm arm, const Angle& elbowRoll) const;
  inline Vector3f calcElbowPosition(const Arms::Arm arm, const Angle& shoulderRoll) const;
};
