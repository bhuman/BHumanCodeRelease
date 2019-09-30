
#include "ArmKeyPoseEngine.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/RobotParts/Limbs.h"

MAKE_MODULE(ArmKeyPoseEngine, motionControl)

ArmKeyPoseEngine::ArmKeyPoseEngine() : ArmKeyPoseEngineBase()
{
  ASSERT(theArmKeyPoseRequest.numOfArmKeyPoseIds == allLeftPoses.size()
         && theArmKeyPoseRequest.numOfArmKeyPoseIds == allRightPoses.size());

  for(unsigned id = 0; id < theArmKeyPoseRequest.numOfArmKeyPoseIds; id++)
  {
    allPoses[Arms::left][id] = allLeftPoses.at(id);
    allPoses[Arms::right][id] = allRightPoses.at(id);
  }
}

void ArmKeyPoseEngine::update(ArmKeyPoseEngineOutput& armKeyPoseEngineOutput)
{
  updateArm(Arms::left, armKeyPoseEngineOutput);
  updateArm(Arms::right, armKeyPoseEngineOutput);
}

void ArmKeyPoseEngine::updateArm(Arms::Arm arm, ArmKeyPoseEngineOutput& armKeyPoseEngineOutput)
{
  if(theArmMotionSelection.targetArmMotion[arm] != ArmMotionSelection::keyPoseS)
    return;

  const Pose3f& targetPose = getPose(arm, armKeyPoseEngineOutput);
  VERIFY(0u == InverseKinematic::calcArmJoints(arm, targetPose, theRobotDimensions, theJointLimits, armKeyPoseEngineOutput));

  //---------------------------FIXME-------------
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f handPosOutput = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                               .rotateY(armKeyPoseEngineOutput.angles[arm0 + 0])
                               .rotateZ(armKeyPoseEngineOutput.angles[arm0 + 1])
                               .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0)
                               .rotateX(armKeyPoseEngineOutput.angles[arm0 + 2])
                               .rotateZ(armKeyPoseEngineOutput.angles[arm0 + 3])
                               .translate(theRobotDimensions.lowerArmLength, 0, 0);

  const Pose3f targetInOutput = handPosOutput.inverse() * targetPose;
  //----------------------------------------------------

  armKeyPoseEngineOutput.angles[Joints::combine(arm, Joints::wristYaw)] = targetInOutput.rotation.getZAngle();//arm == Arms::left ? -90_deg : 90_deg;
  armKeyPoseEngineOutput.angles[Joints::combine(arm, Joints::hand)] = 0;// 1;

  for(unsigned joint = Joints::combine(arm, Joints::JointArmVarieties(0)); joint <= Joints::combine(arm, Joints::hand); joint++)
    armKeyPoseEngineOutput.stiffnessData.stiffnesses[joint] = stiffness;

  lastMotionId[arm] = theArmKeyPoseRequest.arms[arm].motion;
}

const Pose3f& ArmKeyPoseEngine::getPose(Arms::Arm arm, ArmKeyPoseEngineOutput& armKeyPoseEngineOutput)
{
  if(theArmKeyPoseRequest.arms[arm].motion != lastMotionId[arm])
    lastMotionFrame[arm] = 1;

  if(theArmKeyPoseRequest.arms[arm].motion < ArmKeyPoseRequest::numOfArmKeyPoseIds)
  {
    armKeyPoseEngineOutput.arms[arm].pose = theArmKeyPoseRequest.arms[arm].motion;
    return allPoses[arm][theArmKeyPoseRequest.arms[arm].motion];
  }

  const Pose3f handPos = theRobotModel.limbs[arm == Arms::left ? Limbs::wristLeft : Limbs::wristRight].translated(theRobotDimensions.handOffset.x(), 0, 0);

  switch(theArmKeyPoseRequest.arms[arm].motion)
  {
    case ArmKeyPoseRequest::wave:
    {
      static const std::vector<ArmKeyPoseRequest::ArmKeyPoseMotionId> usedPosIds({ ArmKeyPoseRequest::outerWavePos, ArmKeyPoseRequest::innerWavePos });
      if((handPos.translation - allPoses[arm][usedPosIds[lastMotionFrame[arm]]].translation).squaredNorm() < squardDisToSwitchFrame)
      {
        lastMotionFrame[arm] += 1;
        lastMotionFrame[arm] %= usedPosIds.size();
      }
      armKeyPoseEngineOutput.arms[arm].pose = usedPosIds[lastMotionFrame[arm]];
      return allPoses[arm][usedPosIds[lastMotionFrame[arm]]];
    }
    default:
      FAIL("Unsupported arm motion.");
      return allPoses[arm][ArmKeyPoseRequest::nullPose];
  }
}
