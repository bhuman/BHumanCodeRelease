#include "ClearArmEngine.h"
#include "Tools/Motion/InverseKinematic.h"

MAKE_MODULE(ClearArmEngine, motionControl)

void ClearArmEngine::update(ClearArmEngineOutput& clearArmEngineOutput)
{
  updateArm(Arms::left, clearArmEngineOutput);
  updateArm(Arms::right, clearArmEngineOutput);
}

void ClearArmEngine::updateArm(Arms::Arm arm, ClearArmEngineOutput& clearArmEngineOutput)
{
  if(theArmMotionSelection.targetArmMotion[arm] == ArmMotionSelection::clearS)
    clearArm(arm, true, clearArmEngineOutput);

  else if(bit(theArmMotionSelection.targetArmMotion[arm]) & motionsToClear)
    clearMotion(arm, clearArmEngineOutput);

  else
  {
    motionWasCleaned[arm] = false;
    clearArmEngineOutput.providingOutput[arm] = false;
  }
}

void ClearArmEngine::clearArm(Arms::Arm arm, const bool clearOverLeg, ClearArmEngineOutput& clearArmEngineOutput)
{
  const Vector3f& nullPose = arm == Arms::left ? leftArmNullPosition : rightArmNullPosition;
  const Vector3f& knee = theRobotModel.limbs[arm == Arms::left ? Limbs::tibiaLeft : Limbs::tibiaRight].translation;

  const Vector3f nullKnee(0.f, arm == Arms::left ? theRobotDimensions.yHipOffset : -theRobotDimensions.yHipOffset, -theRobotDimensions.upperLegLength);

  const Vector3f kneeDiff = knee - nullKnee;
  const Vector3f newPose = nullPose + (clearOverLeg ? kneeDiff : Vector3f(0.f, arm == Arms::left ? 1.f : -1.f, 1.f).normalize(kneeDiff.norm()));

  InverseKinematic::calcShoulderJoints(arm, newPose, theRobotDimensions, theJointLimits, clearArmEngineOutput);
  clearArmEngineOutput.angles[Joints::combine(arm, Joints::elbowRoll)] = 0_rad;
  clearArmEngineOutput.angles[Joints::combine(arm, Joints::elbowYaw)] = 0_rad;
  clearArmEngineOutput.angles[Joints::combine(arm, Joints::wristYaw)] = arm == Arms::left ? -90_deg : 90_deg;
  clearArmEngineOutput.angles[Joints::combine(arm, Joints::hand)] = 0;

  for(unsigned i = Joints::combine(arm, Joints::JointArmVarieties(0)); i < Joints::combine(arm, Joints::numOfJointArmVarietiess); i++)
    clearArmEngineOutput.stiffnessData.stiffnesses[i] = stiffnesBySelfOutput;

  const Vector3f targetHandPostion = calcTargetHandPostition(arm, clearArmEngineOutput);
  const Vector3f handPosition = theRobotModel.limbs[Limbs::combine(arm, Limbs::foreArm)] * Vector3f(theRobotDimensions.lowerArmLength, 0.f, 0.f);

  clearArmEngineOutput.providingOutput[arm] = true;
  clearArmEngineOutput.cleared[arm] = (targetHandPostion - handPosition).squaredNorm() < squaredHandDisToBeCleared &&
                                      std::abs(clearArmEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] - theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)]) < shoulderPitchLRSideThreshholdVariance;
}

void ClearArmEngine::clearMotion(Arms::Arm arm, ClearArmEngineOutput& clearArmEngineOutput)
{
  if(theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)] < 0)
  {
    clearArmEngineOutput.providingOutput[arm] = false;
    return;
  }

  const JointRequest& inputJointRequest = getInputJointRequest(arm);
  const Vector3f targetHandPosition = calcTargetHandPostition(arm, inputJointRequest);

  const bool sameTarget = (lastTargetHandPos[arm] - targetHandPosition).squaredNorm() < squaredDisToStopCD;

  if(motionWasCleaned[arm])
  {
    if(sameTarget)
      return;
    else
      motionWasCleaned[arm] = false;
  }

  const Vector3f handPosition = theRobotModel.limbs[Limbs::combine(arm, Limbs::foreArm)] * Vector3f(theRobotDimensions.lowerArmLength, 0.f, 0.f);

  if(targetHandPosition.x() < handPosition.x()) //backwards motion
  {
    const Joints::Joint shoulderPitch = Joints::combine(arm, Joints::shoulderPitch);
    const bool isTargetNotSameSide = std::abs(inputJointRequest.angles[shoulderPitch] - shoulderPitchLRSideThreshhold) > shoulderPitchLRSideThreshholdVariance && theJointAngles.angles[shoulderPitch] < shoulderPitchLRSideThreshhold != inputJointRequest.angles[shoulderPitch] < shoulderPitchLRSideThreshhold;

    if(isTargetNotSameSide)
      clearArm(arm, false, clearArmEngineOutput);
    else if(!sameTarget)
      clearArmEngineOutput.providingOutput[arm] = false;
  }
  else // forward motion
  {
    const Vector3f& hibPosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::hip)].translation;
    const Vector3f& anklePosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::ankle)].translation;
    const Vector3f& shoulderPosition = theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation;
    const Vector3f& kneePosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::tibia)].translation;

    auto isLeftOfLine = [&](const Vector3f& start, const Vector3f& end, const Vector3f& point)
    {
      return ((end.x() - start.x()) * (point.y() - start.y()) - (end.y() - start.y()) * (point.x() - start.x())) > 0.f;
    };

    const bool isTargetNotSameLegSide =
      (shoulderPosition - kneePosition).squaredNorm() < (shoulderPosition - handPosition).squaredNorm() &&
      isLeftOfLine(hibPosition, anklePosition, targetHandPosition) != isLeftOfLine(hibPosition, anklePosition, handPosition);

    if(isTargetNotSameLegSide)
      clearArm(arm, true, clearArmEngineOutput);
    else if(!sameTarget)
      clearArmEngineOutput.providingOutput[arm] = false;
  }

  lastTargetHandPos[arm] = targetHandPosition;
  clearArmEngineOutput.cleared[arm] = (calcTargetHandPostition(arm, clearArmEngineOutput) - handPosition).squaredNorm() < squaredHandDisToBeCleared &&
                                      std::abs(clearArmEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] - theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)]) < shoulderPitchLRSideThreshholdVariance;

  if(clearArmEngineOutput.providingOutput[arm] && clearArmEngineOutput.cleared[arm])
  {
    motionWasCleaned[arm] = true;
    clearArmEngineOutput.providingOutput[arm] = false;
  }
}

Vector3f ClearArmEngine::calcTargetHandPostition(const Arms::Arm arm, const JointAngles& jointAngles) const
{
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f handPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                         .rotateY(jointAngles.angles[arm0 + 0])
                         .rotateZ(jointAngles.angles[arm0 + 1])
                         .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0)
                         .rotateX(jointAngles.angles[arm0 + 2])
                         .rotateZ(jointAngles.angles[arm0 + 3])
                         .translate(theRobotDimensions.lowerArmLength, 0, 0);
  return handPos.translation;
}

const JointRequest& ClearArmEngine::getInputJointRequest(const Arms::Arm arm) const
{
  switch(theArmMotionSelection.targetArmMotion[arm])
  {
    case ArmMotionSelection::keyPoseS:
      return theArmKeyPoseEngineOutput;
    default:
      FAIL("Unsupported arm motion");
      return assertedFalse;
  }
}
