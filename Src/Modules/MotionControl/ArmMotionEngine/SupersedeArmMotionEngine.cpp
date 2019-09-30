
#include "SupersedeArmMotionEngine.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/RobotParts/Arms.h"
#include "Tools/RobotParts/Legs.h"

MAKE_MODULE(SupersedeArmMotionEngine, motionControl)

void SupersedeArmMotionEngine::update(SupersedeArmMotionEngineOutput& output)
{
  updateArm(Arms::left, output);
  updateArm(Arms::right, output);
}

void SupersedeArmMotionEngine::updateArm(Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput)
{
  if(!(bit(theArmMotionSelection.targetArmMotion[arm]) & motionsToSupersede))
    return;

  const JointRequest& inputJointRequest = getInputJointRequest(arm);
  for(unsigned i = Joints::combine(arm, Joints::JointArmVarieties(0)); i < Joints::combine(arm, Joints::numOfJointArmVarietiess); i++)
    supersedeOutput.angles[i] = inputJointRequest.angles[i];

  if(shouldSupersede(arm, supersedeOutput))
    calcPotentialOnLowerArm(arm, supersedeOutput);

  interpolate(arm, supersedeOutput);

  for(unsigned i = Joints::combine(arm, Joints::JointArmVarieties(0)); i < Joints::combine(arm, Joints::numOfJointArmVarietiess); i++)
    supersedeOutput.stiffnessData.stiffnesses[i] = inputJointRequest.stiffnessData.stiffnesses[i];
}

inline bool SupersedeArmMotionEngine::shouldSupersede(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const
{
  if(handleLooseJoints)
  {
    for(unsigned i = Joints::combine(arm, Joints::JointArmVarieties(0)); i < Joints::combine(arm, Joints::wristYaw); i++)
      if(std::abs(supersedeOutput.angles[i] - theJointAngles.angles[i]) > offsetStopSupersedeByLooseJointsHandling)
        return true;
    return false;
  }
  return true;
}

const JointRequest& SupersedeArmMotionEngine::getInputJointRequest(Arms::Arm arm) const
{
  if(theClearArmEngineOutput.providingOutput[arm])
    return theClearArmEngineOutput;

  switch(theArmMotionSelection.targetArmMotion[arm])
  {
    case ArmMotionSelection::clearS:
      return theClearArmEngineOutput;
    case ArmMotionSelection::keyPoseS:
      return theArmKeyPoseEngineOutput;
    default:
      FAIL("Unsupported arm motion.");
      return assertedFalse;
  }
}

void SupersedeArmMotionEngine::interpolate(const Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput) const
{
  for(unsigned i = Joints::combine(arm, Joints::JointArmVarieties(0)); i < Joints::combine(arm, Joints::numOfJointArmVarietiess); i++)
    supersedeOutput.angles[i] = (std::abs(supersedeOutput.angles[i] - theJointAngles.angles[i]) < defaultSpeedPercentate * refSpeed) ? supersedeOutput.angles[i] : Angle((supersedeOutput.angles[i] < theJointAngles.angles[i]) ? theJointAngles.angles[i] - defaultSpeedPercentate* refSpeed : theJointAngles.angles[i] + defaultSpeedPercentate * refSpeed);
}

Vector3f SupersedeArmMotionEngine::calcTargetElbowPosition(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const
{
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f elbowPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                          .rotateY(supersedeOutput.angles[arm0 + 0])
                          .rotateZ(supersedeOutput.angles[arm0 + 1])
                          .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0);
  return elbowPos.translation;
}

Vector3f SupersedeArmMotionEngine::calcElbowPosition(const Arms::Arm arm, const Angle& shouldeRoll) const
{
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f elbowPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                          .rotateY(theJointAngles.angles[arm0 + 0])
                          .rotateZ(shouldeRoll)
                          .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0);
  return elbowPos.translation;
}

Vector3f SupersedeArmMotionEngine::calcTargetHandPostition(const Arms::Arm arm, const SupersedeArmMotionEngineOutput& supersedeOutput) const
{
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f handPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                         .rotateY(supersedeOutput.angles[arm0 + 0])
                         .rotateZ(supersedeOutput.angles[arm0 + 1])
                         .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0)
                         .rotateX(supersedeOutput.angles[arm0 + 2])
                         .rotateZ(supersedeOutput.angles[arm0 + 3])
                         .translate(theRobotDimensions.lowerArmLength, 0, 0);
  return handPos.translation;
}

Vector3f SupersedeArmMotionEngine::calcHandPostition(const Arms::Arm arm, const Angle& elbowRoll) const
{
  const int sign = arm == Arms::left ? 1 : -1;
  const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

  const Pose3f handPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                         .rotateY(theJointAngles.angles[arm0 + 0])
                         .rotateZ(theJointAngles.angles[arm0 + 1])
                         .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0)
                         .rotateX(theJointAngles.angles[arm0 + 2])
                         .rotateZ(elbowRoll)
                         .translate(theRobotDimensions.lowerArmLength, 0, 0);
  return handPos.translation;
}

Vector3f SupersedeArmMotionEngine::calcGroundPotential(const Vector3f& onPointInRobot) const
{
  const Vector3f pointInGround = theTorsoMatrix * onPointInRobot;
  const float potential = sqr((maxDisToFeelGoundPotential - std::min(maxDisToFeelGoundPotential, pointInGround.z())) / maxDisToFeelGoundPotential);
  return theTorsoMatrix.rotation.inverse() * Vector3f(0.f, 0.f, potential);
}

float SupersedeArmMotionEngine::calcSidePotential(const Arms::Arm arm, const float disToTarget, const SupersedeArmMotionEngineOutput& supersedeOutput,
                                                  const Vector3f& handPosition, const Vector3f& targetHandPosition) const
{
  const Joints::Joint shoulderPitch = Joints::combine(arm, Joints::shoulderPitch);
  const float sign = arm == Arms::left ? 1.f : -1.f;

  const Vector3f& hibPosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::hip)].translation;
  const Vector3f& anklePosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::ankle)].translation;
  const Vector3f& shoulderPosition = theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].translation;
  const Vector3f& kneePosition = theRobotModel.limbs[Limbs::combine(Legs::Leg(arm), Limbs::tibia)].translation;

  auto isLeftOfLine = [&](const Vector3f& start, const Vector3f& end, const Vector3f& point)
  {
    return ((end.x() - start.x()) * (point.y() - start.y()) - (end.y() - start.y()) * (point.x() - start.x())) > 0.f;
  };

  const bool isTargetNotSameLegSide = (shoulderPosition - kneePosition).squaredNorm() < (shoulderPosition - handPosition).squaredNorm() && isLeftOfLine(hibPosition, anklePosition, targetHandPosition) != isLeftOfLine(hibPosition, anklePosition, handPosition);
  const bool isTargetNotSameSide = std::abs(supersedeOutput.angles[shoulderPitch] - shoulderPitchLRSideThreshhold) > shoulderPitchLRSideThreshholdVariance && theJointAngles.angles[shoulderPitch] < shoulderPitchLRSideThreshhold != supersedeOutput.angles[shoulderPitch] < shoulderPitchLRSideThreshhold;
  const float disPotential = (std::max(minDisForAddSidePenalty, std::min(disToTarget, minDisForMaxAddSidePenalty)) - minDisForAddSidePenalty) / (minDisForMaxAddSidePenalty - minDisForAddSidePenalty);

  const float potential = sign * (constLRSidePenalty + int(isTargetNotSameSide || isTargetNotSameLegSide) * addLRSidePenaltyByFalseBFSide) + disPotential * maxAddSidePenaltyByDis;
  return potential;
}

void SupersedeArmMotionEngine::calcPotentialOnLowerArm(Arms::Arm arm, SupersedeArmMotionEngineOutput& supersedeOutput) const
{
  const Capsule& lowerArm = arm == Arms::left ? theBodyBoundary.foreArmLeftVal : theBodyBoundary.foreArmRightVal;
  const Pose3f& elbowPose = theRobotModel.limbs[Limbs::combine(arm, Limbs::foreArm)];
  const Vector3f targetElbowPosition = calcTargetElbowPosition(arm, supersedeOutput);

  const Vector3f handPostition = theRobotModel.limbs[Limbs::combine(arm, Limbs::foreArm)] * Vector3f(theRobotDimensions.lowerArmLength, 0.f, 0.f);
  const Vector3f targetHandPostion = calcTargetHandPostition(arm, supersedeOutput);
  const float distanceToTarget = (handPostition - targetHandPostion).norm();

  //const Vector3f groundPotenialOnHand = calcGroundPotential(handPostition);
  const float sidePotential = calcSidePotential(arm, distanceToTarget, supersedeOutput, handPostition, targetHandPostion);

  const Vector3f sidePotentialOnElbow = (calcElbowPosition(arm, pi_2 + theJointAngles.angles[Joints::combine(arm, Joints::shoulderRoll)]) - elbowPose.translation).normalized(sidePotential);
  //const Vector3f sidePotentialOnHand = (calcHandPostition(arm, refSpeed + theJointAngles.angles[Joints::combine(arm, Joints::elbowRoll)]) - elbowPose.translation).normalized(sidePotential);

  const Vector3f targetElbowPotential = (targetElbowPosition - elbowPose.translation).normalized();
  //  const Vector3f targetHandPotential = (targetHandPostion - handPostition).normalized();

  float maximalPotential = 0.f;
  Vector3f potentialDirection = Vector3f::Zero();

  auto addPotentialVector = [&](const BodyBoundary::BodyObject eObject)
  {
    Geometry::LineSegment3D ls;
    lowerArm.isIntersectingWith(*theBodyBoundary.objects[eObject], ls);
    const Vector3f directionOfPotential = (ls.P0 - ls.P1);
    const float rawPotential = (maxDistanceToFeelObjects - (directionOfPotential.norm() - additionalDistancsToHold - lowerArm.radius - theBodyBoundary.objects[eObject]->radius)) / maxDistanceToFeelObjects;
    const float potential = rawPotential < 0.f ? 0.f : sqr(rawPotential);

    maximalPotential = std::max(potential, maximalPotential);
    potentialDirection += directionOfPotential.normalized(potential);
  };

  for(unsigned eObject = BodyBoundary::firstNonArmObject; eObject < BodyBoundary::numOfBodyObjects; eObject++)
    addPotentialVector(BodyBoundary::BodyObject(eObject));

  const Vector3f potentialByObjectsOnElbow = potentialDirection.normalized(maximalPotential);

  //const Vector3f nonTargetElbowPotential = (sidePotentialOnElbow + potentialByObjectsOnElbow).normalized(std::min(sidePotential + maximalPotential, std::max(1.f, maximalPotential)));
  const Vector3f elbowPotential = sidePotentialOnElbow + (targetElbowPotential + potentialByObjectsOnElbow).normalized();
  //const Vector3f handPotential = targetHandPotential + sidePotentialOnHand;

  const Vector3f newElbowPosition = elbowPose.translation + elbowPotential.normalized((targetElbowPosition - elbowPose.translation).norm());
  //const Vector3f newHandPosition = handPostition + handPotential.normalized((targetHandPostion - handPostition).norm());

  InverseKinematic::calcShoulderJoints(arm, newElbowPosition, theRobotDimensions, theJointLimits, supersedeOutput);
}
