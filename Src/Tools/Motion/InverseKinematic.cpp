/**
 * @file InverseKinematic.cpp
 * @author Alexander Härtl
 * @author jeff
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "InverseKinematic.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Tools/Range.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/Rotation.h"

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, JointAngles& jointAngles,
                                     const RobotDimensions& robotDimensions, float ratio)
{
  static const Pose3f rotPi_4 = RotationMatrix::aroundX(pi_4);
  static const Pose3f rotMinusPi_4 = RotationMatrix::aroundX(-pi_4);
  const Rangef cosClipping = Rangef::OneRange();

  Rangef::ZeroOneRange().clamp(ratio);

  const Pose3f lTarget0 = (rotMinusPi_4 + Vector3f(0.f, -robotDimensions.yHipOffset, 0.f)) *= positionLeft;
  const Pose3f rTarget0 = (rotPi_4 + Vector3f(0.f, robotDimensions.yHipOffset, 0.f)) *= positionRight;
  const Vector3f lFootToHip = lTarget0.rotation.inverse() * -lTarget0.translation;
  const Vector3f rFootToHip = rTarget0.rotation.inverse() * -rTarget0.translation;
  const float lMinusJoint5 = std::atan2(lFootToHip.y(), lFootToHip.z());
  const float rJoint5 = std::atan2(rFootToHip.y(), rFootToHip.z());
  const float lMinusBetaAndJoint4 = -std::atan2(lFootToHip.x(), std::sqrt(sqr(lFootToHip.y()) + sqr(lFootToHip.z())));
  const float rMinusBetaAndJoint4 = -std::atan2(rFootToHip.x(), std::sqrt(sqr(rFootToHip.y()) + sqr(rFootToHip.z())));
  const Vector3f lHipRotationC1 = lTarget0.rotation * (RotationMatrix::aroundX(-lMinusJoint5) * RotationMatrix::aroundY(-lMinusBetaAndJoint4)).col(1);
  const Vector3f rHipRotationC1 = rTarget0.rotation * (RotationMatrix::aroundX(-rJoint5) * RotationMatrix::aroundY(-rMinusBetaAndJoint4)).col(1);
  const float lMinusJoint0 = std::atan2(-lHipRotationC1.x(), lHipRotationC1.y());
  const float rJoint0 = std::atan2(-rHipRotationC1.x(), rHipRotationC1.y());
  const float lJoint0Combined = -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

  const Pose3f lTarget1 = RotationMatrix::aroundZ(lJoint0Combined) * lTarget0;
  const Pose3f rTarget1 = RotationMatrix::aroundZ(-lJoint0Combined) * rTarget0;
  const Vector3f& lHipToFoot = lTarget1.translation;
  const Vector3f& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y(), -rHipToFoot.z());
  const float lJoint2MinusAlpha = std::atan2(-lHipToFoot.x(), std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float rJoint2MinusAlpha = std::atan2(-rHipToFoot.x(), std::sqrt(sqr(rHipToFoot.y()) + sqr(rHipToFoot.z())) * -sgn(rHipToFoot.z()));
  const Vector3f lFootRotationC2 = Rotation::aroundY(-lJoint2MinusAlpha) * Rotation::aroundX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);
  const Vector3f rFootRotationC2 = Rotation::aroundY(-rJoint2MinusAlpha) * Rotation::aroundX(-rPi_4AndJoint1) * rTarget1.rotation.col(2);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.norm();
  const float hr = rTarget1.translation.norm();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const float rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

  jointAngles.angles[Joints::lHipYawPitch] = lJoint0Combined;
  jointAngles.angles[Joints::lHipRoll] = (lMinusPi_4MinusJoint1 + pi_4);
  jointAngles.angles[Joints::lHipPitch] = lJoint2MinusAlpha + lAlpha;
  jointAngles.angles[Joints::lKneePitch] = -lAlpha - lBeta;
  jointAngles.angles[Joints::lAnklePitch] = std::atan2(lFootRotationC2.x(), lFootRotationC2.z()) + lBeta;
  jointAngles.angles[Joints::lAnkleRoll] = std::asin(-lFootRotationC2.y());

  jointAngles.angles[Joints::rHipYawPitch] = lJoint0Combined;
  jointAngles.angles[Joints::rHipRoll] = rPi_4AndJoint1 - pi_4;
  jointAngles.angles[Joints::rHipPitch] = rJoint2MinusAlpha + rAlpha;
  jointAngles.angles[Joints::rKneePitch] = -rAlpha - rBeta;
  jointAngles.angles[Joints::rAnklePitch] = std::atan2(rFootRotationC2.x(), rFootRotationC2.z()) + rBeta;
  jointAngles.angles[Joints::rAnkleRoll] = std::asin(-rFootRotationC2.y());
  const float maxLen = h1 + h2;
  return hl <= maxLen && hr <= maxLen;
}

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Vector2f& bodyRotation,
                                     JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio)
{
  const Quaternionf bodyRot = Rotation::aroundX(bodyRotation.x()) * Rotation::aroundY(bodyRotation.y());
  return calcLegJoints(positionLeft, positionRight, bodyRot, jointAngles, robotDimensions, ratio);
}

bool InverseKinematic::isPosePossible(const Pose3f& swingInSupportFoot, const Pose3f& torso, float hipYawPitch, const RobotDimensions& robotDimensions, const JointLimits jointLimits)
{
  //Maybe expects sole instead of limb (but every time false...)
  //check overlapping
  if(std::abs(swingInSupportFoot.translation.x()) < 40 && std::abs(swingInSupportFoot.translation.y()) < 60) //TODO: measure values
    return false;

  Pose3f footPos = torso.inverse() * swingInSupportFoot;

  if(footPos.translation.y() < 0)
  {
    footPos.translation(1) = -footPos.translation.y();
    //TODO: mirror y-rotation
  }

  JointAngles jointAngles = JointAngles();
  const Rangef cosClipping = Rangef::OneRange();

  static const Pose3f rotMinusPi_4 = RotationMatrix::aroundX(-pi_4);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;

  const Pose3f lTarget0 = ((rotMinusPi_4 + Vector3f(0.f, -robotDimensions.yHipOffset, 0.f))  *= footPos) += Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Pose3f lTarget1 = RotationMatrix::aroundZ(hipYawPitch) * lTarget0;
  const float hl = lTarget1.translation.norm();

  //check distance
  if(hl > h1 + h2)
    return false;

  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;

  const Vector3f& lHipToFoot = lTarget1.translation;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);

  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float lJoint2MinusAlpha = std::atan2(-lHipToFoot.x(), std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const Vector3f lFootRotationC2 = Rotation::aroundY(-lJoint2MinusAlpha) * Rotation::aroundX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);

  jointAngles.angles[Joints::lHipYawPitch] = hipYawPitch;
  jointAngles.angles[Joints::lHipRoll] = (lMinusPi_4MinusJoint1 + pi_4);
  jointAngles.angles[Joints::lHipPitch] = lJoint2MinusAlpha + lAlpha;
  jointAngles.angles[Joints::lKneePitch] = -lAlpha - lBeta;
  jointAngles.angles[Joints::lAnklePitch] = std::atan2(lFootRotationC2.x(), lFootRotationC2.z()) + lBeta;
  jointAngles.angles[Joints::lAnkleRoll] = std::asin(-lFootRotationC2.y());

  //check angles
  for(const Angle& angle : jointAngles.angles)
    if(!std::isfinite(angle))
      return false;

  for(int j = Joints::firstLeftLegJoint; j < Joints::firstRightLegJoint; j++)
    if(!jointLimits.limits[j].isInside(jointAngles.angles[j]))
      return false;

  //recheck pose with forwardkinematic
  ENUM_INDEXED_ARRAY(Pose3f, Limbs::Limb) limbs;
  ForwardKinematic::calculateLegChain(Legs::left, jointAngles, robotDimensions, limbs);
  Pose3f target = limbs[Limbs::footLeft] += Vector3f(0.f, 0.f, -robotDimensions.footHeight);

  Pose3f diff = target.inverse() * footPos;
  if(std::abs(diff.translation.sum()) > 1)
    return false;

  return true;
}

bool InverseKinematic::calcBalancedLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation,
                                             JointAngles& jointAngles, const RobotDimensions& theRobotDimensions, const MassCalibration& theMassCalibration, const JointLimits& theJointLimits, float ratio)
{
  Vector3f comInTorso(0.f, 0.f, 0.f);
  Pose3f leftAnkleTorso;
  Pose3f rightAnkleTorso;
  for(int i = 0; ; ++i)
  {
    //Compute Joint Angles
    leftAnkleTorso = Pose3f(comInTorso) * positionLeft;
    rightAnkleTorso = Pose3f(comInTorso) * positionRight;
    (void)calcLegJoints(leftAnkleTorso, rightAnkleTorso, jointAngles, theRobotDimensions, ratio);

    JointAngles cuttedAngles(jointAngles);  //Apply Joint Limits
    for(int j = Joints::headYaw; j < Joints::rAnkleRoll; j++)
      cuttedAngles.angles[j] = theJointLimits.limits[j].limit(cuttedAngles.angles[j]);

    //Compute Com
    RobotModel robotModel(cuttedAngles, theRobotDimensions, theMassCalibration);
    Vector3f delta = (robotModel.centerOfMass - comInTorso) * 1.3f;

    if(i >= 7 || (std::abs(delta.x()) < 0.05f && std::abs(delta.y()) < 0.05f/* && std::abs(delta.z()) < 0.05f*/))
    {
      comInTorso = robotModel.centerOfMass;
      break;
    }
    //Update ComInTorso
    comInTorso += delta;
  }
  leftAnkleTorso = Pose3f(comInTorso) * positionLeft;
  rightAnkleTorso = Pose3f(comInTorso) * positionRight;
  return calcLegJoints(leftAnkleTorso, rightAnkleTorso, jointAngles, theRobotDimensions, ratio);
}

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation,
                                     JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio)
{
  static const Pose3f rotPi_4 = RotationMatrix::aroundX(pi_4);
  static const Pose3f rotMinusPi_4 = RotationMatrix::aroundX(-pi_4);
  const Rangef cosClipping = Rangef::OneRange();

  Rangef::ZeroOneRange().clamp(ratio);

  const Pose3f lTarget0 = (((rotMinusPi_4 + Vector3f(0.f, -robotDimensions.yHipOffset, 0.f)) *= bodyRotation.inverse()) *= positionLeft) += Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Pose3f rTarget0 = (((rotPi_4 + Vector3f(0.f, robotDimensions.yHipOffset, 0.f)) *= bodyRotation.inverse()) *= positionRight) += Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Vector3f lFootToHip = lTarget0.rotation.inverse() * -lTarget0.translation;
  const Vector3f rFootToHip = rTarget0.rotation.inverse() * -rTarget0.translation;
  const float lMinusJoint5 = std::atan2(lFootToHip.y(), lFootToHip.z());
  const float rJoint5 = std::atan2(rFootToHip.y(), rFootToHip.z());
  const float lMinusBetaAndJoint4 = -std::atan2(lFootToHip.x(), std::sqrt(sqr(lFootToHip.y()) + sqr(lFootToHip.z())));
  const float rMinusBetaAndJoint4 = -std::atan2(rFootToHip.x(), std::sqrt(sqr(rFootToHip.y()) + sqr(rFootToHip.z())));
  const Vector3f lHipRotationC1 = lTarget0.rotation * (RotationMatrix::aroundX(-lMinusJoint5) * RotationMatrix::aroundY(-lMinusBetaAndJoint4)).col(1);
  const Vector3f rHipRotationC1 = rTarget0.rotation * (RotationMatrix::aroundX(-rJoint5) * RotationMatrix::aroundY(-rMinusBetaAndJoint4)).col(1);
  const float lMinusJoint0 = std::atan2(-lHipRotationC1.x(), lHipRotationC1.y());
  const float rJoint0 = std::atan2(-rHipRotationC1.x(), rHipRotationC1.y());
  const float lJoint0Combined = -lMinusJoint0 * ratio + rJoint0 * (1.f - ratio);

  const Pose3f lTarget1 = RotationMatrix::aroundZ(lJoint0Combined) * lTarget0;
  const Pose3f rTarget1 = RotationMatrix::aroundZ(-lJoint0Combined) * rTarget0;
  const Vector3f& lHipToFoot = lTarget1.translation;
  const Vector3f& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y(), -rHipToFoot.z());
  const float lJoint2MinusAlpha = std::atan2(-lHipToFoot.x(), std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float rJoint2MinusAlpha = std::atan2(-rHipToFoot.x(), std::sqrt(sqr(rHipToFoot.y()) + sqr(rHipToFoot.z())) * -sgn(rHipToFoot.z()));
  const Vector3f lFootRotationC2 = Rotation::aroundY(-lJoint2MinusAlpha) * Rotation::aroundX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);
  const Vector3f rFootRotationC2 = Rotation::aroundY(-rJoint2MinusAlpha) * Rotation::aroundX(-rPi_4AndJoint1) * rTarget1.rotation.col(2);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.norm();
  const float hr = rTarget1.translation.norm();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f * h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f * h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f * h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f * h2 * hr);
  const float lAlpha = -std::acos(cosClipping.limit(lCosMinusAlpha));
  const float rAlpha = -std::acos(cosClipping.limit(rCosMinusAlpha));
  const float lBeta = -std::acos(cosClipping.limit(lCosMinusBeta));
  const float rBeta = -std::acos(cosClipping.limit(rCosMinusBeta));

  jointAngles.angles[Joints::lHipYawPitch] = lJoint0Combined;
  jointAngles.angles[Joints::lHipRoll] = (lMinusPi_4MinusJoint1 + pi_4);
  jointAngles.angles[Joints::lHipPitch] = lJoint2MinusAlpha + lAlpha;
  jointAngles.angles[Joints::lKneePitch] = -lAlpha - lBeta;
  jointAngles.angles[Joints::lAnklePitch] = std::atan2(lFootRotationC2.x(), lFootRotationC2.z()) + lBeta;
  jointAngles.angles[Joints::lAnkleRoll] = std::asin(-lFootRotationC2.y());

  jointAngles.angles[Joints::rHipYawPitch] = lJoint0Combined;
  jointAngles.angles[Joints::rHipRoll] = rPi_4AndJoint1 - pi_4;
  jointAngles.angles[Joints::rHipPitch] = rJoint2MinusAlpha + rAlpha;
  jointAngles.angles[Joints::rKneePitch] = -rAlpha - rBeta;
  jointAngles.angles[Joints::rAnklePitch] = std::atan2(rFootRotationC2.x(), rFootRotationC2.z()) + rBeta;
  jointAngles.angles[Joints::rAnkleRoll] = std::asin(-rFootRotationC2.y());
  const float maxLen = h1 + h2;

  return hl <= maxLen &&  hr <= maxLen;
}

void InverseKinematic::calcHeadJoints(const Vector3f& position, const Angle imageTilt, const RobotDimensions& robotDimensions,
                                      CameraInfo::Camera camera, Vector2a& panTilt, const CameraCalibration& cameraCalibration)
{
  const Vector2f headJoint2Target(std::sqrt(sqr(position.x()) + sqr(position.y())), position.z() - robotDimensions.hipToNeckLength);
  const Vector2f headJoint2Camera(robotDimensions.getXOffsetNeckToCamera(camera == CameraInfo::lower),
                                  robotDimensions.getZOffsetNeckToCamera(camera == CameraInfo::lower));
  const float headJoint2CameraAngle = std::atan2(headJoint2Camera.x(), headJoint2Camera.y());
  const float cameraAngle = pi3_2 - imageTilt - (pi_2 - headJoint2CameraAngle) - robotDimensions.getTiltNeckToCamera(camera == CameraInfo::lower);
  const float targetAngle = std::asin(Rangef(-1.f, 1.f).limit(headJoint2Camera.norm() * std::sin(cameraAngle) / headJoint2Target.norm()));
  const float headJointAngle = pi - targetAngle - cameraAngle;
  panTilt.y() = std::atan2(headJoint2Target.x(), headJoint2Target.y()) - headJointAngle - headJoint2CameraAngle;
  panTilt.x() = std::atan2(position.y(), position.x());
  panTilt.x() -= cameraCalibration.cameraRotationCorrections[camera].z();
  panTilt.y() -= cameraCalibration.cameraRotationCorrections[camera].y();
}

unsigned InverseKinematic::calcArmJoints(const Arms::Arm arm, const Vector3f& elBowPosition, const Vector3f& handPosition,
                                         const RobotDimensions& robotDimensions, const JointLimits& jointLimits,
                                         JointAngles& jointAngles)
{
  unsigned errCode(0);
  const unsigned indexOffset = arm == Arms::left ? 0 : Joints::firstRightArmJoint - Joints::firstLeftArmJoint;
  const float sign(arm == Arms::left ? 1.f : -1.f);
  static constexpr Angle smallValue(0.0001_rad);

  //shoulder
  const Pose3f shoulder(robotDimensions.armOffset.x(), robotDimensions.armOffset.y() * sign + robotDimensions.yOffsetElbowToShoulder * sign, robotDimensions.armOffset.z());
  const Vector3f elbowInShoulder = shoulder.inverse() * elBowPosition;
  const Angle shoulderPitch = jointAngles.angles[Joints::lShoulderPitch + indexOffset] = -std::atan2(elbowInShoulder.z(), elbowInShoulder.x());

  if(!jointLimits.limits[Joints::lShoulderPitch + indexOffset].isInside(shoulderPitch))
    errCode |= bit(shoulderPitchOutOfRange);

  const Pose3f shoulderPitchPos = shoulder * RotationMatrix::aroundY(shoulderPitch);
  const Vector3f elbowInShoulderPitch = shoulderPitchPos.inverse() * elBowPosition;
  const Angle shoulderRoll = jointAngles.angles[Joints::lShoulderRoll + indexOffset] = std::atan2(elbowInShoulderPitch.y(), elbowInShoulderPitch.x());

  if(!jointLimits.limits[Joints::lShoulderRoll + indexOffset].isInside(shoulderRoll))
    errCode |= bit(shoulderRollOutOfRange);

  //elbow
  const Pose3f elbow = (shoulderPitchPos * RotationMatrix::aroundZ(shoulderRoll)).translate(robotDimensions.upperArmLength, robotDimensions.yOffsetElbowToShoulder * sign, 0);
  const Vector3f handInElbow = elbow.inverse() * handPosition;
  if(handInElbow.y() == 0.f && handInElbow.z() == 0.f) //range instead of 0
    jointAngles.angles[Joints::lElbowYaw + indexOffset] = 0;
  else
    jointAngles.angles[Joints::lElbowYaw + indexOffset] = sign * -std::atan2(handInElbow.z(), sign * -handInElbow.y());
  const Angle elbowYaw = jointAngles.angles[Joints::lElbowYaw + indexOffset];

  if(elbowYaw + smallValue < jointLimits.limits[Joints::lElbowYaw + indexOffset].min ||
     elbowYaw - smallValue > jointLimits.limits[Joints::lElbowYaw + indexOffset].max)
    errCode |= bit(elbowYawOutOfRange);

  const float aQuadrat = (handPosition - elBowPosition).squaredNorm();
  const float bQuadrat = (elBowPosition - shoulder.translation).squaredNorm();
  const float cQuadrat = (handPosition - shoulder.translation).squaredNorm();
  const float twoAB = (2.f * (handPosition - elBowPosition).norm() * (elBowPosition - shoulder.translation).norm());
  const Angle elbowRoll = jointAngles.angles[Joints::lElbowRoll + indexOffset] = sign * (-180_deg + std::acos((aQuadrat + bQuadrat - cQuadrat) / twoAB));

  if(elbowRoll + smallValue < jointLimits.limits[Joints::lElbowRoll + indexOffset].min ||
     elbowRoll - smallValue > jointLimits.limits[Joints::lElbowRoll + indexOffset].max)
    errCode |= bit(elbowRollOutOfRange);

  return errCode;
}

unsigned InverseKinematic::calcArmJoints(const Arms::Arm arm, const Pose3f& handPose, const RobotDimensions& robotDimensions,
                                         const JointLimits& jointLimits, JointAngles& jointAngles)
{
  const Vector3f elbowPositon = handPose * Vector3f(-robotDimensions.handOffset.x() - robotDimensions.xOffsetElbowToWrist, 0.f, 0.f);
  return calcArmJoints(arm, elbowPositon, handPose.translation, robotDimensions, jointLimits, jointAngles);
}

//this is not correct for the normal purpose, but for pointing it is
unsigned InverseKinematic::calcShoulderJoints(const Arms::Arm arm, const Vector3f& elBowPosition, const RobotDimensions& robotDimensions,
                                              const JointLimits& jointLimits, JointAngles& jointAngles)
{
  unsigned errCode(0);
  const unsigned indexOffset = arm == Arms::left ? 0 : Joints::firstRightArmJoint - Joints::firstLeftArmJoint;
  const float sign(arm == Arms::left ? 1.f : -1.f);

  //shoulder
  const Pose3f shoulder(robotDimensions.armOffset.x(), robotDimensions.armOffset.y() * sign + robotDimensions.yOffsetElbowToShoulder * sign, robotDimensions.armOffset.z());
  const Vector3f elbowInShoulder = shoulder.inverse() * elBowPosition;
  const Angle shoulderPitch = jointAngles.angles[Joints::lShoulderPitch + indexOffset] = -std::atan2(elbowInShoulder.z(), elbowInShoulder.x());

  if(!jointLimits.limits[Joints::lShoulderPitch + indexOffset].isInside(shoulderPitch))
    errCode |= bit(shoulderPitchOutOfRange);

  const Pose3f shoulderPitchPos = shoulder * RotationMatrix::aroundY(shoulderPitch);
  const Vector3f elbowInShoulderPitch = shoulderPitchPos.inverse() * elBowPosition;
  const Angle shoulderRoll = jointAngles.angles[Joints::lShoulderRoll + indexOffset] = std::atan2(elbowInShoulderPitch.y(), elbowInShoulderPitch.x());

  if(!jointLimits.limits[Joints::lShoulderRoll + indexOffset].isInside(shoulderRoll))
    errCode |= bit(shoulderRollOutOfRange);

  return errCode;
}
