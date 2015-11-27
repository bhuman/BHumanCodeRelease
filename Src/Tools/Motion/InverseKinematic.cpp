/**
 * @file InverseKinematic.cpp
 * @author Alexander Härtl
 * @author jeff
 * @author Jesse Richter-Klug
 */

#include "InverseKinematic.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/JointCalibration.h"
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
  const Rangef& cosClipping = Rangef::OneRange();

  ratio = Rangef::ZeroOneRange().limit(ratio);

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

  const Pose3f lTarget1 = Pose3f(RotationMatrix::aroundZ(lJoint0Combined)) *= lTarget0;
  const Pose3f rTarget1 = Pose3f(RotationMatrix::aroundZ(-lJoint0Combined)) *= rTarget0;
  const Vector3f& lHipToFoot = lTarget1.translation;
  const Vector3f& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y(), -rHipToFoot.z());
  const float lJoint2MinusAlpha = std::atan2(-lHipToFoot.x(), std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float rJoint2MinusAlpha = std::atan2(-rHipToFoot.x(), std::sqrt(sqr(rHipToFoot.y()) + sqr(rHipToFoot.z())) * -sgn(rHipToFoot.z()));
  const Vector3f lFootRotationC2 = RotationMatrix::aroundY(-lJoint2MinusAlpha) * RotationMatrix::aroundX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);
  const Vector3f rFootRotationC2 = RotationMatrix::aroundY(-rJoint2MinusAlpha) * RotationMatrix::aroundX(-rPi_4AndJoint1) * rTarget1.rotation.col(2);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.norm();
  const float hr = rTarget1.translation.norm();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f* h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f* h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f* h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f* h2 * hr);
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

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation,
                                     JointAngles& jointAngles, const RobotDimensions& robotDimensions, float ratio)
{
  static const Pose3f rotPi_4 = RotationMatrix::aroundX(pi_4);
  static const Pose3f rotMinusPi_4 = RotationMatrix::aroundX(-pi_4);
  const Rangef& cosClipping = Rangef::OneRange();

  ratio = Rangef::ZeroOneRange().limit(ratio);

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

  const Pose3f lTarget1 = Pose3f(RotationMatrix::aroundZ(lJoint0Combined)) *= lTarget0;
  const Pose3f rTarget1 = Pose3f(RotationMatrix::aroundZ(-lJoint0Combined)) *= rTarget0;
  const Vector3f& lHipToFoot = lTarget1.translation;
  const Vector3f& rHipToFoot = rTarget1.translation;
  const float lMinusPi_4MinusJoint1 = -std::atan2(-lHipToFoot.y(), -lHipToFoot.z());
  const float rPi_4AndJoint1 = -std::atan2(-rHipToFoot.y(), -rHipToFoot.z());
  const float lJoint2MinusAlpha = std::atan2(-lHipToFoot.x(), std::sqrt(sqr(lHipToFoot.y()) + sqr(lHipToFoot.z())) * -sgn(lHipToFoot.z()));
  const float rJoint2MinusAlpha = std::atan2(-rHipToFoot.x(), std::sqrt(sqr(rHipToFoot.y()) + sqr(rHipToFoot.z())) * -sgn(rHipToFoot.z()));
  const Vector3f lFootRotationC2 = RotationMatrix::aroundY(-lJoint2MinusAlpha) * RotationMatrix::aroundX(-lMinusPi_4MinusJoint1) * lTarget1.rotation.col(2);
  const Vector3f rFootRotationC2 = RotationMatrix::aroundY(-rJoint2MinusAlpha) * RotationMatrix::aroundX(-rPi_4AndJoint1) * rTarget1.rotation.col(2);
  const float h1 = robotDimensions.upperLegLength;
  const float h2 = robotDimensions.lowerLegLength;
  const float hl = lTarget1.translation.norm();
  const float hr = rTarget1.translation.norm();
  const float h1Sqr = h1 * h1;
  const float h2Sqr = h2 * h2;
  const float hlSqr = hl * hl;
  const float hrSqr = hr * hr;
  const float lCosMinusAlpha = (h1Sqr + hlSqr - h2Sqr) / (2.f* h1 * hl);
  const float rCosMinusAlpha = (h1Sqr + hrSqr - h2Sqr) / (2.f* h1 * hr);
  const float lCosMinusBeta = (h2Sqr + hlSqr - h1Sqr) / (2.f* h2 * hl);
  const float rCosMinusBeta = (h2Sqr + hrSqr - h1Sqr) / (2.f* h2 * hr);
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

void InverseKinematic::calcHeadJoints(const Vector3f& position, const float imageTilt, const RobotDimensions& robotDimensions,
                                      const bool lowerCamera, Vector2f& panTilt, const CameraCalibration& cameraCalibration)
{
  const Vector2f headJoint2Target(std::sqrt(sqr(position.x()) + sqr(position.y())), position.z() - robotDimensions.hipToNeckLength);
  const Vector2f headJoint2Camera(robotDimensions.getXOffsetNeckToCamera(lowerCamera),
                                  robotDimensions.getZOffsetNeckToCamera(lowerCamera));
  const float headJoint2CameraAngle = std::atan2(headJoint2Camera.x(), headJoint2Camera.y());
  const float cameraAngle = pi3_2 - imageTilt - (pi_2 - headJoint2CameraAngle) - robotDimensions.getTiltNeckToCamera(lowerCamera);
  const float targetAngle = std::asin(headJoint2Camera.norm() * std::sin(cameraAngle) / headJoint2Target.norm());
  const float headJointAngle = pi - targetAngle - cameraAngle;
  panTilt.y() = std::atan2(headJoint2Target.x(), headJoint2Target.y()) - headJointAngle - headJoint2CameraAngle;
  panTilt.x() = std::atan2(position.y(), position.x());
  if(lowerCamera)
  {
    panTilt.x() -= cameraCalibration.lowerCameraRotationCorrection.z();
    panTilt.y() -= cameraCalibration.lowerCameraRotationCorrection.y();
  }
  else
  {
    panTilt.x() -= cameraCalibration.upperCameraRotationCorrection.z();
    panTilt.y() -= cameraCalibration.upperCameraRotationCorrection.y();
  }
}
