/**
 * @file InverseKinematic.cpp
 * @author Alexander Härtl
 * @author Thijs Jeffry de Haas
 * @author Jesse Richter-Klug
 */

#include "InverseKinematic.h"
#include "Math/Range.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Math/BHMath.h"
#include "Math/Pose3f.h"
#include "Math/Rotation.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Vector2f& bodyRotation,
                                     JointAngles& jointAngles, const RobotDimensions& robotDimensions)
{
  const Quaternionf bodyRot = Rotation::aroundX(bodyRotation.x()) * Rotation::aroundY(bodyRotation.y());
  return calcLegJoints(positionLeft, positionRight, bodyRot, jointAngles, robotDimensions);
}

bool InverseKinematic::calcLegJoints(const Pose3f& positionLeft, const Pose3f& positionRight, const Quaternionf& bodyRotation,
                                     JointAngles& jointAngles, const RobotDimensions& robotDimensions)
{
  jointAngles.angles[Joints::waistYaw] = 0.f;

  // TODO: apply aroundZ(-waistYaw) before or after bodyRotation?
  const Pose3f lTarget0 = (((Pose3f(0.f, -robotDimensions.yHipOffset, 0.f)) *= bodyRotation.inverse()) *= positionLeft) += Vector3f(0.f, 0.f, robotDimensions.footHeight);
  const Pose3f rTarget0 = (((Pose3f(0.f, robotDimensions.yHipOffset, 0.f)) *= bodyRotation.inverse()) *= positionRight) += Vector3f(0.f, 0.f, robotDimensions.footHeight);

  const auto calcSingleLeg = [&](const Pose3f& ankleInPelvis, Legs::Leg leg)
  {
    const float constPart = ankleInPelvis.translation.squaredNorm() + sqr(robotDimensions.hipPitchToRollOffset.z()) + sqr(robotDimensions.xOffsetHipToKnee) + sqr(robotDimensions.zOffsetAnklePitchToRoll) + sqr(robotDimensions.upperLegLength) - sqr(robotDimensions.lowerLegLength);

    const auto hipPitchResidual = [&](float hipPitch) -> float
    {
      const float c1 = std::cos(hipPitch), s1 = std::sin(hipPitch);

      const Vector3f ankleInShiftedPelvis = ankleInPelvis.translation - robotDimensions.hipPitchToRollOffset.z() * Vector3f(s1, 0.f, c1);

      const Vector3f yP = ankleInPelvis.rotation.col(0).cross(ankleInShiftedPelvis).normalized();

      const Vector3f zP = Vector3f(c1, 0.f, -s1).cross(yP).normalized();

      const Vector3f xP = yP.cross(zP);

      const float ankleX = xP.dot(ankleInShiftedPelvis - robotDimensions.zOffsetAnklePitchToRoll * ankleInPelvis.rotation.col(0).cross(yP));

      // Do hip, ankle and knee form a counterclockwise triangle?
      if(ankleX > robotDimensions.xOffsetHipToKnee)
        return std::numeric_limits<float>::max();

      const Vector3f kneeInHipPitch = robotDimensions.xOffsetHipToKnee * xP - robotDimensions.upperLegLength * zP;

      return constPart
           - 2.f * robotDimensions.hipPitchToRollOffset.z() * ankleInPelvis.translation.dot(Vector3f(s1, 0.f, c1))
           - 2.f * robotDimensions.zOffsetAnklePitchToRoll * (ankleInShiftedPelvis - kneeInHipPitch).dot(ankleInPelvis.rotation.col(0).cross(yP))
           - 2.f * ankleInShiftedPelvis.dot(kneeInHipPitch);
    };

    static const Angle minPitch = -118_deg;
    static const Angle maxPitch = 118_deg;
    static const Angle bigStep = 3_deg;
    static const Angle smallStep = 0.1_deg;

    const auto calcHipPitch = [&]() -> float
    {
      float hipPitch = minPitch;
      float prevPrevResidual = std::abs(hipPitchResidual(hipPitch));
      hipPitch += bigStep;
      float prevResidual = std::abs(hipPitchResidual(hipPitch));
      while(hipPitch < maxPitch)
      {
        hipPitch += bigStep;
        const float thisResidual = std::abs(hipPitchResidual(hipPitch));
        if(thisResidual - prevResidual > 0.3 * (prevResidual - prevPrevResidual))
          break;
        prevPrevResidual = prevResidual;
        prevResidual = thisResidual;
      }

      hipPitch -= 2.f * bigStep;
      prevResidual = prevPrevResidual;
      while(hipPitch < maxPitch)
      {
        hipPitch += smallStep;
        const float thisResidual = std::abs(hipPitchResidual(hipPitch));
        if(thisResidual >= prevResidual)
          return hipPitch - smallStep;
        prevResidual = thisResidual;
      }
      return hipPitch;
    };

    const float hipPitch = calcHipPitch();
    const float c1 = std::cos(hipPitch), s1 = std::sin(hipPitch);

    const Vector2f hipRollDirection = (Matrix2x3f() << 0.f, 1.f, 0.f, s1, 0.f, c1).finished() * ankleInPelvis.rotation.col(0).cross(ankleInPelvis.translation - robotDimensions.hipPitchToRollOffset.z() * Vector3f(s1, 0.f, c1));
    const float hipRoll = hipRollDirection.angle();
    const float c2 = std::cos(hipRoll), s2 = std::sin(hipRoll); // hipRollDirection.normalized();

    const Vector2f hipYawDirection = (Matrix2x3f() << c1, 0, -s1, s1 * s2, c2, c1 * s2).finished() * ankleInPelvis.rotation.col(0);
    const float hipYaw = (hipYawDirection.x() < 0.f ? -hipYawDirection : hipYawDirection).angle();
    const float c3 = std::cos(hipYaw), s3 = std::sin(hipYaw); // (-)hipYawDirection.normalized();

    const Vector3f yP(-s3 * c1 + c3 * s2 * s1, c3 * c2, s3 * s1 + c3 * s2 * c1);
    const float ankleRoll = std::asin(Rangef::OneRange().limit(-ankleInPelvis.rotation.col(2).dot(yP)));

    const Vector3f xP(c3 * c1 + s3 * s2 * s1, s3 * c2, -c3 * s1 + s3 * s2 * c1);
    // This can be refactored:
    const float lengthSqr = ankleInPelvis.translation.squaredNorm() + sqr(robotDimensions.hipPitchToRollOffset.z()) + sqr(robotDimensions.xOffsetHipToKnee) + sqr(robotDimensions.zOffsetAnklePitchToRoll)
                          + 2.f * robotDimensions.hipPitchToRollOffset.z() * robotDimensions.xOffsetHipToKnee * s2 * s3
                          - 2.f * robotDimensions.hipPitchToRollOffset.z() * (s1 * ankleInPelvis.translation.x() + c1 * ankleInPelvis.translation.z())
                          - 2.f * robotDimensions.xOffsetHipToKnee * ankleInPelvis.translation.dot(xP)
                          - 2.f * robotDimensions.zOffsetAnklePitchToRoll * ankleInPelvis.translation.dot(ankleInPelvis.rotation.col(0).cross(yP))
                          + 2.f * robotDimensions.xOffsetHipToKnee * robotDimensions.zOffsetAnklePitchToRoll * xP.dot(ankleInPelvis.rotation.col(0).cross(yP))
                          + 2.f * robotDimensions.hipPitchToRollOffset.z() * robotDimensions.zOffsetAnklePitchToRoll * ankleInPelvis.rotation.col(0).cross(yP).dot(Vector3f(s1, 0.f, c1));
    const float cosKneePitch = (lengthSqr - sqr(robotDimensions.upperLegLength) - sqr(robotDimensions.lowerLegLength)) / (2.f * robotDimensions.lowerLegLength * robotDimensions.upperLegLength);
    // TODO: determine reachability here, return it
    const float kneePitch = std::acos(Rangef::OneRange().limit(cosKneePitch));

    const float anklePitch = std::asin(-Vector3f(std::sin(kneePitch), 0.f, std::cos(kneePitch)).dot(RotationMatrix::aroundZ(-hipYaw) * RotationMatrix::aroundX(-hipRoll) * RotationMatrix::aroundY(-hipPitch) * ankleInPelvis.rotation.col(0)));

    jointAngles.angles[Joints::combine(leg, Joints::hipYaw)] = hipYaw;
    jointAngles.angles[Joints::combine(leg, Joints::hipRoll)] = hipRoll;
    jointAngles.angles[Joints::combine(leg, Joints::hipPitch)] = hipPitch;
    jointAngles.angles[Joints::combine(leg, Joints::kneePitch)] = kneePitch;
    jointAngles.angles[Joints::combine(leg, Joints::anklePitch)] = anklePitch;
    jointAngles.angles[Joints::combine(leg, Joints::ankleRoll)] = ankleRoll;
  };

  calcSingleLeg(lTarget0, Legs::left);
  calcSingleLeg(rTarget0, Legs::right);

  return true;
}

void InverseKinematic::calcHeadJoints(const Vector3f& position, const Angle imageTilt, const RobotDimensions& robotDimensions,
                                      Vector2a& panTilt, const CameraCalibration& cameraCalibration)
{
  const Vector2f headJoint2Target(std::sqrt(sqr(position.x() - robotDimensions.hipToNeckOffset.x()) +
                                            sqr(position.y() - robotDimensions.hipToNeckOffset.y())),
                                  position.z() - robotDimensions.hipToNeckOffset.z());
  const Vector2f headJoint2Camera(robotDimensions.offsetNeckToCamera.x(),
                                  robotDimensions.offsetNeckToCamera.z());
  const float headJoint2CameraAngle = std::atan2(headJoint2Camera.x(), headJoint2Camera.y());
  const float cameraAngle = pi3_2 - imageTilt - (pi_2 - headJoint2CameraAngle);
  const float targetAngle = std::asin(Rangef(-1.f, 1.f).limit(headJoint2Camera.norm() * std::sin(cameraAngle) / headJoint2Target.norm()));
  const float headJointAngle = pi - targetAngle - cameraAngle;
  panTilt.y() = std::atan2(headJoint2Target.x(), headJoint2Target.y()) - headJointAngle - headJoint2CameraAngle;
  panTilt.x() = std::atan2(position.y(), position.x());
  panTilt.x() -= cameraCalibration.cameraRotationCorrection.z();
  panTilt.y() -= cameraCalibration.cameraRotationCorrection.y();
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

  const Pose3f shoulderPitchPos = shoulder * RotationMatrix::aroundY(shoulderPitch);
  const Vector3f elbowInShoulderPitch = shoulderPitchPos.inverse() * elBowPosition;
  const Angle shoulderRoll = jointAngles.angles[Joints::lShoulderRoll + indexOffset] = std::atan2(elbowInShoulderPitch.y(), elbowInShoulderPitch.x());

  //elbow
  const Pose3f elbow = (shoulderPitchPos * RotationMatrix::aroundZ(shoulderRoll)).translate(robotDimensions.upperArmLength, robotDimensions.yOffsetElbowToShoulder * sign, 0);
  const Vector3f handInElbow = elbow.inverse() * handPosition;
  if(handInElbow.y() == 0.f && handInElbow.z() == 0.f) //range instead of 0
    jointAngles.angles[Joints::lElbowPitch + indexOffset] = 0;
  else
    jointAngles.angles[Joints::lElbowPitch + indexOffset] = sign * -std::atan2(handInElbow.z(), sign * -handInElbow.y());
  const Angle elbowYaw = jointAngles.angles[Joints::lElbowPitch + indexOffset];

  if(elbowYaw + smallValue < jointLimits.limits[Joints::lElbowPitch + indexOffset].min ||
     elbowYaw - smallValue > jointLimits.limits[Joints::lElbowPitch + indexOffset].max)
    errCode |= bit(elbowYawOutOfRange);

  const float aQuadrat = (handPosition - elBowPosition).squaredNorm();
  const float bQuadrat = (elBowPosition - shoulder.translation).squaredNorm();
  const float cQuadrat = (handPosition - shoulder.translation).squaredNorm();
  const float twoAB = (2.f * (handPosition - elBowPosition).norm() * (elBowPosition - shoulder.translation).norm());
  const Angle elbowRoll = jointAngles.angles[Joints::lElbowYaw + indexOffset] = sign * (-180_deg + std::acos((aQuadrat + bQuadrat - cQuadrat) / twoAB));

  if(elbowRoll + smallValue < jointLimits.limits[Joints::lElbowYaw + indexOffset].min ||
     elbowRoll - smallValue > jointLimits.limits[Joints::lElbowYaw + indexOffset].max)
    errCode |= bit(elbowRollOutOfRange);

  if(Global::getSettings().robotType != Settings::t1)
  {
    jointAngles.angles[Joints::lShoulderPitch + indexOffset] -= 90_deg;
    jointAngles.angles[Joints::lShoulderRoll + indexOffset] += 90_deg * (arm == Arms::left ? -1.f : 1.f);
    jointAngles.angles[Joints::lElbowPitch + indexOffset] += 90_deg;
  }

  if(!jointLimits.limits[Joints::lShoulderPitch + indexOffset].isInside(shoulderPitch))
    errCode |= bit(shoulderPitchOutOfRange);

  if(!jointLimits.limits[Joints::lShoulderRoll + indexOffset].isInside(shoulderRoll))
    errCode |= bit(shoulderRollOutOfRange);

  return errCode;
}

unsigned InverseKinematic::calcArmJoints(const Arms::Arm arm, const Pose3f& handPose, const RobotDimensions& robotDimensions,
                                         const JointLimits& jointLimits, JointAngles& jointAngles)
{
  const Vector3f elbowPosition = handPose * Vector3f(-robotDimensions.handOffset.x() - robotDimensions.xOffsetElbowToWrist, 0.f, 0.f);
  return calcArmJoints(arm, elbowPosition, handPose.translation, robotDimensions, jointLimits, jointAngles);
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

  const Pose3f shoulderPitchPos = shoulder * RotationMatrix::aroundY(shoulderPitch);
  const Vector3f elbowInShoulderPitch = shoulderPitchPos.inverse() * elBowPosition;
  const Angle shoulderRoll = jointAngles.angles[Joints::lShoulderRoll + indexOffset] = std::atan2(elbowInShoulderPitch.y(), elbowInShoulderPitch.x());

  if(Global::getSettings().robotType != Settings::t1)
  {
    jointAngles.angles[Joints::lShoulderPitch + indexOffset] -= 90_deg;
    jointAngles.angles[Joints::lShoulderRoll + indexOffset] += 90_deg * (arm == Arms::left ? -1.f : 1.f);
    // ShoulderYaw not necessary, as the arm is always straight
  }

  if(!jointLimits.limits[Joints::lShoulderPitch + indexOffset].isInside(shoulderPitch))
    errCode |= bit(shoulderPitchOutOfRange);

  if(!jointLimits.limits[Joints::lShoulderRoll + indexOffset].isInside(shoulderRoll))
    errCode |= bit(shoulderRollOutOfRange);

  return errCode;
}
