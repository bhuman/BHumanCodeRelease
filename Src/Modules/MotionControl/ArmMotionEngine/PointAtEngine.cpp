/**
 * @file PointAtEngine.cpp
 * This file implements a module that points with arms at specified points.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "PointAtEngine.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(PointAtEngine, motionControl)

void PointAtEngine::update(PointAtEngineOutput& pointAtEngineOutput)
{
  DECLARE_DEBUG_DRAWING3D("module:PointAtEngine:robot", "robot");
  update(Arms::left, pointAtEngineOutput);
  update(Arms::right, pointAtEngineOutput);
}

void PointAtEngine::update(Arms::Arm arm, PointAtEngineOutput& pointAtEngineOutput)
{
  if(theArmMotionSelection.targetArmMotion[arm] != ArmMotionSelection::pointAtS)
  {
    pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] = theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)];
    pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)] = theJointAngles.angles[Joints::combine(arm, Joints::shoulderRoll)];
    return; //not selected
  }

  const Angle oldShoulderPitchValue = std::isnan(static_cast<float>(pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)])) ? theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)] : pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)];
  const Angle oldShoulderRollValue = std::isnan(static_cast<float>(pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)])) ? theJointAngles.angles[Joints::combine(arm, Joints::shoulderRoll)] : pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)];
  //const Angle oldShoulderPitchValue = theJointAngles.angles[Joints::combine(arm, Joints::shoulderPitch)]; //even if this is cooler from an theorical view it is not very good with the actual robot, because it is more delicate by loose joints
  //const Angle oldShoulderRollValue = theJointAngles.angles[Joints::combine(arm, Joints::shoulderRoll)]; // also if the target is to high the robot need a good force (approx stiff * dist) to reach it, but this provides at the end of the movement just low dists
  const Vector3f point(theTorsoMatrix.inverse() * theArmMotionRequest.pointToPointAt);

  if(!theTorsoMatrix.isValid
     || Vector2f(theArmMotionRequest.pointToPointAt.x(), theArmMotionRequest.pointToPointAt.y()).squaredNorm() < maxSquardDisToIgnorePoints //not far away from robot
     || (theRobotModel.limbs[Limbs::combine(arm, Limbs::shoulder)].inverse() * point).squaredNorm() < sqr(theRobotDimensions.lowerArmLength + theRobotDimensions.upperArmLength)//not far enough from shoulder
     || point.x() < -100.f //to far behind the robot
     || (arm == Arms::left ? point.y() : -point.y()) < -100.f)
  {
    pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] = 90_deg;
    pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)] = 0;
  }
  else
    calcShoulderJoints(arm, point, theRobotDimensions, pointAtEngineOutput);

  pointAtEngineOutput.angles[Joints::combine(arm, Joints::elbowRoll)] = 0;
  pointAtEngineOutput.angles[Joints::combine(arm, Joints::elbowYaw)] = 0;
  pointAtEngineOutput.angles[Joints::combine(arm, Joints::wristYaw)] = arm == Arms::left ? -90_deg : 90_deg;
  pointAtEngineOutput.angles[Joints::combine(arm, Joints::hand)] = 0;

  COMPLEX_DRAWING3D("module:PointAtEngine:robot")
  {
    const int sign = arm == Arms::left ? 1 : -1;
    const Joints::Joint arm0 = Joints::combine(arm, Joints::shoulderPitch);

    const Pose3f elbowPos = Pose3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z())
                            .rotateY(pointAtEngineOutput.angles[arm0 + 0])
                            .rotateZ(pointAtEngineOutput.angles[arm0 + 1])
                            .translate(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder * sign, 0);

    const Vector3f dir(elbowPos.translated(10.f, 0.f, 0.f).translation - elbowPos.translation);
    const Vector3f endPoint = elbowPos.translation + dir.normalized(50000.f);
    LINE3D("module:PointAtEngine:robot", elbowPos.translation.x(), elbowPos.translation.y(), elbowPos.translation.z(), endPoint.x(), endPoint.y(), endPoint.z(), 20, arm == Arms::left ? ColorRGBA::red : ColorRGBA::blue);

    const Pose3f& elbowPos2 = theRobotModel.limbs[Limbs::combine(arm, Limbs::elbow)];
    const Vector3f hand = theRobotModel.limbs[Limbs::combine(arm, Limbs::elbow)].translated(10.f, 0.f, 0.f).translation;
    const Vector3f dir2(elbowPos2.translated(10.f, 0.f, 0.f).translation - elbowPos2.translation);
    const Vector3f endPoint2 = elbowPos2.translation + dir2.normalized(50000.f);
    LINE3D("module:PointAtEngine:robot", elbowPos2.translation.x(), elbowPos2.translation.y(), elbowPos2.translation.z(), endPoint2.x(), endPoint2.y(), endPoint2.z(), 20, arm == Arms::left ? ColorRGBA::magenta : ColorRGBA::cyan);
  }

  pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] = oldShoulderPitchValue + (pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderPitch)] - oldShoulderPitchValue) / smoothnessValue;
  pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)] = oldShoulderRollValue + (pointAtEngineOutput.angles[Joints::combine(arm, Joints::shoulderRoll)] - oldShoulderRollValue) / smoothnessValue;

  for(size_t i = Joints::combine(arm, Joints::shoulderPitch); i <= Joints::combine(arm, Joints::hand); ++i)
    pointAtEngineOutput.stiffnessData.stiffnesses[i] = stiffness;
}

void PointAtEngine::calcShoulderJoints(const Arms::Arm arm, const Vector3f& pointPosition, const RobotDimensions& robotDimensions,
                                       JointAngles& targetJointAngles) const
{
  const unsigned indexOffset = arm == Arms::left ? 0 : Joints::firstRightArmJoint - Joints::firstLeftArmJoint;
  const float sign(arm == Arms::left ? 1.f : -1.f);

  const Pose3f shoulder(
    robotDimensions.armOffset.x(),
    robotDimensions.armOffset.y() * sign + robotDimensions.yOffsetElbowToShoulder * sign,
    robotDimensions.armOffset.z());
  const Vector3f elbowInShoulder = shoulder.inverse() * pointPosition;
  const Angle& shoulderPitch = targetJointAngles.angles[Joints::lShoulderPitch + indexOffset] =
    -std::atan2(elbowInShoulder.z(), elbowInShoulder.x());

  const Pose3f shoulderPitchPos = shoulder * RotationMatrix::aroundY(shoulderPitch);
  const Vector3f elbowInShoulderPitch = shoulderPitchPos.inverse() * pointPosition;
  targetJointAngles.angles[Joints::lShoulderRoll + indexOffset] = std::atan2(elbowInShoulderPitch.y(), elbowInShoulderPitch.x());
}
