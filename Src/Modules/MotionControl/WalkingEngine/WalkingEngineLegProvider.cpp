/**
 * @file WalkingEngineLegProvider.h
 *
 * @author Colin Graf
 * @author Felix Wenk
 * @author Alexis Tsogias
 */

#include "WalkingEngineLegProvider.h"
#include "Platform/SystemCall.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Motion/InverseKinematic.h"

MAKE_MODULE(WalkingEngineLegProvider, motionControl)

using namespace WalkingEngineUtils;

void WalkingEngineLegProvider::update(WalkingEngineOutput& walkingEngineOutput)
{
  generateLegJointRequest();
  generateWalkingEngineOutput(walkingEngineOutput);
}

void WalkingEngineLegProvider::generateLegJointRequest()
{
  DECLARE_PLOT("module:WalkingEngine:torsoAngularVelocityY");
  DECLARE_PLOT("module:WalkingEngine:torsoAngularVelocityCorrectionY");
  DECLARE_PLOT("module:WalkingEngine:gyroY");
  DECLARE_PLOT("module:WalkingEngine:smoothedGyroY");

  if(theLegMotionSelection.ratios[MotionRequest::walk] > 0.f || theLegMotionSelection.ratios[MotionRequest::stand] > 0.f)
  {
    const LegPosture& targetPosture = theWalkingEngineState.targetLegPosture;

    // compute torso orientation
    bool transition = theLegMotionSelection.ratios[MotionRequest::specialAction] > 0 ||
                      theLegMotionSelection.ratios[MotionRequest::kick] > 0 || theLegMotionSelection.ratios[MotionRequest::getUp] > 0;
    float additionalBodyRotation = (targetPosture.rightOriginToCom.y() - targetPosture.rightOriginToFoot.translation.y() + targetPosture.leftOriginToCom.y() - targetPosture.leftOriginToFoot.translation.y()) * 0.5f;
    additionalBodyRotation *= 1.f / (22.5f - 50.f);
    additionalBodyRotation *= walkComBodyRotation;
    RotationMatrix bodyRotation = RotationMatrix::aroundX(additionalBodyRotation);
    bodyRotation *= theWalkingEngineState.standBodyRotation;

    float angularVelocityCorrection = 0.0f;
    if(!transition && theGroundContactState.contact && theInertialData.gyro.y() != SensorData::off)
    {
      // Buffer the relative rotations of the torso around its y-axis.
      const RotationMatrix relativeRotation = bodyRotation.inverse() * lastBodyRotationMatrix;
      const float relativeRotationY = std::atan2(relativeRotation(0, 2), relativeRotation(2, 2));
      relativeRotations.push_front(relativeRotationY);

      // Calculate the moving average of the gyro measurements.
      if(lastSmoothedGyroY == SensorData::off)
        lastSmoothedGyroY = theInertialData.gyro.y();
      const float smoothedGyro = theInertialData.gyro.y() * gyroSmoothing + lastSmoothedGyroY * (1.0f - gyroSmoothing);

      // Use the difference between the buffered and measured angular velocities of the torso to calculate a
      // relative y-axis angle offset to control the torso's angular velocity.
      const int frameDelay = static_cast<int>(theWalkingEngineState.observerMeasurementDelay / (theFrameInfo.cycleTime * 1000.0f));
      ASSERT(frameDelay < 10);
      if(relativeRotations.size() == 10)
      {
        const float angularVelocityY = (relativeRotations[frameDelay - 1] - relativeRotations[frameDelay]) / theFrameInfo.cycleTime;
        PLOT("module:WalkingEngine:torsoAngularVelocityY", toDegrees(angularVelocityY));
        PLOT("module:WalkingEngine:gyroY", toDegrees(theInertialData.gyro.y()));
        PLOT("module:WalkingEngine:smoothedGyroY", toDegrees(smoothedGyro));
        const float errorY = angularVelocityY - smoothedGyro;
        const float approxDerivative = (errorY - lastGyroErrorY) / theFrameInfo.cycleTime;
        lastGyroErrorY = errorY;
        angularVelocityCorrection = gyroStateGain * errorY + gyroDerivativeGain * approxDerivative;
        PLOT("module:WalkingEngine:torsoAngularVelocityCorrectionY", toDegrees(angularVelocityCorrection));
      }
      lastSmoothedGyroY = smoothedGyro;
    }
    else
    {
      lastSmoothedGyroY = SensorData::off;
      relativeRotations.clear();
      lastGyroErrorY = 0.f;
    }
    lastBodyRotationMatrix = bodyRotation;

    // compute foot position relative to the center of mass
    const Pose3f comToLeftOrigin = Pose3f(bodyRotation, targetPosture.leftOriginToCom).inverse(); // TODO: optimize this by calculating the inverted left/rightOriginToCom pose directly
    const Pose3f comToRightOrigin = Pose3f(bodyRotation, targetPosture.rightOriginToCom).inverse();
    const Pose3f comToLeftFoot = Pose3f(comToLeftOrigin).conc(targetPosture.leftOriginToFoot);
    const Pose3f comToRightFoot = Pose3f(comToRightOrigin).conc(targetPosture.rightOriginToFoot);

    // try to guess bodyToCom
    const Vector3f averageComToAnkle = (comToLeftFoot.translation + comToRightFoot.translation) * 0.5f;
    const Vector3f bodyToComOffset = lastAverageComToAnkle != Vector3f::Zero() ? Vector3f((averageComToAnkle - lastAverageComToAnkle) * 0.4f) : Vector3f::Zero();
    lastAverageComToAnkle = averageComToAnkle;
    bodyToCom += bodyToComOffset;

    JointRequest joints;
    // add head joints
    joints.angles[Joints::headYaw] = theHeadJointRequest.pan == JointRequest::off ? theJointAngles.angles[Joints::headYaw] : theHeadJointRequest.pan;
    joints.angles[Joints::headPitch] = theHeadJointRequest.tilt == JointRequest::off ? theJointAngles.angles[Joints::headPitch] :  theHeadJointRequest.tilt;
    //add arm joints
    std::copy(theArmJointRequest.angles.begin() + Joints::firstArmJoint, theArmJointRequest.angles.begin() + Joints::firstLegJoint, joints.angles.begin() + Joints::firstArmJoint);

    RobotModel robotModel;
    // find bodyToCom
    for(int i = 0; ; ++i)
    {
      const Pose3f bodyToLeftFoot = Pose3f(bodyToCom).conc(comToLeftFoot);
      const Pose3f bodyToRightFoot = Pose3f(bodyToCom).conc(comToRightFoot);
      (void)InverseKinematic::calcLegJoints(bodyToLeftFoot, bodyToRightFoot, theStandBodyRotation.bodyRotation, joints, theRobotDimensions, 0.5f);
      robotModel.setJointData(joints, theRobotDimensions, theMassCalibration); // TODO: improve this by not calculating the whole limb/mass model in each iteration

      Vector3f delta = (robotModel.centerOfMass - bodyToCom) * 1.3f;
      bodyToCom += delta;
      if(i >= 7 || (std::abs(delta.x()) < 0.05 && std::abs(delta.y()) < 0.05 && std::abs(delta.z()) < 0.05))
      {
        bodyToCom = robotModel.centerOfMass;
        break;
      }
    }

    // Correct the torso's angular velocity.
    const Pose3f torsoInLeftFoot = robotModel.soleLeft.inverse().rotateY(angularVelocityCorrection);
    const Pose3f torsoInRightFoot = robotModel.soleRight.inverse().rotateY(angularVelocityCorrection);
    const Pose3f leftFootInTorso = torsoInLeftFoot.inverse();
    const Pose3f rightFootInTorso = torsoInRightFoot.inverse();
    (void)InverseKinematic::calcLegJoints(leftFootInTorso, rightFootInTorso, theStandBodyRotation.bodyRotation, legJointRequest, theRobotDimensions, 0.5f);

    // set stiffness
    legJointRequest.stiffnessData.stiffnesses.fill(StiffnessData::useDefault);
    legJointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = theDamageConfigurationBody.weakLeftLeg ? 100 : anklePitchStiffness;
    legJointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll] = ankleRollStiffness;
    legJointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = theDamageConfigurationBody.weakRightLeg ? 100 : anklePitchStiffness;
    legJointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll] = ankleRollStiffness;
  }
  else
  {
    bodyToCom = Vector3f::Zero();
    lastAverageComToAnkle = Vector3f::Zero();
    lastBodyRotationMatrix = RotationMatrix();
    relativeRotations.clear();
    lastSmoothedGyroY = SensorData::off;
    lastGyroErrorY = 0.f;
  }
}

void WalkingEngineLegProvider::generateWalkingEngineOutput(WalkingEngineOutput& walkingEngineOutput)
{
  walkingEngineOutput.upcomingOdometryOffsetValid = true;

  if(theLegMotionSelection.ratios[MotionRequest::walk] > 0.f || theLegMotionSelection.ratios[MotionRequest::stand] > 0.f)
    walkingEngineOutput = theWalkingEngineState.unfinishedOutput;
  else
  {
    walkingEngineOutput.standing = false;
    walkingEngineOutput.speed = Pose2f();
    walkingEngineOutput.odometryOffset = Pose2f();
    walkingEngineOutput.upcomingOdometryOffset = Pose2f();
    walkingEngineOutput.isLeavingPossible = true;
    walkingEngineOutput.executedWalk = WalkRequest();
    walkingEngineOutput.supportFoot = WalkingEngineOutput::SupportFoot::both;
  }
}
