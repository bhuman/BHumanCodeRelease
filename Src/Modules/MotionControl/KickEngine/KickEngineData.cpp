/*
 * @file KickEngineData.cpp
 * This file implements a module that creates motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 * @author Philip Reichenberg
 */

#include <cstring>

#include "KickEngineData.h"
#include "Representations/Configuration/JointLimits.h"
#include "Debugging/Plot.h"
#include "Debugging/Modify.h"
#include "Math/Pose3f.h"
#include "Math/RotationMatrix.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Platform/SystemCall.h"
#include "MathBase/Rotation.h"

bool KickEngineData::getMotionIDByName(const KickRequest& kr, const std::vector<KickEngineParameters>& params)
{
  motionID = -1;

  for(unsigned int i = 0; i < params.size(); ++i)
    if(kr.getKickMotionFromName(&params[i].name[0]) == kr.kickMotionType)
    {
      motionID = i;
      return true;
    }
  return false;
}

void KickEngineData::calculateOrigins(const KickRequest& kr, const JointAngles& ja)
{
  if(!wasActive)
  {
    if(!kr.mirror)
    {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footLeft].rotation.inverse() * robotModel.soleLeft.translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footRight].rotation.inverse() * robotModel.soleRight.translation;

      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;

      origins[Phase::leftFootRot] = Vector3f(0, 0, robotModel.limbs[Limbs::footLeft].rotation.getZAngle());
      origins[Phase::rightFootRot] = Vector3f(0, 0, robotModel.limbs[Limbs::footRight].rotation.getZAngle());
      origins[Phase::leftHandRot] = Vector3f(ja.angles[Joints::lElbowYaw], ja.angles[Joints::lElbowRoll], ja.angles[Joints::lWristYaw]);
      origins[Phase::rightHandRot] = Vector3f(ja.angles[Joints::rElbowYaw], ja.angles[Joints::rElbowRoll], ja.angles[Joints::rWristYaw]);
    }
    else
    {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footRight].rotation.inverse() * robotModel.soleRight.translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footLeft].rotation.inverse() * robotModel.soleLeft.translation;
      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;

      origins[Phase::rightHandRot] = Vector3f(-ja.angles[Joints::lElbowYaw], -ja.angles[Joints::lElbowRoll], -ja.angles[Joints::lWristYaw]);
      origins[Phase::leftHandRot] = Vector3f(-ja.angles[Joints::rElbowYaw], -ja.angles[Joints::rElbowRoll], -ja.angles[Joints::rWristYaw]);

      origins[Phase::leftFootRot] = Vector3f(0, 0, ja.angles[Joints::rHipYawPitch]);
      origins[Phase::rightFootRot] = Vector3f(0, 0, ja.angles[Joints::lHipYawPitch]);

      origins[Phase::leftFootTra].y() *= -1;
      origins[Phase::rightFootTra].y() *= -1;
      origins[Phase::leftArmTra].y() *= -1;
      origins[Phase::rightArmTra].y() *= -1;
    }
  }
  else
  {
    for(int i = 0; i < Phase::numOfLimbs; ++i)
      origins[i] = currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].controlPoints[i][2];
  }
}

void KickEngineData::calcPhaseState()
{
  phase = static_cast<float>(timeSinceTimestamp) / static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration);
}

bool KickEngineData::checkPhaseTime(const FrameInfo& frame, const JointAngles& ja)
{
  timeSinceTimestamp = frame.getTimeSince(timestamp);

  if(motionID < 0)
    return false;

  //Is our current Keyframe valid?
  if(phaseNumber < currentParameters.numberOfPhases)
  {
    //Our current Keyframe is over
    if(static_cast<unsigned int>(timeSinceTimestamp) >= currentParameters.phaseParameters[phaseNumber].duration)
    {
      phaseNumber++;
      timestamp = frame.time;
      timeSinceTimestamp = frame.getTimeSince(timestamp);
      lastTrajectoryOffset = currentTrajectoryOffset;
      //Do we have a valid Keyframe left?
      if(phaseNumber < currentParameters.numberOfPhases)
      {
        if(currentKickRequest.armsBackFix)
        {
          //Check which Arm shall be moved to the front
          if(lElbowFront)
          {
            Vector3f inverse = currentParameters.phaseParameters[phaseNumber].controlPoints[Phase::leftHandRot][2];
            inverse.x() *= -1.f;
            addDynPoint(DynPoint(Phase::leftHandRot, phaseNumber, inverse));
          }
          if(rElbowFront)
          {
            Vector3f inverse = currentParameters.phaseParameters[phaseNumber].controlPoints[Phase::rightHandRot][2];
            inverse.x() *= -1.f;
            addDynPoint(DynPoint(Phase::rightHandRot, phaseNumber, inverse));
          }
        }

        //Calculate the control points
        if(currentKickRequest.calcDynPoints)
        {
          const auto dynPoints = currentKickRequest.calcDynPoints(phaseNumber);
          for(const auto& dynPoint : dynPoints)
            if(dynPoint.phaseNumber == phaseNumber)
              addDynPoint(dynPoint);
        }
      }
    }
  }
  else if(currentParameters.loop && phaseNumber == currentParameters.numberOfPhases)
  {
    phaseNumber = 0;
    //calculateOrigins(currentKickRequest, ja);
    currentParameters.initFirstPhaseLoop(origins, currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].comTra[2], Vector2f(ja.angles[Joints::headPitch], ja.angles[Joints::headYaw]));

    if(currentKickRequest.calcDynPoints)
    {
      const auto dynPoints = currentKickRequest.calcDynPoints(phaseNumber);
      for(const auto& dynPoint : dynPoints)
        if(dynPoint.phaseNumber == phaseNumber)
          addDynPoint(dynPoint);
    }
  }

  return phaseNumber < currentParameters.numberOfPhases;
}

bool KickEngineData::calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const DamageConfigurationBody& theDamageConfigurationBody)
{
  //Calculate Legs
  if(motionID > -1)
  {
    if(!currentParameters.ignoreHead)
    {
      jointRequest.angles[Joints::headPitch] = head.x();
      jointRequest.angles[Joints::headYaw] = head.y();
    }

    // calc joints
    calcLegJoints(Joints::lHipYawPitch, jointRequest, rd, theDamageConfigurationBody);
    calcLegJoints(Joints::rHipYawPitch, jointRequest, rd, theDamageConfigurationBody);
    simpleCalcArmJoints(Joints::lShoulderPitch, jointRequest, rd, positions[Phase::leftArmTra], positions[Phase::leftHandRot]);
    simpleCalcArmJoints(Joints::rShoulderPitch, jointRequest, rd, positions[Phase::rightArmTra], positions[Phase::rightHandRot]);

    return true;
  }
  else //just set the angles from init
  {
    for(int i = Joints::lShoulderPitch; i < Joints::numOfJoints; ++i)
      jointRequest.angles[i] = lastBalancedJointRequest.angles[i];

    return false;
  }
}

void KickEngineData::calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const DamageConfigurationBody& theDamageConfigurationBody)
{
  const int sign = joint == Joints::lHipYawPitch ? 1 : -1;

  const Vector3f& footPos = (sign > 0) ? positions[Phase::leftFootTra] : positions[Phase::rightFootTra];
  const Vector3f& footRotAng = (sign > 0) ? positions[Phase::leftFootRot] : positions[Phase::rightFootRot];

  const RotationMatrix rotateBodyTilt = RotationMatrix::aroundX(bodyAngle.x()).rotateY(bodyAngle.y());
  Vector3f anklePos = rotateBodyTilt * (footPos - Vector3f(0.f, 0, -theRobotDimensions.footHeight));
  anklePos -= Vector3f(0.f, sign * (theRobotDimensions.yHipOffset), 0);

  //const RotationMatrix zRot = RotationMatrix::aroundZ(footRotAng.z()).rotateX(sign * pi_4);
  //anklePos = zRot * anklePos;
  const float leg0 = 0;//std::atan2(-anklePos.x(), anklePos.y());
  const float diagonal = anklePos.norm();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength -
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * diagonal);
  //if(std::abs(a1) > 1.f) OUTPUT_TEXT("clipped a1");
  a1 = std::abs(a1) > 1.f ? 0.f : std::acos(a1);

  float a2 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength +
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * theRobotDimensions.lowerLegLength);
  //if(std::abs(a2) > 1.f) OUTPUT_TEXT("clipped a2");
  a2 = std::abs(a2) > 1.f ? pi : std::acos(a2);

  const float leg2 = -a1 - std::atan2(anklePos.x(), Vector2f(anklePos.y(), anklePos.z()).norm() * -sgn(anklePos.z()));
  const float leg1 = anklePos.z() == 0.0f ? 0.0f : (std::atan(anklePos.y() / -anklePos.z()));
  const float leg3 = pi - a2;

  const RotationMatrix rotateBecauseOfHip = RotationMatrix::aroundZ(0).rotateX(bodyAngle.x()).rotateY(bodyAngle.y());
  //calculate inverse foot rotation so that they are flat to the ground
  RotationMatrix footRot = RotationMatrix::aroundX(leg1).rotateY(leg2 + leg3);
  footRot = footRot.inverse() /* * zRot*/ * rotateBecauseOfHip;

  //and add additional foot rotation (which is probably not flat to the ground)
  const float leg4 = std::atan2(footRot(0, 2), footRot(2, 2)) + footRotAng.y();
  const float leg5 = std::asin(-footRot(1, 2)) + footRotAng.x();

  jointRequest.angles[joint] = leg0;
  jointRequest.angles[joint + 1] = (/*-pi_4 * sign + */leg1);
  jointRequest.angles[joint + 2] = leg2;
  jointRequest.angles[joint + 3] = leg3;
  jointRequest.angles[joint + 4] = leg4;
  jointRequest.angles[joint + 5] = leg5;

  //quick hack which allows calibration, but works only if the left foot is the support foot in .kmc file
  Vector2f tiltCalibration = currentKickRequest.mirror ? theDamageConfigurationBody.startTiltRight : theDamageConfigurationBody.startTiltLeft;
  if(currentKickRequest.mirror)
    tiltCalibration.x() *= -1.f;
  jointRequest.angles[Joints::lAnkleRoll] += tiltCalibration.x();
  jointRequest.angles[Joints::lAnklePitch] += tiltCalibration.y();
}

void KickEngineData::calcOdometryOffset(const RobotModel& theRobotModel)
{
  Pose3f ankleInAnkle;
  //quick hack to compute support foot, could be wrong if the motion is using the right foot as support foot as default
  if(currentKickRequest.mirror)
    ankleInAnkle = theRobotModel.limbs[Limbs::ankleRight].inverse() * theRobotModel.limbs[Limbs::ankleLeft];
  else
    ankleInAnkle = theRobotModel.limbs[Limbs::ankleLeft].inverse() * theRobotModel.limbs[Limbs::ankleRight];

  Pose2f currentOdometry(ankleInAnkle.translation.x() * 0.5f, ankleInAnkle.translation.y() * 0.5f);

  odometryOutput = currentOdometry - lastOdometry;
  if(phase == 0)
    odometryOutput += Pose2f(currentParameters.phaseParameters[phaseNumber].odometryOffset.x(), currentParameters.phaseParameters[phaseNumber].odometryOffset.y());

  lastOdometry = currentOdometry;
  odometryOutput.translation.x() = 0.f;
}

void KickEngineData::simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions,
                                         const Vector3f& armPos, const Vector3f& handRotAng)
{
  float sign = joint == Joints::lShoulderPitch ? 1.f : -1.f;

  const Vector3f target = armPos - Vector3f(theRobotDimensions.armOffset.x(), theRobotDimensions.armOffset.y() * sign, theRobotDimensions.armOffset.z());

  jointRequest.angles[joint + 0] = std::atan2(target.z(), target.x());
  jointRequest.angles[joint + 1] = std::atan2(target.y() * sign, std::sqrt(sqr(target.x()) + sqr(target.z())));

  float length2ElbowJoint = Vector3f(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder, 0.f).norm();
  float angle = std::asin(theRobotDimensions.upperArmLength / length2ElbowJoint);

  Pose3f elbow;
  elbow.rotateY(-jointRequest.angles[joint + 0])
  .rotateZ(jointRequest.angles[joint + 1])
  .translate(length2ElbowJoint, 0.f, 0.f)
  .rotateZ(-angle)
  .translate(theRobotDimensions.yOffsetElbowToShoulder, 0.f, 0.f);

  jointRequest.angles[joint + 0] = std::atan2(elbow.translation.z(), elbow.translation.x());
  jointRequest.angles[joint + 1] = std::atan2(elbow.translation.y(), std::sqrt(sqr(elbow.translation.x()) + sqr(elbow.translation.z())));
  jointRequest.angles[joint + 0] = (jointRequest.angles[joint + 0] < pi) ? jointRequest.angles[joint + 0] : 0_deg; //clip special

  jointRequest.angles[joint + 0] *= -1.f;
  jointRequest.angles[joint + 1] *= sign;
  jointRequest.angles[joint + 2] = handRotAng.x();
  jointRequest.angles[joint + 3] = handRotAng.y();
  jointRequest.angles[joint + 4] = handRotAng.z();
  jointRequest.angles[joint + 5] = 0.f;
}

void KickEngineData::balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc)
{
  const Pose3f& torso = toLeftSupport ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];
  comRobotModel.setJointData(joints, rd, mc);
  const Vector3f com = torso.rotation.inverse() * comRobotModel.centerOfMass;

  actualDiff = com - ref;

  balanceSum += actualDiff.head<2>();

  float height = comRobotModel.centerOfMass.z() - ref.z();

  const Vector2f balance(
    currentParameters.kpy * (actualDiff.x()) + currentParameters.kiy * balanceSum.x(),
    -currentParameters.kpx * (actualDiff.y()) + -currentParameters.kix * balanceSum.y());

  if(height != 0.f)
  {
    bodyAngle.x() = (balance.y() != 0) ? std::atan2((balance.y()), height) : 0;
    bodyAngle.y() = (balance.x() != 0) ? std::atan2((balance.x()), height) : 0;
  }

  lastCom = actualDiff;
}

void KickEngineData::mirrorIfNecessary(JointRequest& joints)
{
  if(currentKickRequest.mirror)
  {
    const JointRequest old = joints;
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::headPitch)
        continue;

      joints.angles[i] = old.mirror(static_cast<Joints::Joint>(i), hasSeparateHipYawJoints);
    }
  }
}

void KickEngineData::applyTrajectoryAdjustment(JointRequest& jointRequest, const JointLimits& limits)
{
  std::vector<KickEngineParameters::BoostAngle> currentOffsetList;
  for(KickEngineParameters::JointOffset offset : currentParameters.offsetList)
    if(offset.kickKeyframeLine == phaseNumber)
    {
      currentOffsetList = offset.boost;
      break;
    }
  // first reduce all joints, that have no longer an adjusted trajectory, by adding the missing BoostAngle
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(lastTrajectoryOffset[joint] != 0_deg)
    {
      bool reduce = true;
      for(KickEngineParameters::BoostAngle boost : currentOffsetList)
        reduce &= (currentKickRequest.mirror ? Joints::mirror(boost.joint) : boost.joint) != joint;
      if(reduce)
      {
        KickEngineParameters::BoostAngle reduceJoint;
        reduceJoint.angle = 0_deg;
        reduceJoint.joint = joint;
        reduceJoint.mode = KickEngineParameters::BoostAngle::cosine;
        currentOffsetList.emplace_back(reduceJoint);
      }
    }
  }
  // calculate current offsets
  currentTrajectoryOffset.fill(0_deg);
  for(KickEngineParameters::BoostAngle boost : currentOffsetList)
    currentTrajectoryOffset[(currentKickRequest.mirror ? Joints::mirror(boost.joint) : boost.joint)] = interpolate(lastTrajectoryOffset[(currentKickRequest.mirror ? Joints::mirror(boost.joint) : boost.joint)], boost.angle, phase, boost.mode);

  // apply offsets
  FOREACH_ENUM(Joints::Joint, joint)
  {
    const Angle newOffset = limits.limits[joint].limit(jointRequest.angles[joint] + currentTrajectoryOffset[joint]) - jointRequest.angles[joint];
    const Rangea clip(currentTrajectoryOffset[joint] > 0_deg ? 0_deg : currentTrajectoryOffset[joint], currentTrajectoryOffset[joint] < 0_deg ? 0_deg : currentTrajectoryOffset[joint]);
    currentTrajectoryOffset[joint] = clip.limit(newOffset);
    jointRequest.angles[joint] += currentTrajectoryOffset[joint];
  }
}

Angle KickEngineData::interpolate(Angle from, Angle to, float currentTime, KickEngineParameters::BoostAngle::InterpolationMode mode)
{
  if(mode == KickEngineParameters::BoostAngle::square)
    return Angle((static_cast<float>(to - from)) * ((currentTime * currentTime)) + static_cast<float>(from));
  else if(mode == KickEngineParameters::BoostAngle::cosine)
    return Angle(from + (to - from) * (1.f + std::cos(Constants::pi * phase / 1.f - Constants::pi)) / 2.f);
  else if(mode == KickEngineParameters::BoostAngle::linear)
    return from * (1.f - currentTime) + to * currentTime;
  else return 0_deg;
}

void KickEngineData::addGyroBalance(JointRequest& jointRequest, const JointLimits& jointLimits, const InertialData& id, const RobotStableState& theRobotStableState)
{
  if(id.gyro.y() != 0 && id.gyro.x() != 0)
  {
    //Low-pass filter
    gyro = id.gyro.head<2>().cast<float>() * 0.3f + 0.7f * gyro;

    //some clipping
    for(int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
    {
      jointLimits.limits[i].clamp(jointRequest.angles[i]);
    }

    const JointRequest balancedJointRequest = jointRequest;

    //calculate the commandedVelocity
    float commandedVelocity[4];
    //y-velocity if left leg is support
    commandedVelocity[0] = (balancedJointRequest.angles[Joints::lHipPitch] - lastBalancedJointRequest.angles[Joints::lHipPitch]) / cycleTime;
    //y-velocity if right leg is support
    commandedVelocity[1] = (balancedJointRequest.angles[Joints::rHipPitch] - lastBalancedJointRequest.angles[Joints::rHipPitch]) / cycleTime;
    //x-velocity if left leg is support
    commandedVelocity[2] = (balancedJointRequest.angles[Joints::lHipRoll] - lastBalancedJointRequest.angles[Joints::lHipRoll]) / cycleTime;
    //x-velocity if right leg is support
    commandedVelocity[3] = (balancedJointRequest.angles[Joints::rHipRoll] - lastBalancedJointRequest.angles[Joints::rHipRoll]) / cycleTime;

    //calculate disturbance from measured velocity and commanded velocity
    // y-velocity if left leg is support
    float gyroVelYLeft = (gyro.y() + commandedVelocity[0] - lastGyroLeft.y()) / cycleTime;
    lastGyroLeft.y() = gyro.y() + commandedVelocity[0];
    //y-velocity if right leg is support
    float gyroVelYRight = (gyro.y() + commandedVelocity[1] - lastGyroRight.y()) / cycleTime;
    lastGyroRight.y() = gyro.y() + commandedVelocity[1];
    //x-velocity if left leg is support
    float gyroVelXLeft = (gyro.x() + commandedVelocity[2] - lastGyroLeft.x()) / cycleTime;
    lastGyroLeft.x() = gyro.x() + commandedVelocity[2];
    //x-velocity if right leg is support
    float gyroVelXRight = (gyro.x() + commandedVelocity[3] - lastGyroRight.x()) / cycleTime;
    lastGyroRight.x() = gyro.x() + commandedVelocity[3];

    //calculate control variable with PID-Control
    float calcVelocity[4];
    //y if left support
    calcVelocity[0] = -gyroP.y() * (gyro.y() + commandedVelocity[0]) - gyroD.y() * gyroVelYLeft - gyroI.y() * (gyroErrorLeft.y());
    //y if right support
    calcVelocity[1] = -gyroP.y() * (gyro.y() + commandedVelocity[1]) - gyroD.y() * gyroVelYRight - gyroI.y() * (gyroErrorRight.y());
    //x if left support
    calcVelocity[2] = -gyroP.x() * (gyro.x() + commandedVelocity[2]) + gyroD.x() * gyroVelXLeft + gyroI.x() * gyroErrorLeft.x();
    //x if right support
    calcVelocity[3] = -gyroP.x() * (gyro.x() - commandedVelocity[3]) + gyroD.x() * gyroVelXRight + gyroI.x() * gyroErrorRight.x();

    bool support = (currentKickRequest.mirror) ? !toLeftSupport : toLeftSupport;

    if(support)    //last support Leg was left
    {
      //y
      jointRequest.angles[Joints::rHipPitch] += calcVelocity[0] * cycleTime;
      jointRequest.angles[Joints::lHipPitch] += calcVelocity[0] * cycleTime;
      jointRequest.angles[Joints::lAnklePitch] += calcVelocity[0] * cycleTime;
      jointRequest.angles[Joints::rAnklePitch] += calcVelocity[0] * cycleTime;
      //x
      jointRequest.angles[Joints::lHipRoll] += calcVelocity[2] * cycleTime;
      jointRequest.angles[Joints::rHipRoll] += calcVelocity[2] * cycleTime;
      jointRequest.angles[Joints::lAnkleRoll] -= calcVelocity[2] * cycleTime;
      jointRequest.angles[Joints::rAnkleRoll] -= calcVelocity[2] * cycleTime;
    }
    else //if(toRightSupport)
    {
      //y
      jointRequest.angles[Joints::rHipPitch] += calcVelocity[1] * cycleTime;
      jointRequest.angles[Joints::lHipPitch] += calcVelocity[1] * cycleTime;
      jointRequest.angles[Joints::lAnklePitch] += calcVelocity[1] * cycleTime;
      jointRequest.angles[Joints::rAnklePitch] += calcVelocity[1] * cycleTime;

      //x
      jointRequest.angles[Joints::lHipRoll] += calcVelocity[3] * cycleTime;
      jointRequest.angles[Joints::rHipRoll] += calcVelocity[3] * cycleTime;
      jointRequest.angles[Joints::lAnkleRoll] -= calcVelocity[3] * cycleTime;
      jointRequest.angles[Joints::rAnkleRoll] -= calcVelocity[3] * cycleTime;
    }

    // Arm Balancing in roll direction
    // Positive -> rShoulderRoll shall go out further
    // Negative -> lShoulderRoll shall go out further
    const float armSign = currentKickRequest.mirror ? 1.f : -1.f;
    const float comInSupportFootX = theRobotStableState.comInFeet[currentKickRequest.mirror ? Legs::right : Legs::left].outerSide;

    // Get ratio to interpolate with
    const float armBalancingRatioX = currentParameters.getArmCompensationRatio(phaseNumber, phase);
    const Rangea maxAdjustment(-currentParameters.armBalancing.maxAdjustment, currentParameters.armBalancing.maxAdjustment);
    const float removeArmBalancingValueX = maxAdjustment.limit(armCompensation);
    const Rangef& upperRange = currentParameters.armBalancing.upperRange;
    const Rangef& lowerRange = currentParameters.armBalancing.lowerRange;
    // Calc current error
    const float comInSupportFootErrorX = (mapToRange(comInSupportFootX, lowerRange.min, lowerRange.max, static_cast<float>(-currentParameters.armBalancing.maxAdjustment), 0.f) +
                                          mapToRange(comInSupportFootX, upperRange.min, upperRange.max, 0.f, static_cast<float>(currentParameters.armBalancing.maxAdjustment))) *
                                         armSign;
    // Integrate error and apply correction
    armCompensation = maxAdjustment.limit(armCompensation + (removeArmBalancingValueX * (1.f - armBalancingRatioX) + comInSupportFootErrorX * armBalancingRatioX) * currentParameters.armBalancing.i);
    jointRequest.angles[armCompensation > 0 ? Joints::lShoulderRoll : Joints::rShoulderRoll] += armCompensation;

    gyroErrorLeft += lastGyroLeft;
    gyroErrorRight += lastGyroRight;
    lastBalancedJointRequest = balancedJointRequest;
  }
}

void KickEngineData::addDynPoint(const DynPoint& dynPoint)
{
  Vector3f d = dynPoint.translation;

  const int phaseNumber = dynPoint.phaseNumber;
  const int limb = dynPoint.limb;

  if(dynPoint.duration > 0)
    currentParameters.phaseParameters[phaseNumber].duration = dynPoint.duration;

  currentParameters.phaseParameters[phaseNumber].odometryOffset = dynPoint.odometryOffset;

  const Vector3f& cubePoint = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  const Vector3f diff = cubePoint - d;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -= diff;
  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] -= diff;

  if(phaseNumber < currentParameters.numberOfPhases - 1)
  {
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

    float factor = static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration) /
                   static_cast<float>(currentParameters.phaseParameters[phaseNumber + 1].duration);
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  }
}

void KickEngineData::ModifyData(JointRequest& jointRequest)
{
  wasActive = motionID > -1;

  MODIFY("module:KickEngine:px", gyroP.x());
  MODIFY("module:KickEngine:dx", gyroD.x());
  MODIFY("module:KickEngine:ix", gyroI.x());
  MODIFY("module:KickEngine:py", gyroP.y());
  MODIFY("module:KickEngine:dy", gyroD.y());
  MODIFY("module:KickEngine:iy", gyroI.y());

  MODIFY("module:KickEngine:formMode", formMode);
  MODIFY("module:KickEngine:lFootTraOff", limbOff[Phase::leftFootTra]);
  MODIFY("module:KickEngine:rFootTraOff", limbOff[Phase::rightFootTra]);
  MODIFY("module:KickEngine:lFootRotOff", limbOff[Phase::leftFootRot]);
  MODIFY("module:KickEngine:rFootRotOff", limbOff[Phase::rightFootRot]);
  MODIFY("module:KickEngine:lHandTraOff", limbOff[Phase::leftArmTra]);
  MODIFY("module:KickEngine:rHandTraOff", limbOff[Phase::rightArmTra]);
  MODIFY("module:KickEngine:lHandRotOff", limbOff[Phase::leftHandRot]);
  MODIFY("module:KickEngine:rHandRotOff", limbOff[Phase::rightHandRot]);

  //Plot com stabilizing
  PLOT("module:KickEngine:comY", robotModel.centerOfMass.y());
  PLOT("module:KickEngine:diffY", actualDiff.y());
  PLOT("module:KickEngine:refY", ref.y());

  PLOT("module:KickEngine:comX", robotModel.centerOfMass.x());
  PLOT("module:KickEngine:diffX", actualDiff.x());
  PLOT("module:KickEngine:refX", ref.x());

  PLOT("module:KickEngine:lastDiffY", toDegrees(lastBody.y()));
  PLOT("module:KickEngine:bodyErrorY", toDegrees(bodyError.y()));

  for(int i = 0; i < Phase::numOfLimbs; i++)
  {
    const int stiffness = limbOff[i] ? 0 : 100;

    switch(static_cast<Phase::Limb>(i))
    {
      case Phase::leftFootTra:
        jointRequest.stiffnessData.stiffnesses[Joints::lHipRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lHipPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lKneePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll] = stiffness;
        break;
      case Phase::rightFootTra:
        jointRequest.stiffnessData.stiffnesses[Joints::rHipRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rHipPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rKneePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll] = stiffness;
        break;
      case Phase::leftFootRot:
        jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rHipYawPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch] = stiffness;
        break;
      case Phase::rightFootRot:
        jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rHipYawPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lHipYawPitch] = stiffness;
        break;
      case Phase::leftArmTra:
        jointRequest.stiffnessData.stiffnesses[Joints::lShoulderPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lShoulderRoll] = stiffness;
        break;
      case Phase::rightArmTra:
        jointRequest.stiffnessData.stiffnesses[Joints::rShoulderPitch] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rShoulderRoll] = stiffness;
        break;
      case Phase::leftHandRot:
        jointRequest.stiffnessData.stiffnesses[Joints::lElbowRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lElbowYaw] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lWristYaw] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::lHand] = stiffness;
        break;
      case Phase::rightHandRot:
        jointRequest.stiffnessData.stiffnesses[Joints::rElbowRoll] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rElbowYaw] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rWristYaw] = stiffness;
        jointRequest.stiffnessData.stiffnesses[Joints::rHand] = stiffness;
        break;
    }
  }
}

void KickEngineData::calcPositions()
{
  for(int i = 0; i < Phase::numOfLimbs; ++i)
    positions[i] = currentParameters.getPosition(phase, phaseNumber, i);
  if(!currentParameters.ignoreHead)
    head = currentParameters.getHeadRefPosition(phase, phaseNumber);

  ref << currentParameters.getComRefPosition(phase, phaseNumber),
      (toLeftSupport) ? positions[Phase::leftFootTra].z() : positions[Phase::rightFootTra].z();
}

void KickEngineData::initData(const FrameInfo& frame, const KickRequest& kr, const std::vector<KickEngineParameters>& params,
                              const JointAngles& ja, JointRequest& jointRequest, const RobotDimensions& rd,
                              const MassCalibration& mc, const DamageConfigurationBody& theDamageConfigurationBody)
{
  VERIFY(getMotionIDByName(kr, params));

  phase = 0.f;
  phaseNumber = 0;
  timestamp = frame.time;
  currentParameters = params[motionID];
  toLeftSupport = currentParameters.standLeft;
  ref = Vector3f::Zero();
  actualDiff = ref;
  calculateOrigins(kr, ja);
  currentParameters.initFirstPhase(origins, Vector2f(ja.angles[Joints::headPitch], (kr.mirror) ? -ja.angles[Joints::headYaw] : ja.angles[Joints::headYaw]));
  calcPositions();

  float angleY = toLeftSupport ? -robotModel.limbs[Limbs::footLeft].rotation.inverse().getYAngle() : -robotModel.limbs[Limbs::footRight].rotation.inverse().getYAngle();
  float angleX = toLeftSupport ? -robotModel.limbs[Limbs::footLeft].rotation.inverse().getXAngle() : -robotModel.limbs[Limbs::footRight].rotation.inverse().getXAngle();
  if(kr.mirror)
    angleX *= -1.f;

  bodyAngle = Vector2f(angleX, angleY);
  calcJoints(jointRequest, rd, theDamageConfigurationBody);
  comRobotModel.setJointData(jointRequest, rd, mc);
  const Pose3f& torso = toLeftSupport ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];
  const Vector3f com = torso.rotation.inverse() * comRobotModel.centerOfMass;

  //this calculates inverse of pid com control -> getting com to max pos at start -> being rotated as before engine
  float foot = toLeftSupport ? origins[Phase::leftFootTra].z() : origins[Phase::rightFootTra].z();
  float height = comRobotModel.centerOfMass.z() - foot;

  balanceSum.x() = std::tan(angleY - Constants::pi) * height;
  balanceSum.x() /= currentParameters.kiy;
  balanceSum.y() = std::tan(angleX - Constants::pi) * height;
  balanceSum.y() /= -currentParameters.kix;

  currentParameters.initFirstPhaseLoop(origins, Vector2f(com.x(), com.y()), Vector2f(ja.angles[Joints::headPitch], (kr.mirror) ? -ja.angles[Joints::headYaw] : ja.angles[Joints::headYaw]));

  if(!wasActive)
  {
    // origin = Vector2f::Zero();
    gyro = Vector2f::Zero();
    lastGyroLeft = Vector2f::Zero();
    lastGyroRight = Vector2f::Zero();
    gyroErrorLeft = Vector2f::Zero();
    gyroErrorRight = Vector2f::Zero();
    bodyError = Vector2f::Zero();
    lastBody = Vector2f::Zero();
    lastCom = Vector3f::Zero();
    armCompensation = 0.f;

    for(int i = 0; i < Joints::numOfJoints; i++)
    {
      lastBalancedJointRequest.angles[i] = ja.angles[i];
    }
  }
  if(kr.calcDynPoints)
  {
    const auto dynPoints = kr.calcDynPoints(phaseNumber);
    for(const auto& dynPoint : dynPoints)
      if(dynPoint.phaseNumber == phaseNumber)
        addDynPoint(dynPoint);
  }

  lElbowFront = origins[Phase::leftHandRot].x() > pi_4;
  rElbowFront = origins[Phase::rightHandRot].x() < -pi_4;

  if(kr.armsBackFix)  //quick hack to not break arms while they are on the back
  {
    if(lElbowFront)
      addDynPoint(DynPoint(Phase::leftHandRot, 0, Vector3f(pi_2, -pi_4, 0)));

    if(rElbowFront)
      addDynPoint(DynPoint(Phase::rightHandRot, 0, Vector3f(-pi_2, pi_4, 0)));
  }
}

bool KickEngineData::activateNewMotion(const KickRequest& br)
{
  return (!wasActive || br.kickMotionType != currentKickRequest.kickMotionType) && br.kickMotionType != KickRequest::none;
}

bool KickEngineData::sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, JointRequest& jointRequest, const JointRequest& theJointRequest, const FrameInfo& frame)
{
  if(compensate)
  {
    if(!startComp)
    {
      startComp = true; // kickEngine overrides it back to false, if it is still interpolation into active
      timestamp = frame.time;
      gyro = Vector2f::Zero();
      lastGyroLeft = Vector2f::Zero();
      lastGyroRight = Vector2f::Zero();
      gyroErrorLeft = Vector2f::Zero();
      gyroErrorRight = Vector2f::Zero();
      bodyError = Vector2f::Zero();
      lastBody = Vector2f::Zero();
      lastCom = Vector3f::Zero();
      armCompensation = 0.f;
      motionID = -1;

      lastBalancedJointRequest.angles = theJointRequest.angles;
      compensatedJoints.angles = theJointRequest.angles;
    }
    jointRequest.stiffnessData = theJointRequest.stiffnessData;

    int time = frame.getTimeSince(timestamp);
    if((std::abs(id.gyro.x()) < 0.1f && std::abs(id.gyro.y()) < 0.1f && time > 200) || time > 1000)
    {
      compensate = false;
      compensated = true;
      return true;
    }
    else
      return false;
  }

  return true;
}
