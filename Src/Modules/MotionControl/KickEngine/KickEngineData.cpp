/*
 * @file KickEngineData.cpp
 * This file implements a module that creates motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#include <cstring>

#include "KickEngineData.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Motion/InverseKinematic.h"

using namespace std;

bool KickEngineData::getMotionIDByName(const MotionRequest& motionRequest, const std::vector<KickEngineParameters>& params)
{
  motionID = -1;

  for(unsigned int i = 0; i < params.size(); ++i)
    if(motionRequest.kickRequest.getKickMotionFromName(&params[i].name[0]) == motionRequest.kickRequest.kickMotionType)
    {
      motionID = i;
      return true;
    }

  return false;
}

void KickEngineData::calculateOrigins(const RobotDimensions& theRobotDimensions, const KickRequest& kr)
{
  if(!wasActive)
  {
    if(!kr.mirror)
    {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footLeft].translated(0.f, 0.f, -theRobotDimensions.footHeight).translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footRight].translated(0.f, 0.f, -theRobotDimensions.footHeight).translation;
      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;

      origins[Phase::leftFootRot] = robotModel.limbs[Limbs::footLeft].rotation.getPackedAngleAxisFaulty();
      origins[Phase::rightFootRot] = robotModel.limbs[Limbs::footRight].rotation.getPackedAngleAxisFaulty();
      origins[Phase::leftHandRot] = robotModel.limbs[Limbs::foreArmLeft].rotation.getPackedAngleAxisFaulty();
      origins[Phase::rightHandRot] = robotModel.limbs[Limbs::foreArmRight].rotation.getPackedAngleAxisFaulty();
    }
    else
    {
      origins[Phase::leftFootTra] = robotModel.limbs[Limbs::footRight].translated(0.f, 0.f, -theRobotDimensions.footHeight).translation;
      origins[Phase::rightFootTra] = robotModel.limbs[Limbs::footLeft].translated(0.f, 0.f, -theRobotDimensions.footHeight).translation;
      origins[Phase::leftArmTra] = robotModel.limbs[Limbs::foreArmRight].translation;
      origins[Phase::rightArmTra] = robotModel.limbs[Limbs::foreArmLeft].translation;

      RotationMatrix mirror;
      mirror.col(0) = Vector3f(1, 0, 0);
      mirror.col(1) = Vector3f(0, -1, 0);
      mirror.col(2) = Vector3f(0, 0, 1);

      origins[Phase::leftHandRot] = (robotModel.limbs[Limbs::foreArmRight].rotation * mirror).getPackedAngleAxisFaulty();
      origins[Phase::rightHandRot] = (robotModel.limbs[Limbs::foreArmLeft].rotation * mirror).getPackedAngleAxisFaulty();
      origins[Phase::leftFootRot] = (robotModel.limbs[Limbs::footRight].rotation * mirror).getPackedAngleAxisFaulty();
      origins[Phase::rightFootRot] = (robotModel.limbs[Limbs::footLeft].rotation * mirror).getPackedAngleAxisFaulty();

      origins[Phase::leftFootTra].y() *= -1;
      origins[Phase::rightFootTra].y() *= -1;
      origins[Phase::leftArmTra].y() *= -1;
      origins[Phase::rightArmTra].y() *= -1;
    }
  }
  else
  {
    for(int i = 0; i < Phase::numOfLimbs; ++i)
    {
      origins[i] = currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].controlPoints[i][2];
    }
  }
}

bool KickEngineData::checkPhaseTime(const FrameInfo& frame, const RobotDimensions& rd, const JointAngles& ja, const TorsoMatrix& torsoMatrix)
{
  timeSinceTimeStamp = frame.getTimeSince(timeStamp);

  if(motionID > -1)
  {
    if(phaseNumber < currentParameters.numberOfPhases)
    {
      if(static_cast<unsigned int>(timeSinceTimeStamp) > currentParameters.phaseParameters[phaseNumber].duration)
      {
        phaseNumber++;
        timeStamp = frame.time;
        timeSinceTimeStamp = frame.getTimeSince(timeStamp);
        if(currentKickRequest.dynamical && !currentKickRequest.dynPoints.empty())
          for(unsigned int i = 0; i < currentKickRequest.dynPoints.size(); i++)
            if(currentKickRequest.dynPoints[i].phaseNumber == phaseNumber)
              addDynPoint(currentKickRequest.dynPoints[i], rd, torsoMatrix);
      }
    }
    else if(currentParameters.loop && phaseNumber == currentParameters.numberOfPhases)
    {
      phaseNumber = 0;
      calculateOrigins(rd, currentKickRequest);
      currentParameters.initFirstPhaseLoop(origins, currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].comTra[2], Vector2f(ja.angles[Joints::headPitch], ja.angles[Joints::headYaw]));
      if(currentKickRequest.dynamical && !currentKickRequest.dynPoints.empty())
        for(unsigned int i = 0; i < currentKickRequest.dynPoints.size(); i++)
          if(currentKickRequest.dynPoints[i].phaseNumber == phaseNumber)
            addDynPoint(currentKickRequest.dynPoints[i], rd, torsoMatrix);
    }

    return (phaseNumber < currentParameters.numberOfPhases);
  }
  else
    return false;
}

bool KickEngineData::calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const HeadJointRequest& hr)
{
  //Calculate Legs
  if(motionID > -1)
  {
    if(!currentParameters.ignoreHead)
    {
      jointRequest.angles[Joints::headPitch] = head.x();
      jointRequest.angles[Joints::headYaw] = head.y();
    }
    else
    {
      jointRequest.angles[Joints::headYaw] = hr.pan == JointAngles::off ? Angle(0) : (!currentKickRequest.mirror ? hr.pan : -hr.pan);
      jointRequest.angles[Joints::headPitch] = hr.tilt == JointAngles::off ? Angle(0) : hr.tilt;
    }

    calcLegJoints(Joints::lHipYawPitch, jointRequest, rd);
    calcLegJoints(Joints::rHipYawPitch, jointRequest, rd);

    jointRequest.angles[Joints::lHipRoll] += comOffset.x();
    jointRequest.angles[Joints::rHipRoll] += comOffset.x();

    jointRequest.angles[Joints::rHipPitch] += comOffset.y();
    jointRequest.angles[Joints::lHipPitch] += comOffset.y();

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

void KickEngineData::calcLegJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions)
{
  const int sign = joint == Joints::lHipYawPitch ? 1 : -1;
  float leg0, leg1, leg2, leg3, leg4, leg5;

  const Vector3f& footPos = (sign > 0) ? positions[Phase::leftFootTra] : positions[Phase::rightFootTra];
  const Vector3f& footRotAng = (sign > 0) ? positions[Phase::leftFootRot] : positions[Phase::rightFootRot];

  RotationMatrix rotateBodyTilt = RotationMatrix::aroundX(comOffset.x());
  Vector3f anklePos = rotateBodyTilt * footPos;
  //we need just the leg length x and y have to stay the same
  anklePos.topRows(2) = footPos.topRows(2);
  // for the translation of the footpos we only need to translate the anklepoint, which is the intersection of the axis leg4 and leg5
  // the rotation of the foot will be made later by rotating the footpos around the anklepoint
  anklePos -= Vector3f(0.f, sign * (theRobotDimensions.yHipOffset), -theRobotDimensions.footHeight);

  RotationMatrix rotateBecauseOfHip = RotationMatrix::aroundZ(footRotAng.z()).rotateX(-sign * pi_4);

  anklePos = rotateBecauseOfHip * anklePos;

  float diagonal = anklePos.norm();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength -
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * diagonal);
  //if(abs(a1) > 1.f) OUTPUT_TEXT("clipped a1");
  a1 = abs(a1) > 1.f ? 0.f : acos(a1);

  float a2 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength +
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * theRobotDimensions.lowerLegLength);
  //if(abs(a2) > 1.f) OUTPUT_TEXT("clipped a2");
  a2 = abs(a2) > 1.f ? pi : acos(a2);

  leg0 = footRotAng.z() * sign;
  leg2 = -a1 - atan2(anklePos.x(), Vector2f(anklePos.y(), anklePos.z()).norm() * -sgn(anklePos.z()));

  leg1 = anklePos.z() == 0.0f ? 0.0f : atan(anklePos.y() / -anklePos.z()) * -sign;
  leg3 = pi - a2;

  RotationMatrix footRot = RotationMatrix::aroundX(leg1 * -sign).rotateY(leg2 + leg3);
  footRot = footRot.inverse() * rotateBecauseOfHip;

  leg5 = asin(-footRot.col(2).y()) * -sign;
  leg4 = atan2(footRot.col(2).x(), footRot.col(2).z());

  leg4 += footRotAng.y();
  leg5 += footRotAng.x();

  jointRequest.angles[joint] = leg0;
  jointRequest.angles[joint + 1] = (-pi_4 + leg1) * -sign;
  jointRequest.angles[joint + 2] = leg2;
  jointRequest.angles[joint + 3] = leg3;
  jointRequest.angles[joint + 4] = leg4;
  jointRequest.angles[joint + 5] = leg5 * -sign;
}

void KickEngineData::simpleCalcArmJoints(const Joints::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3f& armPos, const Vector3f& handRotAng)
{
  float sign = joint == Joints::lShoulderPitch ? 1.f : -1.f;

  Vector3f target = armPos - Vector3f(theRobotDimensions.armOffset.x(),
                                      theRobotDimensions.armOffset.y() * sign,
                                      theRobotDimensions.armOffset.z());

  jointRequest.angles[joint + 0] = atan2(target.z(), target.x());
  jointRequest.angles[joint + 1] = atan2(target.y() * sign, sqrt(sqr(target.x()) + sqr(target.z())));

  float length2ElbowJoint = Vector3f(theRobotDimensions.upperArmLength, theRobotDimensions.yOffsetElbowToShoulder, 0.f).norm();
  float angle = asin(theRobotDimensions.upperArmLength / length2ElbowJoint);

  Pose3f elbow;
  elbow.rotateY(-jointRequest.angles[joint + 0])
    .rotateZ(jointRequest.angles[joint + 1])
    .translate(length2ElbowJoint, 0.f, 0.f)
    .rotateZ(-angle)
    .translate(theRobotDimensions.yOffsetElbowToShoulder, 0.f, 0.f);

  jointRequest.angles[joint + 0] = atan2(elbow.translation.z(), elbow.translation.x());
  jointRequest.angles[joint + 1] = atan2(elbow.translation.y(), sqrt(sqr(elbow.translation.x()) + sqr(elbow.translation.z())));
  jointRequest.angles[joint + 0] = (jointRequest.angles[joint + 0] < pi) ? jointRequest.angles[joint + 0] : Angle(0);  //clip special case

  Pose3f hand(elbow.translation);

  hand.rotateZ(handRotAng.z() * sign)
    .rotateY(handRotAng.y())
    .rotateX(handRotAng.x() * sign)
    .translate(theRobotDimensions.lowerArmLength, 0.f, 0.f);

  calcJointsForElbowPos(elbow.translation, hand.translation, jointRequest, joint, theRobotDimensions);

  jointRequest.angles[joint + 0] = jointRequest.angles[joint + 0] * -1.f;
  jointRequest.angles[joint + 1] = jointRequest.angles[joint + 1] * sign;
  jointRequest.angles[joint + 2] = jointRequest.angles[joint + 2] * sign;
  jointRequest.angles[joint + 3] = jointRequest.angles[joint + 3] * sign;
  jointRequest.angles[joint + 4] = -90_deg * sign;
  jointRequest.angles[joint + 5] = 0.f;
}

void KickEngineData::calcJointsForElbowPos(const Vector3f& elbow, const Vector3f& target, JointAngles& targetJointAngles, int offset, const RobotDimensions& theRobotDimensions)
{
  //set elbow position with the pitch/yaw unit in the shoulder
  targetJointAngles.angles[offset + 0] = std::atan2(elbow.z(), elbow.x());
  targetJointAngles.angles[offset + 1] = std::atan2(elbow.y(), std::sqrt(sqr(elbow.x()) + sqr(elbow.z())));

  //calculate desired elbow "klapp"-angle
  const float c = target.norm(),
              a = theRobotDimensions.upperArmLength,
              b = theRobotDimensions.lowerArmLength;

  //cosine theorem
  float cosAngle = (-sqr(c) + sqr(b) + sqr(a)) / (2.0f * a * b);
  if(cosAngle < -1.0f)
  {
    //      ASSERT(cosAngle > -1.1);
    cosAngle = -1.0f;
  }
  else if(cosAngle > 1.0f)
  {
    ASSERT(cosAngle < 1.1);
    cosAngle = 1.0f;
  }
  targetJointAngles.angles[offset + 3] = std::acos(cosAngle) - pi;

  //calculate hand in elbow coordinate system and calculate last angle
  Pose3f shoulder2Elbow;
  shoulder2Elbow.translate(0, -theRobotDimensions.upperArmLength, 0);
  shoulder2Elbow.rotateX(-(targetJointAngles.angles[offset + 1] - pi / 2.0f));
  shoulder2Elbow.rotateY(targetJointAngles.angles[offset + 0] + pi / 2.0f);
  const Vector3f handInEllbow = shoulder2Elbow * target;

  targetJointAngles.angles[offset + 2] = -(std::atan2(handInEllbow.z(), handInEllbow.x()) + pi / 2.f);
  while(targetJointAngles.angles[offset + 2] > pi)
    targetJointAngles.angles[offset + 2] -= 2 * pi;
  while(targetJointAngles.angles[offset + 2] < -pi)
    targetJointAngles.angles[offset + 2] += 2 * pi;
}

void KickEngineData::balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc)
{
  if(currentParameters.autoComTra)
  {
    setStandLeg(origin.y());
    calculatePreviewCom(ref, origin);
  }
  comRobotModel.setJointData(joints, rd, mc);
  Vector3f com(comRobotModel.centerOfMass);

  Pose3f torso = (toLeftSupport) ? comRobotModel.limbs[Limbs::footLeft] : comRobotModel.limbs[Limbs::footRight];

  com = torso.rotation.inverse() * com;

  actualDiff = com - ref;
  // if(!toLeftSupport)
  //    actualDiff.y() += 15;

  balanceSum += Vector2f(actualDiff.x(), actualDiff.y());

  float height = comRobotModel.centerOfMass.z() - ref.z();

  Vector2f balance;
  balance.y() = -currentParameters.kpx * (actualDiff.y()) + -currentParameters.kix * balanceSum.y() + -currentParameters.kdx * ((actualDiff.y() - lastCom.y()) / cycletime);
  balance.x() = currentParameters.kpy * (actualDiff.x()) + currentParameters.kiy * balanceSum.x() + currentParameters.kdy * ((actualDiff.x() - lastCom.x()) / cycletime);

  if(height != 0.f)
  {
    comOffset.x() = (balance.y() != 0) ? atan2((balance.y()), height) : 0;
    comOffset.y() = (balance.x() != 0) ? atan2((balance.x()), height) : 0;
  }

  lastCom = actualDiff;
}

void KickEngineData::calculatePreviewCom(Vector3f& ref, Vector2f& origin)
{
  ref = Vector3f::Zero();
  float lastPhaseTime = 0.f;
  float future = 0.f;
  int futPhaNum = phaseNumber;
  float currPhaseTime = static_cast<float>(timeSinceTimeStamp);

  for(int j = 1; j < currentParameters.preview + 1; j++)
  {
    if(futPhaNum < currentParameters.numberOfPhases)
    {
      future = static_cast<float>(currPhaseTime + j * cycletime * 1000 - lastPhaseTime) / static_cast<float>(currentParameters.phaseParameters[futPhaNum].duration);

      if(future > 1.f &&  futPhaNum < currentParameters.numberOfPhases - 1)
      {
        future -= 1.f;
        lastPhaseTime = j * cycletime * 1000;
        currPhaseTime = 0.f;
        futPhaNum++;
      }

      if(future > 1.f && futPhaNum + 1 > currentParameters.numberOfPhases - 1)

      {
        if(currentParameters.loop)
        {
          future = 0.f;
          lastPhaseTime = j * cycletime * 1000;
          currPhaseTime = 0.f;
          futPhaNum = 0;
        }
        else
        {
          future = 1.f;
          futPhaNum = currentParameters.numberOfPhases - 1;
        }
      }
    }
    else
    {
      future = 1.f;
      futPhaNum = currentParameters.numberOfPhases - 1;
    }

    Vector3f futPos[2];
    futPos[0] = currentParameters.getPosition(future, futPhaNum, Phase::leftFootTra);
    futPos[1] = currentParameters.getPosition(future, futPhaNum, Phase::rightFootTra);

    Vector3f comRef = Vector3f::Zero();
    Vector2f fOrigin = Vector2f::Zero();

    getCOMReference(futPos[0], futPos[1], comRef, fOrigin);

    ref += comRef;
    origin += fOrigin;
  }
  ref /= static_cast<float>(currentParameters.preview);
  origin /= static_cast<float>(currentParameters.preview);
}

void KickEngineData::setStandLeg(const float& originY)
{
  //If the average standLegPos Reference has reached the current standleg position, lock that
  if(abs(positions[Phase::leftFootTra].y() - (float)standLegPos.average().y()) < 1.f)
  {
    lSupp = true;
    rSupp = false;
  }
  if(abs(positions[Phase::rightFootTra].y() - (float)standLegPos.average().y()) < 1.f)
  {
    rSupp = true;
    lSupp = false;
  }
  //originY goes back to zero when the feet reaches a stable position (example: The distances of the feet to the middle are equal)
  if(abs(originY) < 1.f)
  {
    rSupp = false;
    lSupp = false;
  }
}

void KickEngineData::getCOMReference(const Vector3f& lFootPos, const Vector3f& rFootPos, Vector3f& comRef, Vector2f& origin)
{
  origin = Vector2f(rFootPos.x() + 30.f, 0.f);
  comRef = Vector3f(origin.x(), 0.f, rFootPos.z()); //double support

  if(abs(lFootPos.y() + rFootPos.y()) > 2.f)  //equal distance to the middle
  {
    if((lFootPos.y() + rFootPos.y()) < -2.f  && !lSupp && !rSupp) //calc reference
    {
      origin.y() = origins[Phase::leftFootTra].y();
      comRef = lFootPos - Vector3f(-30.f, 0.f, 0.f); //L-Support
      if(comRef.y() > 0.f) comRef.y() = 0.f;
      standLegPos.push_front(Vector2i((int)comRef.x(), (int)comRef.y()));
    }

    if((lFootPos.y() + rFootPos.y()) > 2.f  && !lSupp && !rSupp)
    {
      origin.y() = origins[Phase::rightFootTra].y();
      comRef = rFootPos - Vector3f(-30.f, 0.f, 0.f); //R-Support
      if(comRef.y() < 0.f) comRef.y() = 0.f;
      standLegPos.push_front(Vector2i((int)comRef.x(), (int)comRef.y()));
    }

    if(lSupp)
    {
      origin.y() = origins[Phase::leftFootTra].y();
      comRef = Vector3f((float)standLegPos.average().x(), (float)standLegPos.average().y() + 10, lFootPos.z());
    }
    if(rSupp)
    {
      origin.y() = origins[Phase::rightFootTra].y();
      comRef = Vector3f((float)standLegPos.average().x(), (float)standLegPos.average().y() - 10, rFootPos.z());
    }
  }
}

void KickEngineData::setStaticReference()
{
  if(!currentParameters.autoComTra)
  {
    Vector2f staticref = currentParameters.getComRefPosition(phase, phaseNumber);
    ref << staticref,
        (positions[Phase::leftFootTra].z() > positions[Phase::rightFootTra].z()) ? positions[Phase::leftFootTra].z() : positions[Phase::rightFootTra].z();
  }
}

void KickEngineData::mirrorIfNecessary(JointRequest& joints)
{
  if((positions[Phase::leftFootTra].z() - positions[Phase::rightFootTra].z()) > 5)
  {
    toLeftSupport = false;
  }
  else if((positions[Phase::leftFootTra].z() - positions[Phase::rightFootTra].z()) < -5)
  {
    toLeftSupport = true;
  }
  else
  {
    if(ref.y() > 0)
    {
      toLeftSupport = true;
    }
    else
    {
      toLeftSupport = false;
    }
  }

  if(currentKickRequest.mirror)
  {
    JointRequest old = joints;
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      if(i == Joints::headPitch)
        continue;

      joints.angles[i] = old.mirror(static_cast<Joints::Joint>(i));
    }
  }
}

void KickEngineData::addGyroBalance(JointRequest& jointRequest, const JointCalibration& jc, const InertialData& id, const float& ratio)
{
  if(id.gyro.y() != 0 && id.gyro.x() != 0 && !willBeLeft)
  {
    //Predict next gyrodata
    gyro.y() = id.gyro.y() * 0.3f + 0.7f * gyro.y();
    gyro.x() = id.gyro.x() * 0.3f + 0.7f * gyro.x();

    //some clipping
    for(int i = Joints::lHipYawPitch; i < Joints::numOfJoints; i++)
    {
      if(jointRequest.angles[i] > jc.joints[i].maxAngle) jointRequest.angles[i] = jc.joints[i].maxAngle;
      if(jointRequest.angles[i] < jc.joints[i].minAngle) jointRequest.angles[i] = jc.joints[i].minAngle;
    }

    JointRequest balancedJointRequest = jointRequest;

    //calculate the commandedVelocity
    float commandedVelocity[4];
    //y-velocity if left leg is support
    commandedVelocity[0] = (balancedJointRequest.angles[Joints::lHipPitch] - lastBalancedJointRequest.angles[Joints::lHipPitch]) / cycletime;
    //y-velocity if right leg is support
    commandedVelocity[1] = (balancedJointRequest.angles[Joints::rHipPitch] - lastBalancedJointRequest.angles[Joints::rHipPitch]) / cycletime;
    //x-velcocity if left leg is support
    commandedVelocity[2] = (balancedJointRequest.angles[Joints::lHipRoll] - lastBalancedJointRequest.angles[Joints::lHipRoll]) / cycletime;
    //x-velocity if right leg is support
    commandedVelocity[3] = (balancedJointRequest.angles[Joints::rHipRoll] - lastBalancedJointRequest.angles[Joints::rHipRoll]) / cycletime;

    //calculate disturbance from meseaured velocity and commanded velocity
    // y-velocity if left leg is support
    float gyroVelyLeft = (gyro.y() + commandedVelocity[0] - lastGyroLeft.y()) / cycletime;
    lastGyroLeft.y() = gyro.y() + commandedVelocity[0];
    //y-velocity if right leg is support
    float gyroVelyRight = (gyro.y() + commandedVelocity[1] - lastGyroRight.y()) / cycletime;
    lastGyroRight.y() = gyro.y() + commandedVelocity[1];
    //x-velocity if left leg is support
    float gyroVelxLeft = (gyro.x() + commandedVelocity[2] - lastGyroLeft.x()) / cycletime;
    lastGyroLeft.x() = gyro.x() + commandedVelocity[2];
    //x-velocity if right leg is support
    float gyroVelxRight = (gyro.x() + commandedVelocity[3] - lastGyroRight.x()) / cycletime;
    lastGyroRight.x() = gyro.x() + commandedVelocity[3];

    //calculate control variable with PID-Control
    float calcVelocity[4];
    //y if left supprt
    calcVelocity[0] = -gyroP.y() * (gyro.y() + commandedVelocity[0]) - gyroD.y() * gyroVelyLeft - gyroI.y() * (gyroErrorLeft.y());
    //y if right support
    calcVelocity[1] = -gyroP.y() * (gyro.y() + commandedVelocity[1]) - gyroD.y() * gyroVelyRight - gyroI.y() * (gyroErrorRight.y());
    //x if left support
    calcVelocity[2] = -gyroP.x() * (gyro.x() + commandedVelocity[2]) + gyroD.x() * gyroVelxLeft + gyroI.x() * gyroErrorLeft.x();
    //x if right support
    calcVelocity[3] = -gyroP.x() * (gyro.x() - commandedVelocity[3]) + gyroD.x() * gyroVelxRight + gyroI.x() * gyroErrorRight.x();

    bool supp = (currentKickRequest.mirror) ? !toLeftSupport : toLeftSupport;

    if(supp)  //last support Leg was left
    {
      //y
      jointRequest.angles[Joints::rHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[Joints::lHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[Joints::lAnklePitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[Joints::rAnklePitch] += calcVelocity[0] * cycletime * ratio;
      //x
      jointRequest.angles[Joints::lHipRoll] += calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[Joints::rHipRoll] += calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[Joints::lAnkleRoll] -= calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[Joints::rAnkleRoll] -= calcVelocity[2] * cycletime * ratio;
    }
    else //if(toRightSupport)
    {
      //y
      jointRequest.angles[Joints::rHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.angles[Joints::lHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.angles[Joints::lAnklePitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.angles[Joints::rAnklePitch] += calcVelocity[1] * cycletime * ratio;

      //x
      jointRequest.angles[Joints::lHipRoll] += calcVelocity[3] * cycletime * ratio;
      jointRequest.angles[Joints::rHipRoll] += calcVelocity[3] * cycletime * ratio;
      jointRequest.angles[Joints::lAnkleRoll] -= calcVelocity[3] * cycletime * ratio;
      jointRequest.angles[Joints::rAnkleRoll] -= calcVelocity[3] * cycletime * ratio;
    }
    gyroErrorLeft += lastGyroLeft;
    gyroErrorRight += lastGyroRight;
    lastBalancedJointRequest = balancedJointRequest;
  }
}

void KickEngineData::addDynPoint(const DynPoint& dynPoint, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix)
{
  Vector3f cubePoint, diff, d(dynPoint.translation);

  transferDynPoint(d, rd, torsoMatrix);

  int phaseNumber = dynPoint.phaseNumber;
  int limb = dynPoint.limb;

  if(dynPoint.duration > 0) currentParameters.phaseParameters[phaseNumber].duration = dynPoint.duration;

  currentParameters.phaseParameters[phaseNumber].odometryOffset = dynPoint.odometryOffset;

  cubePoint = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];

  diff = cubePoint - d;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -= diff;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] -= diff;

  Vector3f point1 = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
                    currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

  Vector3f absAngle;

  absAngle.x() = atan2(point1.y(), point1.z());
  absAngle.y() = atan2(point1.x(), point1.z());
  absAngle.z() = atan2(point1.x(), point1.y());

  RotationMatrix rot = RotationMatrix::aroundX(dynPoint.angle.x() - absAngle.x()).
                       rotateY(dynPoint.angle.y() - absAngle.y()).
                       rotateZ(dynPoint.angle.z() - absAngle.z());

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] =
    (rot * point1) + currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

  if(phaseNumber < currentParameters.numberOfPhases - 1)
  {
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

    float factor = static_cast<float>(currentParameters.phaseParameters[phaseNumber + 1].duration) /
                   static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration);
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  }
}

void KickEngineData::transferDynPoint(Vector3f& d, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix)
{
  Pose3f left(positions[Phase::leftFootTra]);
  Pose3f right(positions[Phase::rightFootTra]);

  const bool useLeft = left.translation.z() < right.translation.z();
  const Vector3f foot(useLeft ? left.translation : right.translation);

  Pose3f left2(torsoMatrix.rotation);
  left2.conc(robotModel.limbs[Limbs::footLeft])
    .translate(0.f, 0.f, -rd.footHeight);
  Pose3f right2(torsoMatrix.rotation);
  right2.conc(robotModel.limbs[Limbs::footRight])
    .translate(0.f, 0.f, -rd.footHeight);

  if(currentKickRequest.mirror)
  {
    Pose3f temp = left2;
    left2 = right2;
    right2 = temp;
    left2.translation.y() *= -1;
    right2.translation.y() *= -1;
  }

  const Vector3f foot2(useLeft ? left2.translation : right2.translation);

  Vector3f offset = foot - foot2;

  d += offset;
}

void KickEngineData::ModifyData(const KickRequest& br, JointRequest& kickEngineOutput, std::vector<KickEngineParameters>& params)
{
  auto& p = params.back();
  MODIFY("module:KickEngine:newKickMotion", p);
  strcpy(p.name, "newKick");

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
  PLOT("module:KickEngine:comy", robotModel.centerOfMass.y());
  PLOT("module:KickEngine:diffy", actualDiff.y());
  PLOT("module:KickEngine:refy", ref.y());

  PLOT("module:KickEngine:comx", robotModel.centerOfMass.x());
  PLOT("module:KickEngine:diffx", actualDiff.x());
  PLOT("module:KickEngine:refx", ref.x());

  PLOT("module:KickEngine:lastdiffy", toDegrees(lastBody.y()));
  PLOT("module:KickEngine:bodyErrory", toDegrees(bodyError.y()));

  for(int i = 0; i < Phase::numOfLimbs; i++)
  {
    int stiffness = 100;

    if(limbOff[i])
    {
      stiffness = 0;
    }
    switch((Phase::Limb)i)
    {
      case Phase::leftFootTra:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lHipRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lHipPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lKneePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lAnklePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lAnkleRoll] = stiffness;
        break;
      case Phase::rightFootTra:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rHipRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rHipPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rKneePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rAnklePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rAnkleRoll] = stiffness;
        break;
      case Phase::leftFootRot:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lAnklePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lAnkleRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rHipYawPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lHipYawPitch] = stiffness;
        break;
      case Phase::rightFootRot:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rAnklePitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rAnkleRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rHipYawPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lHipYawPitch] = stiffness;
        break;
      case Phase::leftArmTra:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lShoulderPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lShoulderRoll] = stiffness;
        break;
      case Phase::rightArmTra:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rShoulderPitch] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rShoulderRoll] = stiffness;
        break;
      case Phase::leftHandRot:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lElbowRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lElbowYaw] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lWristYaw] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::lHand] = stiffness;
        break;
      case Phase::rightHandRot:
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rElbowRoll] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rElbowYaw] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rWristYaw] = stiffness;
        kickEngineOutput.stiffnessData.stiffnesses[Joints::rHand] = stiffness;
        break;
    }
  }
}

void KickEngineData::setCycleTime(float time)
{
  cycletime = time;
}

void KickEngineData::calcPhaseState()
{
  phase = static_cast<float>(timeSinceTimeStamp) / static_cast<float>(currentParameters.phaseParameters[phaseNumber].duration);
}

void KickEngineData::calcPositions()
{
  for(int i = 0; i < Phase::numOfLimbs; ++i)
    positions[i] = currentParameters.getPosition(phase, phaseNumber, i);

  if(!currentParameters.ignoreHead)
    head = currentParameters.getHeadRefPosition(phase, phaseNumber);

  //these need to be similar
  if(positions[Phase::leftFootRot].z() != -positions[Phase::rightFootRot].z())
  {
    //just take the nearer postion to 0
    if(abs(positions[Phase::leftFootRot].z()) < abs(positions[Phase::rightFootRot].z()))
    {
      positions[Phase::rightFootRot] = -positions[Phase::leftFootRot];
    }
    else
    {
      positions[Phase::leftFootRot] = -positions[Phase::rightFootRot];
    }
  }
}

void KickEngineData::setRobotModel(const RobotModel& rm)
{
  robotModel = rm;
}

void KickEngineData::setCurrentKickRequest(const MotionRequest& mr)
{
  currentKickRequest = mr.kickRequest;
}

bool KickEngineData::isMotionAlmostOver()
{
  return false;
}

void KickEngineData::setExecutedKickRequest(KickRequest& br)
{
  br.mirror = currentKickRequest.mirror;
  br.dynamical = currentKickRequest.dynamical;
  br.kickMotionType = currentKickRequest.kickMotionType;
}

void KickEngineData::initData(const FrameInfo& frame, const MotionRequest& mr, const RobotDimensions& theRobotDimensions,
                              std::vector<KickEngineParameters>& params, const JointAngles& ja, const TorsoMatrix& torsoMatrix)
{
  if(getMotionIDByName(mr, params))
  {
    phase = 0.f;
    phaseNumber = 0;
    timeStamp = frame.time;

    ref = Vector3f::Zero();
    actualDiff = ref;
    currentParameters = params[motionID];
    calculateOrigins(theRobotDimensions, mr.kickRequest);
    currentParameters.initFirstPhase(origins, Vector2f(ja.angles[Joints::headPitch], (mr.kickRequest.mirror) ? -ja.angles[Joints::headYaw] : ja.angles[Joints::headYaw]));

    if(!wasActive)
    {
      comRobotModel = robotModel;
      lSupp = false;
      rSupp = false;
      toLeftSupport = false;
      comOffset = Vector2f::Zero();
      origin = Vector2f::Zero();
      balanceSum = Vector2f::Zero();
      gyro = Vector2f::Zero();
      lastGyroLeft = Vector2f::Zero();
      lastGyroRight = Vector2f::Zero();
      gyroErrorLeft = Vector2f::Zero();
      gyroErrorRight = Vector2f::Zero();
      bodyError = Vector2f::Zero();
      lastBody = Vector2f::Zero();
      lastCom = Vector3f::Zero();

      for(int i = 0; i < Joints::numOfJoints; i++)
      {
        lastBalancedJointRequest.angles[i] = ja.angles[i];
      }
    }
    if(currentKickRequest.dynamical && !currentKickRequest.dynPoints.empty())
      for(unsigned int i = 0; i < currentKickRequest.dynPoints.size(); i++)
        if(currentKickRequest.dynPoints[i].phaseNumber == phaseNumber)
          addDynPoint(currentKickRequest.dynPoints[i], theRobotDimensions, torsoMatrix);
  }
}

void KickEngineData::setEngineActivation(const float& ratio)
{
  willBeLeft = (ratio < 1.f && lastRatio > ratio);
  wasActive = (ratio != 0.f && motionID > -1);
  startComp = (ratio != 0.f && lastRatio <= ratio);
  lastRatio = ratio;
}

bool KickEngineData::activateNewMotion(const KickRequest& br, const bool& isLeavingPossible)
{
  if(!wasActive || (br.kickMotionType != currentKickRequest.kickMotionType && isLeavingPossible))
    return true;
  else if(br.kickMotionType == currentKickRequest.kickMotionType && br.mirror == currentKickRequest.mirror)
    currentKickRequest = br; // update KickRequest when it is compatible to the current motion

  return false;
}

bool KickEngineData::sitOutTransitionDisturbance(bool& compensate, bool& compensated, const InertialData& id, KickEngineOutput& kickEngineOutput, const JointAngles& ja, const FrameInfo& frame)
{
  if(compensate)
  {
    if(!startComp)
    {
      timeStamp = frame.time;
      lSupp = false;
      rSupp = false;
      toLeftSupport = false;
      comOffset = Vector2f::Zero();
      origin = Vector2f::Zero();
      balanceSum = Vector2f::Zero();
      gyro = Vector2f::Zero();
      lastGyroLeft = Vector2f::Zero();
      lastGyroRight = Vector2f::Zero();
      gyroErrorLeft = Vector2f::Zero();
      gyroErrorRight = Vector2f::Zero();
      bodyError = Vector2f::Zero();
      lastBody = Vector2f::Zero();
      lastCom = Vector3f::Zero();
      motionID = -1;

      kickEngineOutput.isLeavingPossible = false;

      for(int i = 0; i < Joints::numOfJoints; i++)
      {
        lastBalancedJointRequest.angles[i] = ja.angles[i];
        compenJoints.angles[i] = ja.angles[i];
      }
    }

    for(int i = 0; i < Joints::numOfJoints; i++)
    {
      kickEngineOutput.angles[i] = compenJoints.angles[i];
      kickEngineOutput.stiffnessData.stiffnesses[i] = 100;
    }

    int time = frame.getTimeSince(timeStamp);
    if((abs(id.gyro.x()) < 0.1f && abs(id.gyro.x()) < 0.1f && time > 200) || time > 1000)
    {
      compensate = false;
      compensated = true;
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}
