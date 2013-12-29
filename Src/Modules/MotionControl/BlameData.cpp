/*
* @file BLAAME.cpp
* This file implements a module that creates motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#include <cstring>

#include "BlameData.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/InverseKinematic.h"
#include "Tools/Debugging/DebugDrawings.h"

using namespace std;

bool BlameData::getMotionIDByName(const MotionRequest& motionRequest, const std::vector<BIKEParameters>& params)
{
  motionID = -1;

  for(unsigned int i = 0; i < params.size(); ++i)
    if(motionRequest.bikeRequest.getBMotionFromName(&params[i].name[0]) == motionRequest.bikeRequest.bMotionType)
    {
      motionID = i;
      return true;
    }

  return false;
}


void BlameData::calculateOrigins(const RobotDimensions& theRobotDimensions, const FilteredJointData& jd)
{
  if(!wasActive)
  {
    origins[Phase::leftFootTra] = robotModel.limbs[MassCalibration::footLeft].translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint).translation;
    origins[Phase::rightFootTra] = robotModel.limbs[MassCalibration::footRight].translate(0.f, 0.f, -theRobotDimensions.heightLeg5Joint).translation;
    origins[Phase::leftArmTra] = robotModel.limbs[MassCalibration::foreArmLeft].translation;
    origins[Phase::rightArmTra] = robotModel.limbs[MassCalibration::foreArmRight].translation;

    origins[Phase::leftFootRot] =  robotModel.limbs[MassCalibration::footLeft].rotation.getAngleAxis();
    origins[Phase::rightFootRot] =  robotModel.limbs[MassCalibration::footRight].rotation.getAngleAxis();
    origins[Phase::leftHandRot] = robotModel.limbs[MassCalibration::foreArmLeft].rotation.getAngleAxis();
    origins[Phase::rightHandRot] = robotModel.limbs[MassCalibration::foreArmRight].rotation.getAngleAxis();
  }
  else
  {
    for(int i = 0; i < Phase::numOfLimbs; ++i)
    {
      origins[i] = currentParameters.phaseParameters[currentParameters.numberOfPhases - 1].controlPoints[i][2];
    }
  }
}


bool BlameData::checkPhaseTime(const FrameInfo& frame, const RobotDimensions& rd, const FilteredJointData& jd, const TorsoMatrix& torsoMatrix)
{
  timeSinceTimeStamp = frame.getTimeSince(timeStamp);

  if(motionID > -1)
  {
    if(phaseNumber < currentParameters.numberOfPhases)
    {
      if((unsigned int) timeSinceTimeStamp > currentParameters.phaseParameters[phaseNumber].duration)
      {
        phaseNumber++;
        timeStamp = frame.time;
        timeSinceTimeStamp = frame.getTimeSince(timeStamp);
        if(currentBikeRequest.dynamical && !currentBikeRequest.dynPoints.empty())
          for(unsigned int i = 0; i < currentBikeRequest.dynPoints.size(); i++)
            if(currentBikeRequest.dynPoints[i].phaseNumber == phaseNumber)
              addDynPoint(currentBikeRequest.dynPoints[i], rd, torsoMatrix);

      }
    }
    else if(currentParameters.loop && phaseNumber == currentParameters.numberOfPhases)
    {
      phaseNumber = 0;
      calculateOrigins(rd, jd);
      currentParameters.initFirstPhase(origins,  Vector2<>(jd.angles[JointData::HeadPitch], jd.angles[JointData::HeadYaw]));
      if(currentBikeRequest.dynamical && !currentBikeRequest.dynPoints.empty())
        for(unsigned int i = 0; i < currentBikeRequest.dynPoints.size(); i++)
          if(currentBikeRequest.dynPoints[i].phaseNumber == phaseNumber)
            addDynPoint(currentBikeRequest.dynPoints[i], rd, torsoMatrix);
    }

    return (phaseNumber < currentParameters.numberOfPhases);
  }
  else
    return false;
}


bool BlameData::calcJoints(JointRequest& jointRequest, const RobotDimensions& rd, const FilteredJointData& jd)
{
  //Calculate Legs
  if(motionID > -1)
  {
    calcLegJoints(JointData::LHipYawPitch, jointRequest, rd);
    calcLegJoints(JointData::RHipYawPitch, jointRequest, rd);

    jointRequest.angles[JointData::LHipRoll] -= comOffset.x;
    jointRequest.angles[JointData::RHipRoll] += comOffset.x;

    jointRequest.angles[JointData::RHipPitch] += comOffset.y;
    jointRequest.angles[JointData::LHipPitch] += comOffset.y;

    simpleCalcArmJoints(JointData::LShoulderPitch, jointRequest, rd, positions[Phase::leftArmTra], positions[Phase::leftHandRot]);
    simpleCalcArmJoints(JointData::RShoulderPitch, jointRequest, rd, positions[Phase::rightArmTra], positions[Phase::rightHandRot]);

    return true;
  }
  else //just set the measured angeles
  {

    for(int i = JointData::LShoulderPitch; i < JointData::numOfJoints; ++i)
      jointRequest.angles[i] = jd.angles[i];

    return false;
  }
}

void BlameData::calcLegJoints(const JointData::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions)
{
  float sign = joint == JointData::LHipYawPitch ? 1.f : -1.f;
  float leg0, leg1, leg2, leg3, leg4, leg5;

  const Vector3<>& footPos = (sign > 0) ? positions[Phase::leftFootTra] : positions[Phase::rightFootTra];
  const Vector3<>& footRotAng = (sign > 0) ? positions[Phase::leftFootRot] : positions[Phase::rightFootRot];

  RotationMatrix rotateBodyTilt = RotationMatrix().rotateX(comOffset.x);
  Vector3<> anklePos = rotateBodyTilt * Vector3<>(footPos.x, footPos.y, footPos.z);
  //we need just the leg length x and y have to stay the same
  anklePos.y = footPos.y;
  anklePos.x = footPos.x;
  // for the translation of the footpos we only need to translate the anklepoint, which is the intersection of the axis leg4 and leg5
  // the rotation of the foot will be made later by rotating the footpos around the anklepoint
  anklePos -= Vector3<>(0.f, sign * (theRobotDimensions.lengthBetweenLegs / 2), -theRobotDimensions.heightLeg5Joint);

  RotationMatrix rotateBecauseOfHip = RotationMatrix().rotateZ(footRotAng.z).rotateX(-sign * pi_4);

  anklePos = rotateBecauseOfHip * anklePos;

  float diagonal = anklePos.abs();

  // upperLegLength, lowerLegLength, and diagonal form a triangle, use cosine theorem
  float a1 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength -
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength + diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * diagonal);
  //if(abs(a1) > 1.f) OUTPUT(idText, text, "clipped a1");
  a1 = abs(a1) > 1.f ? 0.f : acos(a1);

  float a2 = (theRobotDimensions.upperLegLength * theRobotDimensions.upperLegLength +
              theRobotDimensions.lowerLegLength * theRobotDimensions.lowerLegLength - diagonal * diagonal) /
             (2 * theRobotDimensions.upperLegLength * theRobotDimensions.lowerLegLength);
  //if(abs(a2) > 1.f) OUTPUT(idText, text, "clipped a2");
  a2 = abs(a2) > 1.f ? pi : acos(a2);

  leg0 = footRotAng.z * sign;
  leg2 = -a1 - atan2(anklePos.x, Vector2<>(anklePos.y, anklePos.z).abs() * -sgn(anklePos.z));

  leg1 = anklePos.z == 0.0f ? 0.0f : atan(anklePos.y / -anklePos.z) * -sign;
  leg3 = pi - a2;

  RotationMatrix footRot = RotationMatrix().rotateX(leg1 * -sign).rotateY(leg2 + leg3);
  footRot = footRot.invert() * rotateBecauseOfHip;

  leg5 = asin(-footRot[2].y) * sign * -1;
  leg4 = -atan2(footRot[2].x, footRot[2].z) * -1;

  leg4 += footRotAng.y;
  leg5 += footRotAng.x;

  jointRequest.angles[joint] = leg0;
  jointRequest.angles[joint + 1] = -pi_4 + leg1;
  jointRequest.angles[joint + 2] = leg2;
  jointRequest.angles[joint + 3] = leg3;
  jointRequest.angles[joint + 4] = leg4;
  jointRequest.angles[joint + 5] = leg5;

}

void BlameData::simpleCalcArmJoints(const JointData::Joint& joint, JointRequest& jointRequest, const RobotDimensions& theRobotDimensions, const Vector3<>& armPos, const Vector3<>& handRotAng)
{
  float sign = joint == JointData::LShoulderPitch ? 1.f : -1.f;

  Vector3<> target =  armPos -  Vector3<>(theRobotDimensions.armOffset.x,
                                          theRobotDimensions.armOffset.y * sign,
                                          theRobotDimensions.armOffset.z);

  jointRequest.angles[joint + 0] = atan2(target.z, target.x);
  jointRequest.angles[joint + 1] = atan2(target.y * sign, sqrt(sqr(target.x) + sqr(target.z)));

  float length2ElbowJoint = Vector3<>(theRobotDimensions.upperArmLength, theRobotDimensions.yElbowShoulder, 0.f).abs();
  float angle = asin(theRobotDimensions.upperArmLength / length2ElbowJoint);

  Pose3D elbow;
  elbow.rotateY(-jointRequest.angles[joint + 0])
  .rotateZ(jointRequest.angles[joint + 1])
  .translate(length2ElbowJoint, 0.f , 0.f)
  .rotateZ(-angle)
  .translate(theRobotDimensions.yElbowShoulder, 0.f, 0.f);

  jointRequest.angles[joint + 0] = atan2(elbow.translation.z, elbow.translation.x);
  jointRequest.angles[joint + 1] = atan2(elbow.translation.y, sqrt(sqr(elbow.translation.x) + sqr(elbow.translation.z)));
  jointRequest.angles[joint + 0] = (jointRequest.angles[joint + 0] < pi) ? jointRequest.angles[joint + 0] : 0;  //clip special case


  Pose3D hand(elbow.translation);

  hand.rotateZ(handRotAng.z * sign)
  .rotateY(handRotAng.y)
  .rotateX(handRotAng.x * sign)
  .translate(theRobotDimensions.lowerArmLength, 0.f, 0.f);

  InverseKinematic::calcJointsForElbowPos(elbow.translation, hand.translation, jointRequest, joint, theRobotDimensions);

}

void BlameData::balanceCOM(JointRequest& joints, const RobotDimensions& rd, const MassCalibration& mc, const FilteredJointData& theFilteredJointData)
{

  if(currentParameters.autoComTra)
  {
    setStandLeg(origin.y);
    calculatePreviewCom(ref, origin);
  }

  if(currentParameters.ignoreHead)
  {
    joints.angles[JointData::HeadYaw] = theFilteredJointData.angles[JointData::HeadYaw];
    joints.angles[JointData::HeadPitch] = theFilteredJointData.angles[JointData::HeadPitch];
  }

  comRobotModel.setJointData(joints, rd, mc);
  Vector3<> com(comRobotModel.centerOfMass);

  Pose3D torso = (toLeftSupport) ? comRobotModel.limbs[MassCalibration::footLeft] : comRobotModel.limbs[MassCalibration::footRight];

  com = torso.rotation.invert() * com;

  actualDiff = com - ref;
  // if(!toLeftSupport)
  //	  actualDiff.y += 15;

  balanceSum += Vector2<> (actualDiff.x, actualDiff.y);

  float height = comRobotModel.centerOfMass.z - ref.z;

  Vector2<> balance;
  balance.y = -currentParameters.kpx * (actualDiff.y) + -currentParameters.kix * balanceSum.y + -currentParameters.kdx * ((actualDiff.y - lastCom.y) / cycletime);
  balance.x = currentParameters.kpy * (actualDiff.x)  + currentParameters.kiy * balanceSum.x + currentParameters.kdy * ((actualDiff.x - lastCom.x) / cycletime);

  if(height != 0.f)
  {
    comOffset.x = (balance.y != 0) ? atan2((balance.y), height) : 0;
    comOffset.y = (balance.x != 0) ? atan2((balance.x), height) : 0;
  }

  lastCom = actualDiff;

  if(currentParameters.ignoreHead)
  {
    joints.angles[JointData::HeadPitch] = JointData::ignore;
    joints.angles[JointData::HeadYaw] = JointData::ignore;
  }
}


void BlameData::calculatePreviewCom(Vector3<>& ref, Vector2<>& origin)
{
  ref = Vector3<>(0.f, 0.f, 0.f);
  float lastPhaseTime = 0.f;
  float future = 0.f;
  int futPhaNum = phaseNumber;
  float currPhaseTime = (float)timeSinceTimeStamp;

  for(int j = 1; j < currentParameters.preview + 1; j++)
  {
    if(futPhaNum < currentParameters.numberOfPhases)
    {
      future = (float)(currPhaseTime + j * cycletime * 1000 - lastPhaseTime) / (float) currentParameters.phaseParameters[futPhaNum].duration;

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

    Vector3<> futPos[2];
    futPos[0] = currentParameters.getPositionBlame(future, futPhaNum, Phase::leftFootTra);
    futPos[1] = currentParameters.getPositionBlame(future, futPhaNum, Phase::rightFootTra);


    Vector3<> comRef(0, 0, 0);
    Vector2<> fOrigin(0, 0);

    getCOMReference(futPos[0],  futPos[1], comRef, fOrigin);

    ref += comRef;
    origin += fOrigin;

  }
  ref /= (float)currentParameters.preview;
  origin /= (float)currentParameters.preview;
}

void BlameData::setStandLeg(const float& originY)
{
  //If the average standLegPos Reference has reached the current standleg position, lock that
  if(abs(positions[Phase::leftFootTra].y - (float)standLegPos.getAverage().y) < 1.f)
  {
    lSupp = true;
    rSupp = false;
  }
  if(abs(positions[Phase::rightFootTra].y - (float)standLegPos.getAverage().y) < 1.f)
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

void BlameData::getCOMReference(const Vector3<>& lFootPos, const Vector3<>& rFootPos, Vector3<>& comRef, Vector2<>& origin)
{
  origin = Vector2<>(rFootPos.x + 30.f, 0.f);
  comRef = Vector3<>(origin.x, 0.f, rFootPos.z); //double support

  if(abs(lFootPos.y + rFootPos.y) > 2.f)  //equal distance to the middle
  {
    if((lFootPos.y + rFootPos.y) < -2.f  && !lSupp && !rSupp) //calc reference
    {
      origin.y = origins[Phase::leftFootTra].y;
      comRef = lFootPos - Vector3<>(-30.f, 0.f, 0.f); //L-Support
      if(comRef.y > 0.f) comRef.y = 0.f;
      standLegPos.add(Vector2<int>((int)comRef.x, (int)comRef.y));
    }

    if((lFootPos.y + rFootPos.y) > 2.f  && !lSupp && !rSupp)
    {
      origin.y = origins[Phase::rightFootTra].y;
      comRef = rFootPos - Vector3<>(-30.f, 0.f, 0.f); //R-Support
      if(comRef.y < 0.f) comRef.y = 0.f;
      standLegPos.add(Vector2<int>((int)comRef.x, (int)comRef.y));
    }

    if(lSupp)
    {
      origin.y = origins[Phase::leftFootTra].y;
      comRef = Vector3<>((float)standLegPos.getAverage().x, (float)standLegPos.getAverage().y + 10, lFootPos.z);
    }
    if(rSupp)
    {
      origin.y = origins[Phase::rightFootTra].y;
      comRef = Vector3<>((float)standLegPos.getAverage().x, (float)standLegPos.getAverage().y - 10, rFootPos.z);
    }
  }
}

void BlameData::setStaticReference()
{
  if(!currentParameters.autoComTra)
  {
    Vector2<> staticref =  currentParameters.getComRefPositionBlame(phase, phaseNumber);
    ref = Vector3<>(staticref.x, staticref.y, 0.f);
    ref.z = (positions[Phase::leftFootTra].z > positions[Phase::rightFootTra].z) ? positions[Phase::leftFootTra].z : positions[Phase::rightFootTra].z;
  }
}

void BlameData::mirrorIfNecessary(JointRequest& joints)
{
  if((positions[Phase::leftFootTra].z - positions[Phase::rightFootTra].z) > 5)
  {
    toLeftSupport = false;
  }
  else if((positions[Phase::leftFootTra].z - positions[Phase::rightFootTra].z) < -5)
  {
    toLeftSupport = true;
  }
  else
  {
    if(ref.y > 0)
    {
      toLeftSupport = true;
    }
    else
    {
      toLeftSupport = false;
    }
  }

  if(currentBikeRequest.mirror)
  {
    JointRequest old = joints;
    for(int i = 0; i < JointData::numOfJoints; ++i)
    {
      joints.angles[i] = old.mirror(JointData::Joint(i));
    }
  }
}


void BlameData::addGyroBalance(JointRequest& jointRequest, const JointCalibration& jc, const FilteredSensorData& sd, const float& ratio)
{
  if(sd.data[SensorData::gyroY] != 0 &&  sd.data[SensorData::gyroX] != 0 && !willBeLeft)
  {
    //Predict next gyrodata
    gyro.y = sd.data[SensorData::gyroY] * 0.3f + 0.7f * gyro.y;
    gyro.x = sd.data[SensorData::gyroX] * 0.3f + 0.7f * gyro.x;

    //some clipping
    for(int i = JointData::LHipYawPitch; i < JointData::numOfJoints; i++)
    {
      if(jointRequest.angles[i] > jc.joints[i].maxAngle) jointRequest.angles[i] = jc.joints[i].maxAngle;
      if(jointRequest.angles[i] < jc.joints[i].minAngle) jointRequest.angles[i] = jc.joints[i].minAngle;
    }

    JointRequest balancedJointRequest = jointRequest;

    //calculate the commandedVelocity
    float commandedVelocity[4];
    //y-velocity if left leg is support
    commandedVelocity[0] = (balancedJointRequest.angles[JointData::LHipPitch] - lastBalancedJointRequest.angles[JointData::LHipPitch]) / cycletime;
    //y-velocity if right leg is support
    commandedVelocity[1] = (balancedJointRequest.angles[JointData::RHipPitch] - lastBalancedJointRequest.angles[JointData::RHipPitch]) / cycletime;
    //x-velcocity if left leg is support
    commandedVelocity[2] = (balancedJointRequest.angles[JointData::LHipRoll] - lastBalancedJointRequest.angles[JointData::LHipRoll]) / cycletime;
    //x-velocity if right leg is support
    commandedVelocity[3] = (balancedJointRequest.angles[JointData::RHipRoll] - lastBalancedJointRequest.angles[JointData::RHipRoll]) / cycletime;

    //calculate disturbance from meseaured velocity and commanded velocity
    // y-velocity if left leg is support
    float gyroVelyLeft = (gyro.y + commandedVelocity[0] - lastGyroLeft.y) / cycletime;
    lastGyroLeft.y = gyro.y + commandedVelocity[0];
    //y-velocity if right leg is support
    float gyroVelyRight = (gyro.y + commandedVelocity[1] - lastGyroRight.y) / cycletime;
    lastGyroRight.y = gyro.y + commandedVelocity[1];
    //x-velocity if left leg is support
    float gyroVelxLeft = (gyro.x + commandedVelocity[2] - lastGyroLeft.x) / cycletime;
    lastGyroLeft.x = gyro.x + commandedVelocity[2];
    //x-velocity if right leg is support
    float gyroVelxRight = (gyro.x - commandedVelocity[3] - lastGyroRight.x) / cycletime;
    lastGyroRight.x = gyro.x - commandedVelocity[3];


    //calculate control variable with PID-Control
    float calcVelocity[4];
    //y if left supprt
    calcVelocity[0] = -gyroP.y * (gyro.y + commandedVelocity[0]) - gyroD.y * gyroVelyLeft - gyroI.y * (gyroErrorLeft.y);
    //y if right support
    calcVelocity[1] = -gyroP.y * (gyro.y + commandedVelocity[1]) - gyroD.y * gyroVelyRight - gyroI.y * (gyroErrorRight.y);
    //x if left support
    calcVelocity[2] = gyroP.x * (gyro.x + commandedVelocity[2]) - gyroD.x * gyroVelxLeft - gyroI.x * gyroErrorLeft.x;
    //x if right support
    calcVelocity[3] = -gyroP.x * (gyro.x - commandedVelocity[3]) + gyroD.x * gyroVelxRight + gyroI.x * gyroErrorRight.x;

    bool supp = (currentBikeRequest.mirror) ? !toLeftSupport : toLeftSupport;

    if(supp)  //last support Leg was left
    {
      //y
      jointRequest.angles[JointData::RHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[JointData::LHipPitch] += calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[JointData::LAnklePitch] +=  calcVelocity[0] * cycletime * ratio;
      jointRequest.angles[JointData::RAnklePitch] += calcVelocity[0] * cycletime * ratio;
      //x
      jointRequest.angles[JointData::LHipRoll] += -calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[JointData::RHipRoll] += calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[JointData::LAnkleRoll] -= calcVelocity[2] * cycletime * ratio;
      jointRequest.angles[JointData::RAnkleRoll] -= -calcVelocity[2] * cycletime * ratio;
    }
    else //if (toRightSupport)
    {
      //y
      jointRequest.angles[JointData::RHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.angles[JointData::LHipPitch] += calcVelocity[1] * cycletime * ratio;
      jointRequest.angles[JointData::LAnklePitch] +=  calcVelocity[1] * cycletime * ratio;

      jointRequest.angles[JointData::RAnklePitch] += calcVelocity[1] * cycletime * ratio;

      //x
      jointRequest.angles[JointData::LHipRoll] += calcVelocity[3] * cycletime * ratio;
      jointRequest.angles[JointData::RHipRoll] += -calcVelocity[3] * cycletime * ratio;

      jointRequest.angles[JointData::LAnkleRoll] -= -calcVelocity[3] * cycletime * ratio;
      jointRequest.angles[JointData::RAnkleRoll] -= calcVelocity[3] * cycletime * ratio;
    }
    gyroErrorLeft += lastGyroLeft;
    gyroErrorRight += lastGyroRight;
    lastBalancedJointRequest = balancedJointRequest;
  }
}

void BlameData::addDynPoint(const DynPoint& dynPoint, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix)
{
  Vector3 <> cubePoint, diff, d(dynPoint.translation);

  transferDynPoint(d, rd, torsoMatrix);

  int phaseNumber = dynPoint.phaseNumber;
  int limb =  dynPoint.limb;

  if(dynPoint.duration > 0) currentParameters.phaseParameters[phaseNumber].duration = dynPoint.duration;

  currentParameters.phaseParameters[phaseNumber].odometryOffset = dynPoint.odometryOffset;

  cubePoint = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];

  diff = cubePoint - d;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -= diff;

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] -= diff;

  Vector3<> point1 = currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
                     currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

  Vector3<> absAngle;

  absAngle.x = atan2(point1.y, point1.z);
  absAngle.y = atan2(point1.x, point1.z);
  absAngle.z = atan2(point1.x, point1.y);

  RotationMatrix rot = RotationMatrix().rotateX(dynPoint.angle.x - absAngle.x).
                       rotateY(dynPoint.angle.y - absAngle.y).
                       rotateZ(dynPoint.angle.z - absAngle.z);

  currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1] =
    (rot * point1) + currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

  if(phaseNumber < currentParameters.numberOfPhases - 1)
  {
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] =
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2] -
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][1];

    float factor = (float)currentParameters.phaseParameters[phaseNumber + 1].duration /
                   (float)currentParameters.phaseParameters[phaseNumber].duration;
    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] *= factor;

    currentParameters.phaseParameters[phaseNumber + 1].controlPoints[limb][0] +=
      currentParameters.phaseParameters[phaseNumber].controlPoints[limb][2];
  }
}

void BlameData::transferDynPoint(Vector3<>& d, const RobotDimensions& rd, const TorsoMatrix& torsoMatrix)
{
  Pose3D left(positions[Phase::leftFootTra]);
  Pose3D right(positions[Phase::rightFootTra]);

  const bool useLeft = left.translation.z < right.translation.z;
  const Vector3<> foot(useLeft ? left.translation : right.translation);

  Pose3D left2(torsoMatrix.rotation);
  left2.conc(robotModel.limbs[MassCalibration::footLeft])
  .translate(0.f, 0.f, -rd.heightLeg5Joint);
  Pose3D right2(torsoMatrix.rotation);
  right2.conc(robotModel.limbs[MassCalibration::footRight])
  .translate(0.f, 0.f, -rd.heightLeg5Joint);

  if(currentBikeRequest.mirror)
  {
    Pose3D temp = left2;
    left2 = right2;
    right2 = temp;
    left2.translation.y *= -1;
    right2.translation.y *= -1;
  }


  const Vector3<> foot2(useLeft ? left2.translation : right2.translation);

  Vector3<> offset = foot - foot2;

  d += offset;
}




void BlameData::ModifyData(const BikeRequest& br, JointRequest& bikeOutput, std::vector<BIKEParameters>& params)
{

#ifndef RELEASE
  int p = params.size() - 1;
  MODIFY("module:BIKE:newBMotion", params[p]);
  strcpy(params[p].name, "newKick");
#endif //RELEASE

  MODIFY("module:BIKE:px", gyroP.x);
  MODIFY("module:BIKE:dx", gyroD.x);
  MODIFY("module:BIKE:ix", gyroI.x);
  MODIFY("module:BIKE:py", gyroP.y);
  MODIFY("module:BIKE:dy", gyroD.y);
  MODIFY("module:BIKE:iy", gyroI.y);

  MODIFY("module:BIKE:formMode", formMode);
  MODIFY("module:BIKE:lFootTraOff", limbOff[Phase::leftFootTra]);
  MODIFY("module:BIKE:rFootTraOff", limbOff[Phase::rightFootTra]);
  MODIFY("module:BIKE:lFootRotOff", limbOff[Phase::leftFootRot]);
  MODIFY("module:BIKE:rFootRotOff", limbOff[Phase::rightFootRot]);
  MODIFY("module:BIKE:lHandTraOff", limbOff[Phase::leftArmTra]);
  MODIFY("module:BIKE:rHandTraOff", limbOff[Phase::rightArmTra]);
  MODIFY("module:BIKE:lHandRotOff", limbOff[Phase::leftHandRot]);
  MODIFY("module:BIKE:rHandRotOff", limbOff[Phase::rightHandRot]);

  //Plot com stabilizing
  PLOT("module:Bike:comy", robotModel.centerOfMass.y);
  PLOT("module:Bike:diffy", actualDiff.y);
  PLOT("module:Bike:refy", ref.y);

  PLOT("module:Bike:comx", robotModel.centerOfMass.x);
  PLOT("module:Bike:diffx", actualDiff.x);
  PLOT("module:Bike:refx", ref.x);

  PLOT("module:Bike:lastdiffy", toDegrees(lastBody.y));
  PLOT("module:Bike:bodyErrory", toDegrees(bodyError.y));

#ifndef RELEASE
  for(int i = 0; i < Phase::numOfLimbs; i++)
  {
    int hardness = 100;

    if(limbOff[i])
    {
      hardness = 0;
    }
    switch((Phase::Limb)i)
    {
    case Phase::leftFootTra:
      bikeOutput.jointHardness.hardness[JointData::LHipRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LHipPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LKneePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LAnklePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LAnkleRoll] = hardness;
      break;
    case Phase::rightFootTra:
      bikeOutput.jointHardness.hardness[JointData::RHipRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RHipPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RKneePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RAnklePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RAnkleRoll] = hardness;
      break;
    case Phase::leftFootRot:
      bikeOutput.jointHardness.hardness[JointData::LAnklePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LAnkleRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RHipYawPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LHipYawPitch] = hardness;
      break;
    case Phase::rightFootRot:
      bikeOutput.jointHardness.hardness[JointData::RAnklePitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RAnkleRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RHipYawPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LHipYawPitch] = hardness;
      break;
    case Phase::leftArmTra:
      bikeOutput.jointHardness.hardness[JointData::LShoulderPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LShoulderRoll] = hardness;
      break;
    case Phase::rightArmTra:
      bikeOutput.jointHardness.hardness[JointData::RShoulderPitch] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RShoulderRoll] = hardness;
      break;
    case Phase::leftHandRot:
      bikeOutput.jointHardness.hardness[JointData::LElbowRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::LElbowYaw] = hardness;
      break;
    case Phase::rightHandRot:
      bikeOutput.jointHardness.hardness[JointData::RElbowRoll] = hardness;
      bikeOutput.jointHardness.hardness[JointData::RElbowYaw] = hardness;
      break;
    }
  }
#endif

}

void BlameData::debugFormMode(std::vector<BIKEParameters>& params)
{
#ifndef RELEASE
  //this is for making motions with the tool
  /*  if(motionID != -1 && formMode)
    {
      currentParameters = params[motionID];
      currentParameters.initFirstPhase(origins);

      if(phaseNumber >= currentParameters.numberOfPhases)
      {
        for(int i = 0; i < Phase::numOfLimbs; ++i)
        {
          positions[i] = currentParameters.getPositionBlame(1.f, currentParameters.numberOfPhases - 1, i);
        }
      }
    }*/
#endif
}




void BlameData::setCycleTime(float time)
{
  cycletime = time;
};

void BlameData::calcPhaseState()
{
  phase = (float) timeSinceTimeStamp / (float) currentParameters.phaseParameters[phaseNumber].duration;
};

void BlameData::calcPositions(JointRequest& joints, const FilteredJointData& theFilteredJointData)
{
  for(int i = 0; i < Phase::numOfLimbs; ++i)
    positions[i] = currentParameters.getPositionBlame(phase, phaseNumber, i);

  if(!currentParameters.ignoreHead)
  {
    Vector2<> head = currentParameters.getHeadRefPositionBlame(phase, phaseNumber);
    joints.angles[JointData::HeadPitch] = head.x;
    joints.angles[JointData::HeadYaw] = head.y;
  }
  //these need to be similar
  if(positions[Phase::leftFootRot].z != -positions[Phase::rightFootRot].z)
  {
    //just take the nearer postion to 0
    if(abs(positions[Phase::leftFootRot].z) < abs(positions[Phase::rightFootRot].z))
    {
      positions[Phase::rightFootRot] = -positions[Phase::leftFootRot];
    }
    else
    {
      positions[Phase::leftFootRot] = -positions[Phase::rightFootRot];
    }
  }
};

void BlameData::setRobotModel(RobotModel rm)
{
  robotModel = rm;
};

void BlameData::setCurrentBikeRequest(const MotionRequest& mr)
{
  currentBikeRequest = mr.bikeRequest;
}

void BlameData::setExecutedBikeRequest(BikeRequest& br)
{
  br.mirror = currentBikeRequest.mirror;
  br.dynamical = currentBikeRequest.dynamical;
  br.bMotionType = currentBikeRequest.bMotionType;
}

void BlameData::initData(const bool& compensated, const FrameInfo& frame, const MotionRequest& mr, const RobotDimensions& theRobotDimensions, std::vector<BIKEParameters>& params, const FilteredJointData& jd, const TorsoMatrix& torsoMatrix)
{
  if(getMotionIDByName(mr, params))
  {
    phase = 0.f;
    phaseNumber = 0;
    timeStamp = frame.time;
    ref = Vector3<>(0.f, 0.f, 0.f);
    actualDiff = ref;
    currentParameters = params[motionID];
    calculateOrigins(theRobotDimensions, jd);
    currentParameters.initFirstPhase(origins, Vector2<>(jd.angles[JointData::HeadPitch], (mr.bikeRequest.mirror) ? -jd.angles[JointData::HeadYaw] : jd.angles[JointData::HeadYaw]));

    if(!wasActive)
    {
      comRobotModel = robotModel;
      lSupp = false;
      rSupp = false;
      toLeftSupport = false;
      comOffset = Vector2<>(0.f, 0.f);
      origin = Vector2<>(0.f, 0.f);
      balanceSum = Vector2<>(0.f, 0.f);
      gyro = Vector2<>(0.f, 0.f);
      lastGyroLeft = Vector2<>(0.f, 0.f);
      lastGyroRight = Vector2<>(0.f, 0.f);
      gyroErrorLeft = Vector2<>(0.f, 0.f);
      gyroErrorRight = Vector2<>(0.f, 0.f);
      bodyError = Vector2<>(0.f, 0.f);
      lastBody = Vector2<>(0.f, 0.f);
      lastCom = Vector3<>(0.f, 0.f, 0.f);

      for(int i = 0; i < JointData::numOfJoints; i++)
      {
        lastBalancedJointRequest.angles[i] = jd.angles[i];
      }
    }
    if(currentBikeRequest.dynamical && !currentBikeRequest.dynPoints.empty())
      for(unsigned int i = 0; i < currentBikeRequest.dynPoints.size(); i++)
        if(currentBikeRequest.dynPoints[i].phaseNumber == phaseNumber)
          addDynPoint(currentBikeRequest.dynPoints[i], theRobotDimensions, torsoMatrix);
  }
};

void BlameData::setEngineActivation(const float& ratio)
{
  willBeLeft = (ratio < 1.f && lastRatio > ratio);
  wasActive = (ratio != 0.f && motionID > -1);
  startComp = (ratio != 0.f && lastRatio <= ratio);
  lastRatio = ratio;
}

bool BlameData::activateNewMotion(const BikeRequest& br, const bool& isLeavingPossible)
{
  if(!wasActive /*|| ((br.bMotionType != currentBikeRequest.bMotionType || br.mirror != currentBikeRequest.mirror) && isLeavingPossible)) && !willBeLeft*/)
    return true;
  else if(br.bMotionType == currentBikeRequest.bMotionType && br.mirror == currentBikeRequest.mirror)
    currentBikeRequest = br; // update BikeRequest when it is compatible to the current motion

  return false;
}

Pose3D BlameData::calcDesBodyAngle(JointRequest& jointData, const RobotDimensions& robotDimensions, JointData::Joint joint)
{
  Pose3D footPos;
  float sign = joint == JointData::LHipYawPitch ? -1.f : 1.f;

  footPos.translate(0, 0, robotDimensions.heightLeg5Joint)
  .rotateX(-jointData.angles[joint + 5]*sign)
  .rotateY(-jointData.angles[joint + 4])
  .translate(0, 0, robotDimensions.lowerLegLength)
  .rotateY(-jointData.angles[joint + 3])
  .translate(0, 0, robotDimensions.upperLegLength)
  .rotateY(-jointData.angles[joint + 2])
  .rotateX(((-jointData.angles[joint + 1] - pi_4)*sign))
  .rotateZ(-jointData.angles[joint]*sign)
  .rotateX(-pi_4 * sign);

  return footPos;
}





bool BlameData::sitOutTransitionDisturbance(bool& compensate, bool& compensated, const FilteredSensorData& sd, BikeEngineOutput& blameOutput, const JointData& jd, const FrameInfo& frame)
{
  if(compensate)
  {
    if(!startComp)
    {
      timeStamp = frame.time;
      lSupp = false;
      rSupp = false;
      toLeftSupport = false;
      comOffset = Vector2<>(0.f, 0.f);
      origin = Vector2<>(0.f, 0.f);
      balanceSum = Vector2<>(0.f, 0.f);
      gyro = Vector2<>(0.f, 0.f);
      lastGyroLeft = Vector2<>(0.f, 0.f);
      lastGyroRight = Vector2<>(0.f, 0.f);
      gyroErrorLeft = Vector2<>(0.f, 0.f);
      gyroErrorRight = Vector2<>(0.f, 0.f);
      bodyError = Vector2<>(0.f, 0.f);
      lastBody = Vector2<>(0.f, 0.f);
      lastCom = Vector3<>(0.f, 0.f, 0.f);
      motionID = -1;

      blameOutput.isLeavingPossible = false;

      for(int i = 0; i < JointData::numOfJoints; i++)
      {
        lastBalancedJointRequest.angles[i] = jd.angles[i];
        compenJoints.angles[i] = jd.angles[i];
      }
    }

    for(int i = 0; i < JointData::numOfJoints; i++)
    {
      blameOutput.angles[i] = compenJoints.angles[i];
      blameOutput.jointHardness.hardness[i] = 100;
    }

    int time = frame.getTimeSince(timeStamp);
    if((abs(sd.data[SensorData::gyroX]) < 0.1f && abs(sd.data[SensorData::gyroY]) < 0.1f && time > 200) || time > 1000)
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
