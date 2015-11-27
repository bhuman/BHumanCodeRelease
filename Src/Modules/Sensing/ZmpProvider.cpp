
#include "ZmpProvider.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"
#include <iostream>

using namespace std;

MAKE_MODULE(ZmpProvider, sensing)

void ZmpProvider::update(Zmp& zmp)
{
  JointRequest jointRequest = theJointRequest;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    if(jointRequest.angles[i] == JointRequest::off)
    {
      jointRequest.angles[i] = theJointDataPrediction.angles[i];
    }
  }
  if(useRealCom)
  {
    requestModel.setJointData(theJointDataPrediction, theRobotDimensions, theMassCalibration);
  }
  else
  {
    requestModel.setJointData(jointRequest, theRobotDimensions, theMassCalibration);
  }

  Pose3f leftFootInTorso = requestModel.limbs[Limbs::footLeft];
  leftFootInTorso.translate(0, 0, -theRobotDimensions.footHeight);
  const Pose3f torsoInLeftSole = leftFootInTorso.invert();

  Pose3f rightFootInTorso = requestModel.limbs[Limbs::footRight];
  rightFootInTorso.translate(0, 0, -theRobotDimensions.footHeight);
  const Pose3f torsoInRightSole = rightFootInTorso.invert();

  //calculate zmp based on current sensor data including imu
  zmp.zmpInLeftSole = calcZmp(torsoInLeftSole * requestModel.centerOfMass, torsoInLeftSole);
  zmp.zmpInRightSole = calcZmp(torsoInRightSole * requestModel.centerOfMass, torsoInRightSole);

  zmp.rotZmpInLeftSole = calcRotZmp(torsoInLeftSole * requestModel.centerOfMass, torsoInLeftSole);
  zmp.rotZmpInRightSole = calcRotZmp(torsoInRightSole * requestModel.centerOfMass, torsoInRightSole);


  //just for drawing
  zmp.comInLeftSole = (torsoInLeftSole * theRobotModel.centerOfMass).topRows(2);
  zmp.comInRightSole = (torsoInRightSole * theRobotModel.centerOfMass).topRows(2);

  zmp.predComInLeftSole = (torsoInLeftSole * requestModel.centerOfMass).topRows(2);
  zmp.predComInRightSole = (torsoInRightSole * requestModel.centerOfMass).topRows(2);

  zmp.jointZmpInRightSole = calcJointZmp(torsoInRightSole * requestModel.centerOfMass, torsoInRightSole, rightComs);
  zmp.jointZmpInLeftSole = calcJointZmp(torsoInLeftSole * requestModel.centerOfMass, torsoInLeftSole, leftComs);
}

Vector2f ZmpProvider::calcZmp(const Vector3f& comInSole, const Pose3f& torsoInSole) const
{
  //FIXME this is wrong, aldebaran angles are not euler angles!
  RotationMatrix bodyRot = RotationMatrix::fromEulerAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0.0f);

  Vector3f comAccInBody;
  comAccInBody = theInertialSensorData.acc;
  comAccInBody.topRows(2) += accOffset;
  if(useBodyRotAcc)
    comAccInBody = bodyRot * comAccInBody;
  else
    comAccInBody = torsoInSole.rotation * comAccInBody;

  comAccInBody *= 1000.0f;//convert from m/s^2 to mm/s^2
  return comInSole.topRows(2) - comAccInBody.topRows(2) / Constants::g * theMotionSettings.comHeight;
}

Vector2f ZmpProvider::calcJointZmp(const Vector3f& comInSole, const Pose3f& torsoInSole, RingBuffer<Vector3f, 2>& coms)
{

  Vector3f comAccInBody = Vector3f::Zero();
  if(coms.full())
  {
    const Vector3f vel0 = (comInSole - coms[0]) / theFrameInfo.cycleTime;
    const Vector3f vel1 = (coms[0] - coms[1]) / theFrameInfo.cycleTime;
    comAccInBody = (vel0 - vel1) / theFrameInfo.cycleTime;
  }
  coms.push_front(comInSole);

  //FIXME this is wrong, aldebaran angles are not euler angles!
  RotationMatrix bodyRot = RotationMatrix::fromEulerAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0.0f);
  if(useBodyRotAcc)
    comAccInBody = bodyRot * comAccInBody;
  else
    comAccInBody = torsoInSole.rotation * comAccInBody;

  //comAccInBody = Vector3f::Zero();
  return comInSole.topRows(2) - comAccInBody.topRows(2) / Constants::g * theMotionSettings.comHeight;

}

Vector2f ZmpProvider::calcRotZmp(const Vector3f& comInSole, const Pose3f& torsoInSole) const
{
  const float xRotDiff = theInertialSensorData.angle.x() - torsoInSole.rotation.getXAngle();
  const float yRotDiff = theInertialSensorData.angle.y() - torsoInSole.rotation.getYAngle();

  const float comX = comInSole.x() + comP.y() * yRotDiff;
  const float comY = comInSole.y() + comP.x() * xRotDiff;

  const Vector2f com(comX, comY);

  RotationMatrix bodyRot = RotationMatrix::fromEulerAngles(theInertialSensorData.angle.x(), theInertialSensorData.angle.y(), 0.0f);

  Vector3f comAccInBody;
  comAccInBody = theInertialSensorData.acc;
  comAccInBody.topRows(2) += accOffset;
  if(useBodyRotAcc)
    comAccInBody = bodyRot * comAccInBody;
  else
    comAccInBody = torsoInSole.rotation * comAccInBody;

  comAccInBody *= 1000.0f;//convert from m/s^2 to mm/s^2
  return com - comAccInBody.topRows(2) / Constants::g * theMotionSettings.comHeight;
}