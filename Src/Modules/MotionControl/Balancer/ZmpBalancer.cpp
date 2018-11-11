/**
 * @file ZmpBalancer.cpp
 *
 * This file implements balancing methods using zmp and previewController based on
 * A. Tsogias, 2016, Laufen für den NAO-Roboter mittels Zero-Moment Point Preview Control
 * A. Böckmann, 2015, Entwicklung einer dynamischen Schussbewegung mit dem humanoiden Roboter NAO
 *
 * @author <A href="mailto:enno.roehrig@gmx.de">Enno Röhrig</A>
 *
 * @date 04 Mar 2017
 */

#include <algorithm>
#include "Tools/Streams/OutStreams.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"

#include "ZmpBalancer.h"

#include "Tools/Motion/LIP3D.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/RobotParts/Limbs.h"
#include "Tools/RobotParts/Limbs.h"

#include "Tools/Range.h"
#include "Tools/Math/Angle.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Platform/Time.h"
#include <cmath>

MAKE_MODULE(ZmpBalancer, motionControl);

void ZmpBalancer::update(Balancer& representation)
{
  DECLARE_DEBUG_DRAWING3D("module:ZmpBalancer:coordinates", "robot");
  representation.initWithTarget = [this, &representation](bool rightIsSupportFoot, const Pose3f& targetZMP, float initializationTime, const BalancingParameter& param) -> void { return initWithTarget(rightIsSupportFoot, targetZMP, initializationTime, param, representation); };
  representation.init = [this, &representation](bool rightIsSupportFoot, const BalancingParameter& param) -> Pose3f { return init(rightIsSupportFoot, param, representation); };

  representation.addBalance = [this, &representation](JointRequest& jointRequest, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param) -> bool
  {
    return addBalance(jointRequest, zmpPreviewsX, zmpPreviewsY, param, representation);
  };
  representation.balanceJointRequest = [this, &representation](JointRequest& jointRequest, const BalancingParameter& param) -> bool
  {
    return balanceJointRequest(jointRequest, param, representation);
  };
  representation.calcBalancedJoints = [this, &representation](JointRequest& jointRequest, const Pose3f& swingFootInSupport, float comHeight, const RotationMatrix& torsoRotation, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param) -> bool
  {
    return calcBalancedJoints(jointRequest, swingFootInSupport, comHeight, torsoRotation, zmpPreviewsX, zmpPreviewsY, param, representation);
  };
}
ZmpBalancer::ZmpBalancer():
  zmpControllerX(), zmpControllerY(), estimator(theInertialData, theRobotModel)
{}

Pose3f ZmpBalancer::calcComInStand(const JointAngles& angles)
{
  RobotModel robotModel(angles, theRobotDimensions, theMassCalibration);
  Pose3f supportAnkleTorso = rightIsSupportFoot ? robotModel.limbs[Limbs::footRight] : robotModel.limbs[Limbs::footLeft];
  Pose3f supportAnkleInCom = Pose3f(robotModel.centerOfMass).inverse() * supportAnkleTorso;

  Pose3f comInStand = ankleInStand * supportAnkleInCom.inverse();
  return comInStand;
}

void ZmpBalancer::calcJointAngles(const Pose3f& comInStand, const Pose3f& swingInSupoort, const Quaternionf& torsoRotation, const JointAngles& theJointAngles, JointRequest& jointRequest)
{
  JointAngles tmp(theJointAngles);
  Pose3f supportAnkleInCom = (standInAnkle * comInStand).inverse();

  Pose3f rightAnkleCom(supportAnkleInCom);
  Pose3f leftAnkleCom(supportAnkleInCom);
  if(rightIsSupportFoot)
    leftAnkleCom *= swingInSupoort;
  else
    rightAnkleCom *= swingInSupoort;

  /*Up to here we have foot in com position, but we need foot in torso position and
   there is no fast forward solution to compute ComInTorso, so its calculated iterative*/
  for(int i = 0; ; ++i)
  {
    //Compute Joint Angles
    leftAnkleTorso = Pose3f(comInTorso) * leftAnkleCom;
    rightAnkleTorso = Pose3f(comInTorso) * rightAnkleCom;
    (void)InverseKinematic::calcLegJoints(leftAnkleTorso, rightAnkleTorso, tmp, theRobotDimensions, rightIsSupportFoot ? 0.0f : 1.f);

    //Apply Joint Limits
    JointAngles cuttedAngles(tmp);
    for(int j = Joints::headYaw; j < Joints::rAnkleRoll; j++)
      cuttedAngles.angles[j] = theJointLimits.limits[j].limit(cuttedAngles.angles[j]);

    //Consider possible changes of swing leg after calling the balancer
    for(int j = firstSwingLegJoint; j < firstSwingLegJoint + 6; j++)
      cuttedAngles.angles[j] = theJointAngles.angles[j];

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
  leftAnkleTorso = Pose3f(comInTorso) * leftAnkleCom;
  rightAnkleTorso = Pose3f(comInTorso) * rightAnkleCom;
  (void)InverseKinematic::calcLegJoints(leftAnkleTorso, rightAnkleTorso, jointRequest, theRobotDimensions, rightIsSupportFoot ? 0.0f : 1.f);
}

Pose3f ZmpBalancer::init(bool rightIsSupportFoot, const BalancingParameter& param, Balancer& representation)
{
  this->rightIsSupportFoot = rightIsSupportFoot;
  mirrorConstants(rightIsSupportFoot);

  comInTorso = Vector3f::Zero(3);
  initialZMPTrajectoryX.clear();
  initialZMPTrajectoryY.clear();
  initializationSteps = 1; //not null because of division
  torsoRotationOffset = Vector3f(0, 0, 0);
  lastAnkleRoll = theJointAngles.angles[sAnkleRoll];
  lastKnee = theJointAngles.angles[sKneePitch];

  //calculate current com and torso height
  Pose3f supportAnkle = rightIsSupportFoot ? theRobotModel.limbs[Limbs::footRight] : theRobotModel.limbs[Limbs::footLeft];
  Pose3f comInStand = calcComInStand(theJointAngles);
  this->initialComHeight = comHeight = comInStand.translation.z();

  //initialize ZmpController
  zmpControllerX.reset(comHeight, Constants::motionCycleTime, param.previewControllerX);
  zmpControllerY.reset(comHeight, Constants::motionCycleTime, param.previewControllerY);

  //initialize state and state estimator
  estimator.init(Array2f(comHeight, comHeight), leftLIPOrigin, param.lipStateEstimator);
  LIPStateEstimator::EstimatedState state = estimator.getEstimate();
  LIPStateEstimator::SupportFoot estimatedFoot = estimator.guessSupportFoot(leftLIPOrigin);
  if((estimatedFoot == LIPStateEstimator::SupportFoot::left && rightIsSupportFoot) || (estimatedFoot == LIPStateEstimator::SupportFoot::right && !rightIsSupportFoot))
    state = estimator.convertToOtherFoot(state);

  stateX = Vector3f(comInStand.translation.x(), state.com.velocity.x(), state.zmp.x());
  stateY = Vector3f(comInStand.translation.y(), state.com.velocity.y(), state.zmp.y());

  //compute initial tilt
  //Pose3f realSupportSole = rightIsSupportFoot ? theRobotModel.soleRight : theRobotModel.soleLeft;
  //Vector3f modelAngles = realSupportSole.inverse().rotation.getPackedAngleAxis();
  //tilt = Vector2f(theInertialData.angle.x() - modelAngles.x(), theInertialData.angle.y() - modelAngles.y());
  tilt = Vector2f(0, 0);

  //Calculate current body rotation under assumption of a working balancer to avoid jumps in jointAngles
  float stabilizationFactor = Rangef(0.f, 1.f).limit(1.f - std::abs(stateY[0]) / param.stabilizationRange);
  RotationMatrix curBodyRotation = AngleAxisf(supportAnkle.invert().rotation.getPackedAngleAxis().x() + param.balanceWithHip.x() *  tilt.x() * stabilizationFactor + (sign * 50 + stateY(0)) *   param.oneLeggedTorsoRotation / 50.f, Vector3f::UnitX());
  curBodyRotation *= AngleAxisf(supportAnkle.invert().rotation.getPackedAngleAxis().y() + param.balanceWithHip.y() *  tilt.y(), Vector3f::UnitY());

  return Pose3f(curBodyRotation, Vector3f(stateX(0), stateY(0), comHeight));
}

void ZmpBalancer::initWithTarget(bool rightIsSupportFoot, const Pose3f& targetZMP, float initializationTime, const BalancingParameter& param, Balancer& representation)
{
  Pose3f curZmp = init(rightIsSupportFoot, param, representation);
  initialComHeight = targetZMP.translation.z();
  tilt = Vector2f(0, 0);

  //Compute initial trajectory of zero moment points (zmps) to reach a stable position
  if(std::abs(stateX(0) - targetZMP.translation.x()) > std::abs(stateY(0) - targetZMP.translation.y()))
  {
    initialZMPTrajectoryX = calcZMPTrajectory(stateX, targetZMP.translation.x(), 0.5f / initializationTime, 100);
    initialZMPTrajectoryY = calcZMPTrajectory(stateY, sign * targetZMP.translation.y(), 0.5f / initializationTime, initialZMPTrajectoryX.size() * 10.f);
  }
  else
  {
    initialZMPTrajectoryY = calcZMPTrajectory(stateY, sign * targetZMP.translation.y(), 0.5f / initializationTime, 100);
    initialZMPTrajectoryX = calcZMPTrajectory(stateX, targetZMP.translation.x(), 0.5f / initializationTime, initialZMPTrajectoryY.size() * 10.f);
  }
  initializationSteps = initialZMPTrajectoryX.size();

  //to avoid jumps in torso rotation
  Pose3f supportAnkle = rightIsSupportFoot ? theRobotModel.limbs[Limbs::footRight] : theRobotModel.limbs[Limbs::footLeft];
  torsoRotationOffset(0) = supportAnkle.invert().rotation.getPackedAngleAxis().x() + (sign * 50 + stateY(0)) *   param.oneLeggedTorsoRotation / 50.f;
  torsoRotationOffset(1) = supportAnkle.invert().rotation.getPackedAngleAxis().y();
  torsoRotationOffset(2) = 0;

  assert(initialZMPTrajectoryY.size() == initialZMPTrajectoryX.size());
}

std::vector<float> ZmpBalancer::calcZMPTrajectory(const Vector3f& state, float targetPos, float maxAcceleration, float minEndTime)
{
  //using a quadratic function to calculate the best trajectory under consideration of a maximal acceleration and current celocity
  float startPos = state(0);
  float curVelocity = startPos < 0 ? 0.1f : -0.1f;
  if((targetPos - startPos) < 0)
    maxAcceleration *= -1;

  float p_2 = curVelocity / maxAcceleration;
  float p2_q = (targetPos - startPos) / maxAcceleration;
  float cp = -p_2 + std::sqrt(p2_q);

  std::vector<float> ZMPs;
  ZMPs.push_back(startPos);
  bool finished = false;
  for(int t = 10; t < minEndTime || !finished; t += 10)
  {
    float value = startPos;
    if(t < cp)
      value += t * t * maxAcceleration / 2.f + curVelocity * t;
    else
      value += -cp * cp * maxAcceleration  + 2 * cp * maxAcceleration * t - t * t * maxAcceleration / 2.f + curVelocity * t;
    finished = t >= cp && maxAcceleration * (value - ZMPs[ZMPs.size() - 1]) <= 0; //veclocity sign changed
    if(finished)
      value = targetPos;
    ZMPs.push_back(value);
  }
  return ZMPs;
}

bool ZmpBalancer::balanceJointRequest(JointRequest& jointRequest, const BalancingParameter& param, Balancer& representation)
{
  //compute requested zmp values
  Pose3f com = calcComInStand(jointRequest);
  std::vector<float> zmpX, zmpY;
  //zmpX.push_back(com.translation.x());
  zmpX.push_back(0.f);
  zmpY.push_back(com.translation.y());
  return addBalance(jointRequest, zmpX, zmpY, param, representation);
}

bool ZmpBalancer::addBalance(JointRequest& jointRequest, const std::vector<float>& zmpPreviewsX, const std::vector<float>& zmpPreviewsY, const BalancingParameter& param, Balancer& representation)
{
  //compute swing foot position relative to support foot
  RobotModel robotModel(jointRequest, theRobotDimensions, theMassCalibration);
  Pose3f leftInRight = robotModel.limbs[Limbs::footRight].inverse() * robotModel.limbs[Limbs::footLeft];
  Pose3f swingFootInSupport = rightIsSupportFoot ? leftInRight  : leftInRight.inverse();
  Pose3f com = calcComInStand(jointRequest);

  return calcBalancedJoints(jointRequest, swingFootInSupport, com.translation.z(), com.rotation, zmpPreviewsX, zmpPreviewsY, param, representation);
}

bool ZmpBalancer::calcBalancedJoints(JointRequest& jointRequest, const Pose3f& swingInSupport, float requiredComHeight, const RotationMatrix& bodyRotation, const std::vector<float>& nextZMPPreviewsX, const std::vector<float>& nextZMPPreviewsY, const BalancingParameter& param, Balancer& representation)
{
  integrateDamageConfig(jointRequest);

  float initTime = (Rangef(0.f, 1.f).limit(1.f - static_cast<float>(initialZMPTrajectoryX.size()) / static_cast<float>(initializationSteps)));
  if(initTime < 1.f) //initialization not finished
    comHeight += (initialComHeight - comHeight) / (initialZMPTrajectoryX.size() + 1);
  else
    comHeight = requiredComHeight;

  // update tilt calibration (could be changed via simulator)
  Vector2f tiltCalibration = rightIsSupportFoot ? theDamageConfigurationBody.startTiltRight : theDamageConfigurationBody.startTiltLeft;
  tiltCalibration += rightIsSupportFoot ? param.tiltCalibrationRight : param.tiltCalibrationLeft;

  // update model
  LIP3D com(Vector2f(stateX(0), stateY(0)), Vector2f(stateX(1), stateY(1)), Vector2f(comHeight, comHeight));
  estimator.update(Constants::motionCycleTime, Array2f(comHeight, comHeight), leftLIPOrigin);
  LIPStateEstimator::EstimatedState state = estimator.getEstimate();
  LIPStateEstimator::SupportFoot estimatedFoot = estimator.guessSupportFoot(leftLIPOrigin);
  if((estimatedFoot == LIPStateEstimator::SupportFoot::left && rightIsSupportFoot) || (estimatedFoot == LIPStateEstimator::SupportFoot::right && !rightIsSupportFoot))
    state = estimator.convertToOtherFoot(state);

  stateX = Vector3f(com.position.x() * (1 - param.sensorFeedbackX.x()) + param.sensorFeedbackX.x() * state.com.position.x(), com.velocity.x() * (1 - param.sensorFeedbackX.y()) + param.sensorFeedbackX.y() * state.com.velocity.x(), stateX(2) * (1 - param.sensorFeedbackX.z()) + param.sensorFeedbackX.z() * state.zmp.x());
  stateY = Vector3f(com.position.y() * (1 - param.sensorFeedbackY.x()) + param.sensorFeedbackY.x() * state.com.position.y(), com.velocity.y() * (1 - param.sensorFeedbackY.y()) + param.sensorFeedbackY.y() * state.com.velocity.y(), stateY(2) * (1 - param.sensorFeedbackY.z()) + param.sensorFeedbackY.z() * state.zmp.y());

  // control position using ZMPController
  VectorXf zmpPreviewsX = VectorXf::Zero(param.previewControllerX.numOfZmpPreviews);
  VectorXf zmpPreviewsY = VectorXf::Zero(param.previewControllerY.numOfZmpPreviews);

  int indexOffset = param.strictInitialTrajectory ? 4 : 0;
  unsigned int initialSize = std::max(static_cast<int>(initialZMPTrajectoryX.size()) - indexOffset, 0);
  for(unsigned int i = 0; i < param.previewControllerX.numOfZmpPreviews; i++)
  {
    if(i < initialSize)
      zmpPreviewsX[i] = initialZMPTrajectoryX[i + indexOffset];
    else if(i < nextZMPPreviewsX.size() + initialSize)
      zmpPreviewsX[i] = nextZMPPreviewsX[i - initialSize];
    else
      zmpPreviewsX[i] = zmpPreviewsX[i - 1]; // fill up with the last value
  }

  initialSize = std::max(static_cast<int>(initialZMPTrajectoryY.size()) - indexOffset, 0);
  for(unsigned int i = 0; i < param.previewControllerY.numOfZmpPreviews; i++)
  {
    if(i < initialSize)
      zmpPreviewsY[i] = initialZMPTrajectoryY[i + indexOffset];
    else if(i < nextZMPPreviewsY.size() + initialSize)
      zmpPreviewsY[i] = nextZMPPreviewsY[i - initialSize];
    else
      zmpPreviewsY[i] = zmpPreviewsY[i - 1];
  }

  if(initialZMPTrajectoryY.size() > 0 && (!param.strictInitialTrajectory || (initialZMPTrajectoryY[0] > state.com.position.y() && rightIsSupportFoot) || (!rightIsSupportFoot && initialZMPTrajectoryY[0] < state.com.position.y()))) //last zmp position reached
  {
    initialZMPTrajectoryY.erase(initialZMPTrajectoryY.begin());
    initialZMPTrajectoryX.erase(initialZMPTrajectoryX.begin());
  }

  if(param.usePreviewController)
  {
    stateX = zmpControllerX.control(stateX, zmpPreviewsX, comHeight, param.previewControllerX);
    stateY = zmpControllerY.control(stateY, zmpPreviewsY, comHeight, param.previewControllerY);
  }
  else
  {
    stateX(0) = zmpPreviewsX[0];
    stateY(0) = zmpPreviewsY[0];
  }

  float stabilizationFactor = Rangef(0.f, 1.f).limit(1.f - std::min(std::abs(zmpPreviewsY[0]), std::abs(stateY[0])) / param.stabilizationRange) * initTime;

  // compute current tilt
  Pose3f realSupportSole = rightIsSupportFoot ? theRobotModel.soleRight : theRobotModel.soleLeft;
  Vector3f modelAngles = realSupportSole.inverse().rotation.getPackedAngleAxis();
  Vector2f currentTilt = Vector2f(theInertialData.angle.x() - modelAngles.x(), theInertialData.angle.y() - modelAngles.y()) - tiltCalibration;
  Vector2f tiltDiff = currentTilt - tilt;

  Vector2f tiltFeedbackFactor = Vector2f(0, 0);
  tiltFeedbackFactor(0) = std::abs(tiltDiff.x()) > param.fallingTiltFeedback.x() || std::abs(currentTilt.x()) > param.fallingTiltFeedback.x() ? param.fallingTiltFeedback.x() : param.tiltFeedback.x() * std::sqrt(std::max(std::abs(currentTilt.x()), std::abs(tiltDiff.x()))) / param.maxTilt;
  tiltFeedbackFactor(1) = std::abs(tiltDiff.y()) > param.fallingTiltFeedback.y() || std::abs(currentTilt.y()) > param.fallingTiltFeedback.y() ? param.fallingTiltFeedback.y() : param.tiltFeedback.y() * std::sqrt(std::max(std::abs(currentTilt.y()), std::abs(tiltDiff.y()))) / param.maxTilt;
  tilt.array() += tiltDiff.array()  * tiltFeedbackFactor.array();
  tilt(0) = Rangea(-param.maxTilt, param.maxTilt).limit(tilt(0));
  tilt(1) = Rangea(-param.maxTilt, param.maxTilt).limit(tilt(1));

  if(!param.useIMU)
    tilt = Vector2f(0, 0);

  // Compute JointAngles
  RotationMatrix torsoRotation(bodyRotation);
  //RotationMatrix torsoRotation = AngleAxisf(0, Vector3f::UnitY());
  torsoRotation *= AngleAxisf((1.f - initTime) * torsoRotationOffset.y(), Vector3f::UnitY());
  torsoRotation *= AngleAxisf((1.f - initTime) * torsoRotationOffset.x(), Vector3f::UnitX());
  torsoRotation *= AngleAxisf(-(sign * 50 + zmpPreviewsY[0]) *  param.oneLeggedTorsoRotation / 50.f, Vector3f::UnitX());
  torsoRotation *= AngleAxisf(-param.balanceWithHip.x() *  tilt.x() * stabilizationFactor, Vector3f::UnitX());
  torsoRotation *= AngleAxisf(-param.balanceWithHip.y() *  tilt.y(), Vector3f::UnitY());
  Pose3f comInStand(torsoRotation, Vector3f(stateX(0), stateY(0), comHeight));
  calcJointAngles(comInStand, swingInSupport, Quaternionf(torsoRotation), theJointAngles, jointRequest);

  //correct swing foot rotation relative to ground
  Pose3f currentTibiaInSupport = theRobotModel.limbs[Limbs::footLeft].inverse() * theRobotModel.limbs[Limbs::tibiaRight];
  if(rightIsSupportFoot)
    currentTibiaInSupport = theRobotModel.limbs[Limbs::footRight].inverse() * theRobotModel.limbs[Limbs::tibiaLeft];

  Pose3f requiredSwingFootInStand = swingInSupport;
  Pose3f requiredSwingFootInSupport = requiredSwingFootInStand;// .rotateX(-tilt[0] * 0.9f).rotateY(-tilt[1] * 0.6f);
  Pose3f requiredFootInTibia = currentTibiaInSupport.inverse() * requiredSwingFootInSupport;
  Vector3f requiredAnglesInTibia = requiredFootInTibia.rotation.getPackedAngleAxis();
  jointRequest.angles[swingAnkleRoll] = 0.6f * jointRequest.angles[swingAnkleRoll] + requiredAnglesInTibia[0] * 0.4f;
  jointRequest.angles[swingAnklePitch] = 0.6f * jointRequest.angles[swingAnklePitch] + requiredAnglesInTibia[1] * 0.4f;

  //compensate floor slopes and calibration errors
  jointRequest.angles[sAnkleRoll] += tiltCalibration.x() * stabilizationFactor;
  jointRequest.angles[sAnklePitch] += tiltCalibration.y() * stabilizationFactor;

  //balance with arms

  jointRequest.angles[Joints::lShoulderRoll] += param.balanceWithArms.x() *  tilt.x() * stabilizationFactor;
  jointRequest.angles[Joints::rShoulderRoll] += param.balanceWithArms.x() *  tilt.x() * stabilizationFactor;
  if(jointRequest.angles[Joints::lShoulderPitch] > 0)
    jointRequest.angles[Joints::lShoulderPitch] = std::max(0.f, param.balanceWithArms.y() *  tilt.y() * stabilizationFactor + jointRequest.angles[Joints::lShoulderPitch]);
  if(jointRequest.angles[Joints::rShoulderPitch] > 0)
    jointRequest.angles[Joints::rShoulderPitch] = std::max(0.f, param.balanceWithArms.y() *  tilt.y() * stabilizationFactor + jointRequest.angles[Joints::rShoulderPitch]);

  jointRequest.angles[Joints::lShoulderRoll] = Rangea(8_deg, 65_deg).limit(jointRequest.angles[Joints::lShoulderRoll]);
  jointRequest.angles[Joints::rShoulderRoll] = Rangea(-65_deg, -8_deg).limit(jointRequest.angles[Joints::rShoulderRoll]);

  //balance with swing-leg and torso if neccessary
  float criticalTiltX = tilt.x() - Rangea(-param.criticalTiltThreshold, param.criticalTiltThreshold).limit(tilt.x());
  jointRequest.angles[swingHipRoll] += -param.critBalanceWithLeg.x() * criticalTiltX * stabilizationFactor;
  jointRequest.angles[sHipRoll] += -param.critBalanceWithHip.x() * criticalTiltX * stabilizationFactor;

  //compensate loose AnkleRoll
  Angle ankleDiff = lastAnkleRoll - theJointAngles.angles[sAnkleRoll];
  lastAnkleRoll = jointRequest.angles[sAnkleRoll];
  if(initialZMPTrajectoryX.size() == 0)
  {
    jointRequest.angles[sHipRoll] += param.weakAnkleTorso * ankleDiff;
    jointRequest.angles[sAnkleRoll] += param.counterWeakAnkle * ankleDiff;
  }

  //compensate loose Knee
  Angle kneeDiff = theJointRequest.angles[sKneePitch] - theJointAngles.angles[sKneePitch];
  if(initialZMPTrajectoryX.size() == 0)
    jointRequest.angles[sHipPitch] += param.weakKneeTorso * kneeDiff;

  // Debug stuff
  COMPLEX_DRAWING3D("module:ZmpBalancer:coordinates")
  {
    Pose3f torsoToGround = realSupportSole; //correct floor slope

    //estimated com (projected to Stand)
    Pose3f comInStand = Pose3f(state.com.position.x(), state.com.position.y(), 0);
    Pose3f point = realSupportSole * standInSole * comInStand;
    drawCoordinateSystem(point, ColorRGBA::red, ColorRGBA::red, ColorRGBA::red);

    drawCoordinateSystem(theRobotModel.centerOfMass, ColorRGBA::blue, ColorRGBA::green, ColorRGBA::yellow); //com

    //set com (projected to Bottom)
    Vector3f transToBottom = RotationMatrix(torsoToGround.rotation)  *  Vector3f(0, 0, -comHeight);
    point = Pose3f(transToBottom) * theRobotModel.centerOfMass;
    point.rotation = torsoToGround.rotation;
    Pose3f pointX = point * Pose3f(Vector3f(50, 0, 0));
    Pose3f pointY = point * Pose3f(Vector3f(0, 50, 0));
    Pose3f pointZ = point *  Pose3f(Vector3f(0, 0, 300));
    LINE3D("module:ZmpBalancer:coordinates", point.translation.x(), point.translation.y(), point.translation.z(), pointZ.translation.x(), pointZ.translation.y(), pointZ.translation.z(), 4, ColorRGBA::yellow);

    drawCoordinateSystem(leftAnkleTorso * soleInAnkle * standInSole, ColorRGBA::blue, ColorRGBA::green, ColorRGBA::yellow);    //left
    drawCoordinateSystem(rightAnkleTorso  * soleInAnkle * standInSole, ColorRGBA::blue, ColorRGBA::green, ColorRGBA::yellow);  //right
  }

  //write logging data
  representation.tiltX = tilt.x();
  representation.tiltY = tilt.y();
  representation.stabilizationFactor = stabilizationFactor;
  representation.comX = state.com.position.x();
  representation.comY = state.com.position.y();
  representation.comXvel = state.com.velocity.x();
  representation.comYvel = state.com.velocity.y();
  representation.kneeDiff = kneeDiff;

  /*/ Debug Output
  Pose3f real = theRobotModel.soleLeft.inverse() * theRobotModel.soleRight;
  double kmc_diff = sqrt(swingInSupport.translation.x() * swingInSupport.translation.x() + swingInSupport.translation.y() * swingInSupport.translation.y() + swingInSupport.translation.z() * swingInSupport.translation.z());
  double real_diff = sqrt(real.translation.x() * real.translation.x() + real.translation.y() * real.translation.y() + real.translation.z() * real.translation.z());
  bool hit = theKeyStates.pressed[KeyStates::leftFootLeft] || theKeyStates.pressed[KeyStates::leftFootRight] || theKeyStates.pressed[KeyStates::rightFootLeft] || theKeyStates.pressed[KeyStates::rightFootRight];
  Pose3f MyComInStand = calcComInStand(theJointAngles);
  OUTPUT_TEXT("" << swingInSupport.translation.x() << "," << swingInSupport.translation.y() << "," << swingInSupport.translation.z() << "," << real.translation.x() << "," << real.translation.y() << "," << kmc_diff << "," << real_diff << "," << real.translation.z() << "," << comInStand.translation.y() << "," << static_cast<int>(hit) << "," << state.com.position.y() << "," << initialZMPTrajectoryY[0] << "," << stateY(0) << "," << state.com.position.x());
  //*/
  return((theFootSupport.support < -0.4 && rightIsSupportFoot) || (theFootSupport.support > 0.4 && !rightIsSupportFoot)) && initialZMPTrajectoryY.size() < 5;
}

void ZmpBalancer::integrateDamageConfig(JointRequest& jointRequest)
{
  for(Joints::Joint joint : theDamageConfigurationBody.jointsToEraseStiffness)
    jointRequest.angles[joint] = theJointAngles.angles[joint];
}

void ZmpBalancer::mirrorConstants(bool rightIsSupportFoot)
{
  sign = rightIsSupportFoot ? -1.f : 1.f;
  sHipPitch = rightIsSupportFoot ? Joints::rHipPitch : Joints::lHipPitch;
  sHipRoll = rightIsSupportFoot ? Joints::rHipRoll : Joints::lHipRoll;
  swingHipRoll = !rightIsSupportFoot ? Joints::rHipRoll : Joints::lHipRoll;
  sKneePitch = rightIsSupportFoot ? Joints::rKneePitch : Joints::lKneePitch;
  sAnkleRoll = rightIsSupportFoot ? Joints::rAnkleRoll : Joints::lAnkleRoll;
  sAnklePitch = rightIsSupportFoot ? Joints::rAnklePitch : Joints::lAnklePitch;
  swingAnkleRoll = !rightIsSupportFoot ? Joints::rAnkleRoll : Joints::lAnkleRoll;
  swingAnklePitch = !rightIsSupportFoot ? Joints::rAnklePitch : Joints::lAnklePitch;
  firstSupportLegJoint = rightIsSupportFoot ? Joints::firstRightLegJoint : Joints::firstLeftLegJoint;
  firstSwingLegJoint = rightIsSupportFoot ? Joints::firstLeftLegJoint : Joints::firstRightLegJoint;

  //Prepare for further computations
  ankleInStand = Pose3f(-leftLIPOrigin.x(), rightIsSupportFoot ? leftLIPOrigin.y() : -leftLIPOrigin.y(), theRobotDimensions.footHeight);
  ankleInSole = Pose3f(0, 0, theRobotDimensions.footHeight);
  soleInStand = Pose3f(-leftLIPOrigin.x(), rightIsSupportFoot ? leftLIPOrigin.y() : -leftLIPOrigin.y(), 0);

  standInAnkle = ankleInStand.inverse();
  soleInAnkle = ankleInSole.inverse();
  standInSole = soleInStand.inverse();
}

void ZmpBalancer::drawCoordinateSystem(Pose3f point, ColorRGBA colorX, ColorRGBA colorY, ColorRGBA colorZ)
{
  Pose3f pointX = point * Pose3f(Vector3f(50, 0, 0));
  Pose3f pointY = point * Pose3f(Vector3f(0, 50, 0));
  Pose3f pointZ = point *  Pose3f(Vector3f(0, 0, 50));
  LINE3D("module:ZmpBalancer:coordinates", point.translation.x(), point.translation.y(), point.translation.z(), pointX.translation.x(), pointX.translation.y(), pointX.translation.z(), 4, colorX);
  LINE3D("module:ZmpBalancer:coordinates", point.translation.x(), point.translation.y(), point.translation.z(), pointY.translation.x(), pointY.translation.y(), pointY.translation.z(), 4, colorY);
  LINE3D("module:ZmpBalancer:coordinates", point.translation.x(), point.translation.y(), point.translation.z(), pointZ.translation.x(), pointZ.translation.y(), pointZ.translation.z(), 4, colorZ);
}
