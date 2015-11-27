/**
 * @file WalkingEngine.cpp
 * Implementation of a module that creates the walking motions
 * @author Colin Graf
 */

#include "WalkingEngine.h"
#include "Tools/SensorData.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>

using namespace std;

MAKE_MODULE(WalkingEngine, motionControl)

PROCESS_LOCAL WalkingEngine* WalkingEngine::theInstance = 0;

WalkingEngine::WalkingEngine() : optimizeStarted(false)
{
  theInstance = this;

  // init() is not called automatically when parameters are first read,
  // because this object has not been constructed at that time. So
  // it has to be called manually.
  init();

  if(SystemCall::getMode() == SystemCall::simulatedRobot)
    observerMeasurementDelay = 60.f;

  // load walking engine kicks
  kicks.load();

  // reset internal state
  reset();
}

bool WalkingEngine::handleMessage(InMessage& message)
{
  WalkingEngine* engine = theInstance;
  if(engine && message.getMessageID() == idWalkingEngineKick)
  {
    unsigned int id, size;
    message.bin >> id >> size;
    ASSERT(id < WalkRequest::numOfKickTypes);
    char* buffer = new char[size + 1];
    message.bin.read(buffer, size);
    buffer[size] = '\0';
    engine->kicks.load(WalkRequest::KickType(id), buffer);
    delete[] buffer;
    return true;
  }
  return false;
}

void WalkingEngine::onRead()
{
  init();
}

void WalkingEngine::init()
{
  // pre compute constants
  walkK.x() = sqrt(Constants::g / walkHeight.x());
  walkK.y() = sqrt(Constants::g / walkHeight.y());
  standBodyRotation = RotationMatrix::aroundY(standBodyTilt);
  walkPhaseDuration = walkStepDuration * (0.001f * 0.5f);
  walkPhaseDurationAtFullSpeedX = walkStepDurationAtFullSpeedX * (0.001f * 0.5f);
  walkPhaseDurationAtFullSpeedY = walkStepDurationAtFullSpeedY * (0.001f * 0.5f);

  // compute walkXvdXSoftLimit and walkXvdXHardLimit

  // td = tt * 0.5f
  // xv0 * sinh(k * td) / k = ns * 0.5f
  // xvd = xv0 * cosh(k * td)

  // xv0 = ns * 0.5f * k / sinh(k * td)

  const float k = walkK.x();
  const float tt = walkPhaseDurationAtFullSpeedX;
  const float td = tt * 0.5f;
  const float coshKTd = cosh(k * td);
  const float sinhKTd = sinh(k * td);
  float ns, xv0;

  ns = walkStepSizeXPlanningLimit.max;
  xv0 = ns * 0.5f * k / sinhKTd;
  walkXvdXPlanningLimit.max = xv0 * coshKTd;
  ASSERT(fabs(xv0 * sinh(k * td) / k - ns * 0.5f) < 0.1f);

  ns = walkStepSizeXPlanningLimit.min;
  xv0 = ns * 0.5f * k / sinhKTd;
  walkXvdXPlanningLimit.min = xv0 * coshKTd;
  ASSERT(fabs(xv0 * sinh(k * td) / k - ns * 0.5f) < 0.1f);

  ns = walkStepSizeXLimit.max;
  xv0 = ns * 0.5f * k / sinhKTd;
  walkXvdXLimit.max = xv0 * coshKTd;
  ASSERT(fabs(xv0 * sinh(k * td) / k - ns * 0.5f) < 0.1f);

  ns = walkStepSizeXLimit.min;
  xv0 = ns * 0.5f * k / sinhKTd;
  walkXvdXLimit.min = xv0 * coshKTd;
  ASSERT(fabs(xv0 * sinh(k * td) / k - ns * 0.5f) < 0.1f);
}

void WalkingEngine::reset()
{
  currentMotionType = stand;
  requestedMotionType = stand;

  pendulumPlayer.engine = this;
  generateFirstPendulumPhase(pendulumPlayer.phase);
  generateNextPendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase);
  pendulumPlayer.seek(0.f);
  lastRequestedSpeedRel = Pose2f();

  lastExpectedLeftToCom = Vector3f::Zero();
  lastExpectedRightToCom = Vector3f::Zero();

  lastGyroErrorY = 0.0f;
  lastSmoothedGyroY = SensorData::off;
}

void WalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  DECLARE_PLOT("module:WalkingEngine:torsoAngularVelocityY");
  DECLARE_PLOT("module:WalkingEngine:torsoAngularVelocityCorrectionY");
  DECLARE_PLOT("module:WalkingEngine:gyroY");
  DECLARE_PLOT("module:WalkingEngine:smoothedGyroY");
  MODIFY("module:WalkingEngine:optimizeBestParameters", optimizeBestParameters);
  DEBUG_RESPONSE("module:WalkingEngine:optimize")
  {
    if(theMotionSelection.ratios[MotionRequest::walk] > 0.9f)
    {
      if(!optimizeOptimizer.isRunning())
      {
        optimizeOptimizer.addDimension(walkHeight.y(), walkHeight.y() - 200.f, walkHeight.y() + 200.f, 1.f);
        optimizeOptimizer.addDimension(walkRef.y(), walkRef.y() - 20.f, walkRef.y() + 20.f, 0.5f);
        optimizeOptimizer.start();
        static_cast<Parameters&>(optimizeBestParameters) = *this;
      }
      if(optimizeStarted && theFrameInfo.getTimeSince(optimizeStartTime) > 4000)
      {
        const float rating = optimizeFitness.average();
        optimizeFitness.clear();
        optimizeOptimizer.setRating(rating);
        if(rating == optimizeOptimizer.getBestRating())
        {
          OUTPUT_TEXT("optimize: rating=" << rating << " (new optimum)");
          static_cast<Parameters&>(optimizeBestParameters) = *this;
        }
        else
        {
          OUTPUT_TEXT("optimize: rating=" << rating);
        }
        optimizeStarted = false;
      }
      if(!optimizeStarted)
      {
        optimizeStarted = true;
        optimizeStartTime = theFrameInfo.time;

        optimizeOptimizer.next();
        init();
      }
    }
  }
  else if(optimizeStarted)
  {
    optimizeStarted = false;
    static_cast<Parameters&>(*this) = optimizeBestParameters;
  }



  if(theMotionSelection.ratios[MotionRequest::walk] > 0.f || theMotionSelection.ratios[MotionRequest::stand] > 0.f)
  {
    updateMotionRequest();
    updatePendulumPlayer();
    computeMeasuredPosture();
    computeExpectedPosture();
    computeEstimatedPosture();
    computeError();
    correctPendulumPlayer();
    updatePredictedPendulumPlayer();
    generateTargetPosture();
    generateJointRequest();
    computeOdometryOffset();
    generateOutput(walkingEngineOutput);
  }
  else
  {
    reset();
    generateDummyOutput(walkingEngineOutput);
  }

  DEBUG_RESPONSE("module:WalkingEngine:initialPosition")
  {
    for(int i = 0; i < Joints::numOfJoints; ++i)
      jointRequest.angles[i] = walkingEngineOutput.angles[i] = 0.f;
  }

  DEBUG_DRAWING3D("module:WalkingEngine:zmp", "field") drawZmp();

  PLOT("module:WalkingEngine:measuredComX", pendulumPlayer.phase.type == leftSupportPhase ? measuredLeftToCom.x() : measuredRightToCom.x());
  PLOT("module:WalkingEngine:measuredComY", pendulumPlayer.phase.type == leftSupportPhase ? measuredLeftToCom.y() : measuredRightToCom.y());
  PLOT("module:WalkingEngine:expectedComX", pendulumPlayer.phase.type == leftSupportPhase ? expectedLeftToCom.x() : expectedRightToCom.x());
  PLOT("module:WalkingEngine:expectedComY", pendulumPlayer.phase.type == leftSupportPhase ? expectedLeftToCom.y() : expectedRightToCom.y());
  PLOT("module:WalkingEngine:nsx", pendulumPlayer.nextPhase.s.translation.x());

#ifdef TARGET_SIM
  DEBUG_DRAWING3D("module:WalkingEngine:W", "field") drawW();
  DEBUG_DRAWING3D("module:WalkingEngine:P", "robot") drawP();
  DEBUG_DRAWING3D("module:WalkingEngine:Q", "robot") drawQ(walkingEngineOutput);
#endif
  drawStats();
}

void WalkingEngine::updateMotionRequest()
{
  if(theMotionRequest.motion == MotionRequest::walk || theMotionRequest.motion == MotionRequest::stand)
  {
    if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
    {
      if(theMotionRequest.walkRequest.target != Pose2f() && theMotionRequest.walkRequest.target != lastCopiedWalkTarget)
        lastCopiedWalkTarget = requestedWalkTarget = theMotionRequest.walkRequest.target;
    }
  }
  else if(theMotionRequest.walkRequest.target != Pose2f() && theMotionRequest.walkRequest.target != lastCopiedWalkTarget)
    lastCopiedWalkTarget = requestedWalkTarget = theMotionRequest.walkRequest.target;

  // get requested motion state
  requestedMotionType = stand;
  if(theGroundContactState.contact && theMotionSelection.ratios[MotionRequest::walk] >= 1.f)
    if(theMotionRequest.motion == MotionRequest::walk)
    {
      requestedMotionType = stepping;
    }
}

void WalkingEngine::updatePendulumPlayer()
{
  // motion update
  if(pendulumPlayer.phase.type != standPhase || pendulumPlayer.nextPhase.type != standPhase)
  {
    pendulumPlayer.seek(theFrameInfo.cycleTime);
    if(pendulumPlayer.phase.type == standPhase && pendulumPlayer.nextPhase.type == standPhase)
      currentMotionType = stand;
  }

  // change motion type from stand to stepping
  if(currentMotionType == stand && requestedMotionType == stepping)
  {
    generateFirstPendulumPhase(pendulumPlayer.phase);
    generateNextPendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase);
    updatePendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase, true);
    currentMotionType = stepping;
  }
}

void WalkingEngine::computeMeasuredPosture()
{
  measuredLeftToCom = -Pose3f(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[Limbs::footLeft]).translate(0.f, 0.f, -theRobotDimensions.footHeight).translation;
  measuredRightToCom = -Pose3f(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.limbs[Limbs::footRight]).translate(0.f, 0.f, -theRobotDimensions.footHeight).translation;
}

void WalkingEngine::computeExpectedPosture()
{
  LegPosture expectedPosture;
  pendulumPlayer.getPosture(expectedPosture, 0, 0, &observedStepOffset);

  expectedLeftToCom = expectedPosture.leftOriginToCom - expectedPosture.leftOriginToFoot.translation;
  expectedRightToCom = expectedPosture.rightOriginToCom - expectedPosture.rightOriginToFoot.translation;
  expectedComVelocity = pendulumPlayer.phase.xv0;
}

void WalkingEngine::computeEstimatedPosture()
{
  if(lastExpectedLeftToCom.z() == 0.f)
  {
    estimatedLeftToCom = expectedLeftToCom;
    estimatedRightToCom = expectedRightToCom;
    estimatedComVelocity = expectedComVelocity;

    covX << sqr(observerProcessDeviation[0]),                       0,                                                         observerProcessDeviation[0] * observerProcessDeviation[2],
         0,                                                         sqr(observerProcessDeviation[0]),                          observerProcessDeviation[0] * observerProcessDeviation[2],
         observerProcessDeviation[0] * observerProcessDeviation[2], observerProcessDeviation[0] * observerProcessDeviation[2], sqr(observerProcessDeviation[2]);

    covY << sqr(observerProcessDeviation[1]),                       0,                                                          observerProcessDeviation[1] * observerProcessDeviation[3],
         0,                                                         sqr(observerProcessDeviation[1]),                           observerProcessDeviation[1] * observerProcessDeviation[3],
         observerProcessDeviation[1] * observerProcessDeviation[3], observerProcessDeviation[1] * observerProcessDeviation[3], sqr(observerProcessDeviation[3]);
  }
  else
  {
    estimatedLeftToCom += expectedLeftToCom - lastExpectedLeftToCom;
    estimatedRightToCom += expectedRightToCom - lastExpectedRightToCom;
    estimatedComVelocity = expectedComVelocity;
  }

  static const Matrix2x3f c = Matrix2x3f::Identity();
  static const Matrix3x2f cTransposed = c.transpose();
  Matrix3f a;
  a <<
    1, 0, pendulumPlayer.phase.type == leftSupportPhase ? theFrameInfo.cycleTime : 0,
    0, 1, pendulumPlayer.phase.type == rightSupportPhase ? theFrameInfo.cycleTime : 0,
    0, 0, 1;

  covX = a * covX * a.transpose();
  covY = a * covY * a.transpose();

  covX(0, 0) += sqr(observerProcessDeviation[0]);
  covX(1, 1) += sqr(observerProcessDeviation[0]);
  covX(2, 2) += sqr(observerProcessDeviation[2]);
  covY(0, 0) += sqr(observerProcessDeviation[1]);
  covY(1, 1) += sqr(observerProcessDeviation[1]);
  covY(2, 2) += sqr(observerProcessDeviation[3]);

  Matrix2f covXPlusSensorCov = c * covX * cTransposed;
  covXPlusSensorCov(0, 0) += sqr(observerMeasurementDeviation[0]);
  covXPlusSensorCov(1, 1) += sqr(observerMeasurementDeviation[0]);

  Matrix2f covYPlusSensorCov = c * covY * cTransposed;
  covYPlusSensorCov(0, 0) += sqr(observerMeasurementDeviation[1]);
  covYPlusSensorCov(1, 1) += sqr(observerMeasurementDeviation[1]);

  Matrix3x2f kalmanGainX = covX * cTransposed * covXPlusSensorCov.inverse();
  covX -= kalmanGainX * c * covX;
  Vector3f correctionX = kalmanGainX * Vector2f(measuredLeftToCom.x() - estimatedLeftToCom.x(), measuredRightToCom.x() - estimatedRightToCom.x());

  Matrix3x2f kalmanGainY = covY * cTransposed * covYPlusSensorCov.inverse();
  covY -= kalmanGainY * c * covY;
  Vector3f correctionY = kalmanGainY * Vector2f(measuredLeftToCom.y() - estimatedLeftToCom.y(), measuredRightToCom.y() - estimatedRightToCom.y());

  estimatedLeftToCom.x() += correctionX[0];
  estimatedRightToCom.x() += correctionX[1];
  estimatedComVelocity.x() += correctionX[2];

  estimatedLeftToCom.y() += correctionY[0];
  estimatedRightToCom.y() += correctionY[1];
  estimatedComVelocity.y() += correctionY[2];

  lastExpectedLeftToCom = expectedLeftToCom;
  lastExpectedRightToCom = expectedRightToCom;
  lastExpectedComVelocity = expectedComVelocity;
}

void WalkingEngine::computeError()
{
  if(theGroundContactState.contact && theFallDownState.state == FallDownState::upright)
  {
    errorLeft = estimatedLeftToCom - expectedLeftToCom;
    errorRight = estimatedRightToCom - expectedRightToCom;
    errorVelocity = estimatedComVelocity - expectedComVelocity;
    instability.push_front(sqr((errorLeft.y() + errorRight.y()) * 0.5f));
  }
  else
  {
    errorLeft = errorRight = Vector3f::Zero();
    errorVelocity = Vector2f::Zero();
  }

  DEBUG_RESPONSE("module:WalkingEngine:optimize")
  {
    if(theGroundContactState.contact && currentMotionType == stepping)
      optimizeFitness.push_front(sqr((errorLeft.y() + errorRight.y()) * 0.5f));
    else
      optimizeFitness.push_front(sqr((20.f + 20.f) * 0.5f));
  }
}

void WalkingEngine::correctPendulumPlayer()
{
  if(balance && pendulumPlayer.phase.type != standPhase)
    applyCorrection(pendulumPlayer.phase, pendulumPlayer.nextPhase);
}

void WalkingEngine::updatePredictedPendulumPlayer()
{
  predictedPendulumPlayer = pendulumPlayer;

  // predict future pendulum state
  if(pendulumPlayer.phase.type != standPhase || pendulumPlayer.nextPhase.type != standPhase)
    predictedPendulumPlayer.seek(observerMeasurementDelay * 0.001f);
}

void WalkingEngine::generateTargetPosture()
{
  predictedPendulumPlayer.getPosture(targetPosture);
}

void WalkingEngine::generateJointRequest()
{
  // use head and arm joint angles
  jointRequest.angles[Joints::headYaw] = targetPosture.headJointAngles[0];
  jointRequest.angles[Joints::headPitch] = targetPosture.headJointAngles[1];
  jointRequest.angles[Joints::lShoulderPitch] = targetPosture.leftArmJointAngles[0];
  jointRequest.angles[Joints::lShoulderRoll] = targetPosture.leftArmJointAngles[1];
  jointRequest.angles[Joints::lElbowYaw] = targetPosture.leftArmJointAngles[2];
  jointRequest.angles[Joints::lElbowRoll] = targetPosture.leftArmJointAngles[3];
  jointRequest.angles[Joints::lWristYaw] = -90_deg;
  jointRequest.angles[Joints::lHand] = 0.f;
  jointRequest.angles[Joints::rShoulderPitch] = targetPosture.rightArmJointAngles[0];
  jointRequest.angles[Joints::rShoulderRoll] = targetPosture.rightArmJointAngles[1];
  jointRequest.angles[Joints::rElbowYaw] = targetPosture.rightArmJointAngles[2];
  jointRequest.angles[Joints::rElbowRoll] = targetPosture.rightArmJointAngles[3];
  jointRequest.angles[Joints::rWristYaw] = 90_deg;
  jointRequest.angles[Joints::rHand] = 0.f;

  // compute torso orientation
  bool transition = theMotionSelection.ratios[MotionRequest::specialAction] > 0 || theMotionSelection.ratios[MotionRequest::dmpKick] > 0 || theMotionSelection.ratios[MotionRequest::kick] > 0 || theMotionSelection.ratios[MotionRequest::getUp] > 0;
  float additionalBodyRotation = (targetPosture.rightOriginToCom.y() - targetPosture.rightOriginToFoot.translation.y() + targetPosture.leftOriginToCom.y() - targetPosture.leftOriginToFoot.translation.y()) * 0.5f;
  additionalBodyRotation *= 1.f / (22.5f - 50.f);
  additionalBodyRotation *= walkComBodyRotation;
  RotationMatrix bodyRotation = RotationMatrix::aroundX(additionalBodyRotation);
  bodyRotation *= standBodyRotation;

  float angularVelocityCorrection = 0.0f;
  if(!transition && theGroundContactState.contact && theInertialData.gyro.y() != SensorData::off)
  {
    // Buffer the relative rotations of the torso around its y-axis.
    const RotationMatrix relativeRotation = bodyRotation.inverse() * lastBodyRotationMatrix;
    const float relativeRotationY = atan2(relativeRotation(0, 2), relativeRotation(2, 2));
    relativeRotations.push_front(relativeRotationY);

    // Calculate the moving average of the gyro measurements.
    if(lastSmoothedGyroY == SensorData::off)
      lastSmoothedGyroY = theInertialData.gyro.y();
    const float smoothedGyro = theInertialData.gyro.y() * gyroSmoothing + lastSmoothedGyroY * (1.0f - gyroSmoothing);

    // Use the difference between the buffered and measured angular velocities of the torso to calculate a
    // relative y-axis angle offset to control the torso's angular velocity.
    const int frameDelay = static_cast<int>(observerMeasurementDelay / (theFrameInfo.cycleTime * 1000.0f));
    ASSERT(frameDelay < 10);
    if(relativeRotations.size() == 10)
    {
      const float angularVelocityY = (relativeRotations[frameDelay - 1] - relativeRotations[frameDelay]) / theFrameInfo.cycleTime;
      PLOT("module:WalkingEngine:torsoAngularVelocityY", toDegrees(angularVelocityY));
      PLOT("module:WalkingEngine:gyroY", toDegrees(theInertialData.gyro.y()));
      PLOT("module:WalkingEngine:smoothedGyroY", toDegrees(smoothedGyro));
      const float errorY = angularVelocityY - smoothedGyro;
      const float approxDerivative = (errorY - lastGyroErrorY) / theFrameInfo.cycleTime;
      angularVelocityCorrection = gyroStateGain * errorY + gyroDerivativeGain * approxDerivative;
      PLOT("module:WalkingEngine:torsoAngularVelocityCorrectionY", toDegrees(angularVelocityCorrection));
    }
    lastSmoothedGyroY = smoothedGyro;
  }
  else
  {
    lastSmoothedGyroY = SensorData::off;
    relativeRotations.clear();
  }
  lastBodyRotationMatrix = bodyRotation;

  // compute foot position relative to the center of mass
  const Pose3f comToLeftOrigin = Pose3f(bodyRotation, targetPosture.leftOriginToCom).inverse(); // TODO: optimize this by calculating the inverted left/rightOriginToCom pose directly
  const Pose3f comToRightOrigin = Pose3f(bodyRotation, targetPosture.rightOriginToCom).inverse();
  const Pose3f comToLeftAnkle = Pose3f(comToLeftOrigin).conc(targetPosture.leftOriginToFoot).translate(0.f, 0.f, theRobotDimensions.footHeight);
  const Pose3f comToRightAnkle = Pose3f(comToRightOrigin).conc(targetPosture.rightOriginToFoot).translate(0.f, 0.f, theRobotDimensions.footHeight);

  // try to guess bodyToCom
  const Vector3f averageComToAnkle = (comToLeftAnkle.translation + comToRightAnkle.translation) * 0.5f;
  Vector3f bodyToComOffset = lastAverageComToAnkle != Vector3f::Zero() ? Vector3f((averageComToAnkle - lastAverageComToAnkle) * 0.4f) : Vector3f::Zero();
  lastAverageComToAnkle = averageComToAnkle;
  bodyToCom += bodyToComOffset;

  RobotModel robotModel;
  Vector3f offsetFootHeight(0.f, 0.f, -theRobotDimensions.footHeight);
  // find bodyToCom
  for(int i = 0; ; ++i)
  {
    Pose3f bodyToLeftAnkle = Pose3f(bodyToCom).conc(comToLeftAnkle).translate(offsetFootHeight);
    Pose3f bodyToRightAnkle = Pose3f(bodyToCom).conc(comToRightAnkle).translate(offsetFootHeight);
    bool reachable = InverseKinematic::calcLegJoints(bodyToLeftAnkle, bodyToRightAnkle, theStandBodyRotation.bodyRotation, jointRequest, theRobotDimensions, 0.5f);
    robotModel.setJointData(jointRequest, theRobotDimensions, theMassCalibration); // TODO: improve this by not calculating the whole limb/mass model in each iteration

    Vector3f delta = (robotModel.centerOfMass - bodyToCom) * 1.3f;
    bodyToCom += delta;
    if(i >= 7 || (abs(delta.x()) < 0.05 && abs(delta.y()) < 0.05 && abs(delta.z()) < 0.05))
    {
      bodyToCom = robotModel.centerOfMass;
      if(!reachable)
      {
        ANNOTATION("WalkingEngine", "bodyToCom based on unreachable foot pose");
        OUTPUT_WARNING("WalkingEngine: bodyToCom based on unreachable foot pose");
      }
      break;
    }
  }

  // Correct the torso's angular velocity.
  Pose3f torsoInLeftFoot = robotModel.limbs[Limbs::footLeft].inverse();
  Pose3f torsoInRightFoot = robotModel.limbs[Limbs::footRight].inverse();
  torsoInLeftFoot.rotateY(angularVelocityCorrection);
  torsoInRightFoot.rotateY(angularVelocityCorrection);
  const Pose3f leftFootInTorso = torsoInLeftFoot.inverse().translate(offsetFootHeight);
  const Pose3f rightFootInTorso = torsoInRightFoot.inverse().translate(offsetFootHeight);
  if(!InverseKinematic::calcLegJoints(leftFootInTorso, rightFootInTorso, theStandBodyRotation.bodyRotation, jointRequest, theRobotDimensions, 0.5f))
  {
    ANNOTATION("WalkingEngine", "At least one foot pose unreachable");
    OUTPUT_WARNING("WalkingEngine: At least one foot pose unreachable");
  }

  // turn head joints off?
  if(theHeadJointRequest.pan == JointAngles::off)
    jointRequest.angles[Joints::headYaw] = JointAngles::off;
  if(theHeadJointRequest.tilt == JointAngles::off)
    jointRequest.angles[Joints::headPitch] = JointAngles::off;

  // set stiffness
  jointRequest.stiffnessData.stiffnesses[Joints::lAnklePitch] = theDamageConfigurationBody.weakLeftLeg ? 100 : standStiffnessAnklePitch;
  jointRequest.stiffnessData.stiffnesses[Joints::lAnkleRoll] = standStiffnessAnkleRoll;
  jointRequest.stiffnessData.stiffnesses[Joints::rAnklePitch] = theDamageConfigurationBody.weakRightLeg ? 100 : standStiffnessAnklePitch;
  jointRequest.stiffnessData.stiffnesses[Joints::rAnkleRoll] = standStiffnessAnkleRoll;


  /*
  * Making sure that the walkingengine-output arm-angles are the angles the walkingengine would normaly set, but
  * letting her to calculate on the real (motion combinator) angles
  */
  //left arm
  if(theArmMotionSelection.armRatios[ArmMotionRequest::none] < 1.f)
  {
    jointRequest.angles[Joints::lShoulderPitch] = targetPosture.walkingArms.leftArmJointAngles[0];
    jointRequest.angles[Joints::lShoulderRoll] = targetPosture.walkingArms.leftArmJointAngles[1];
    jointRequest.angles[Joints::lElbowYaw] = targetPosture.walkingArms.leftArmJointAngles[2];
    jointRequest.angles[Joints::lElbowRoll] = targetPosture.walkingArms.leftArmJointAngles[3];
    jointRequest.angles[Joints::lWristYaw] = -90_deg;
    jointRequest.angles[Joints::lHand] = 0.f;
  }
  //right arm
  if(theArmMotionSelection.armRatios[theArmMotionSelection.rightArmRatiosOffset + ArmMotionRequest::none] < 1.f)
  {
    jointRequest.angles[Joints::rShoulderPitch] = targetPosture.walkingArms.rightArmJointAngles[0];
    jointRequest.angles[Joints::rShoulderRoll] = targetPosture.walkingArms.rightArmJointAngles[1];
    jointRequest.angles[Joints::rElbowYaw] = targetPosture.walkingArms.rightArmJointAngles[2];
    jointRequest.angles[Joints::rElbowRoll] = targetPosture.walkingArms.rightArmJointAngles[3];
  }
}

void WalkingEngine::generateOutput(WalkingEngineOutput& walkingEngineOutput)
{
  if(pendulumPlayer.phase.type != standPhase)
  {
    const float stepDuration = (pendulumPlayer.phase.td - pendulumPlayer.nextPhase.tu) * 2.f;
    walkingEngineOutput.speed.translation = Vector2f(pendulumPlayer.phase.s.translation.x() + pendulumPlayer.nextPhase.s.translation.x(), pendulumPlayer.phase.s.translation.y() + pendulumPlayer.nextPhase.s.translation.y()) / stepDuration;
    walkingEngineOutput.speed.rotation = (pendulumPlayer.phase.s.rotation + pendulumPlayer.nextPhase.s.rotation) / stepDuration;
  }
  else
    walkingEngineOutput.speed = Pose2f();
  walkingEngineOutput.odometryOffset = odometryOffset;

  walkingEngineOutput.upcomingOdometryOffset = upcomingOdometryOffset;
  walkingEngineOutput.upcomingOdometryOffsetValid = true;
  walkingEngineOutput.isLeavingPossible = currentMotionType == stand;
  if(pendulumPlayer.phase.type == standPhase)
    walkingEngineOutput.positionInWalkCycle = 0.f;
  else
  {
    const float duration = pendulumPlayer.phase.td - pendulumPlayer.phase.tu;
    walkingEngineOutput.positionInWalkCycle = 0.5f * (-pendulumPlayer.phase.tu / duration) + (pendulumPlayer.phase.type == leftSupportPhase ? 0.5f : 0.f);
  }

  walkingEngineOutput.standing = currentMotionType == stand;
  walkingEngineOutput.instability = 0.f;
  walkingEngineOutput.executedWalk = theMotionRequest.walkRequest;
  walkingEngineOutput.executedWalk.kickType = predictedPendulumPlayer.phase.kickType; //kickPlayer.isActive() ? kickPlayer.getType() : WalkRequest::none;
  (JointRequest&)walkingEngineOutput = jointRequest;

  walkingEngineOutput.walkPhase = static_cast<WalkingEngineOutput::PhaseType>(pendulumPlayer.phase.type);
}

void WalkingEngine::generateDummyOutput(WalkingEngineOutput& walkingEngineOutput)
{
  walkingEngineOutput.standing = false;
  walkingEngineOutput.speed = Pose2f();
  walkingEngineOutput.odometryOffset = Pose2f();
  walkingEngineOutput.upcomingOdometryOffset = Pose2f();
  walkingEngineOutput.upcomingOdometryOffsetValid = true;
  walkingEngineOutput.isLeavingPossible = true;
  walkingEngineOutput.positionInWalkCycle = 0.f;
  walkingEngineOutput.instability = 0.f;
  walkingEngineOutput.executedWalk = WalkRequest();
  walkingEngineOutput.walkPhase = WalkingEngineOutput::PhaseType::standPhase;
  // leaving joint data untouched
}

void WalkingEngine::generateFirstPendulumPhase(WalkingEngine::PendulumPhase& phase)
{
  phase.type = standPhase;
  phase.k = walkK;
  phase.td = observerMeasurementDelay * (0.001f * 0.5f);
  phase.tu = -phase.td;
  phase.l = Vector3f::Zero();
  phase.cl = Vector3f::Zero();
  phase.s = StepSize();
  phase.rOpt = phase.rRef = phase.r = Vector2f(walkRef.x(), 0.f);
  phase.x0 = Vector2f::Zero();
  phase.xv0 = Vector2f::Zero();
  phase.toStand = false;
  phase.fromStand = false;
  phase.kickType = WalkRequest::none;
  phase.id = phaseBuffer.empty() ? 1 : (phaseBuffer.front().id + 1);

  phaseBuffer.push_front(phase);
}

void WalkingEngine::generateNextPendulumPhase(const WalkingEngine::PendulumPhase& phase, WalkingEngine::PendulumPhase& nextPhase)
{
  // phase may have already been generated, if so do not generate it again
  unsigned int nextPhaseId = phase.id + 1;
  ASSERT(!phaseBuffer.empty());
  if(nextPhaseId <= phaseBuffer.front().id)
  {
    for(const PendulumPhase& phase : phaseBuffer)
      if(phase.id == nextPhaseId)
      {
        nextPhase = phase;
        return;
      }
    ASSERT(false);
  }

  // get next phase type
  if(requestedMotionType == stepping && phase.type == standPhase)
  {
    nextPhase.type = ((theMotionRequest.walkRequest.mode == WalkRequest::targetMode ? requestedWalkTarget.translation.y() : theMotionRequest.walkRequest.speed.translation.y()) > 0.f) ? rightSupportPhase : leftSupportPhase;
    // TODO: check sign
  }
  else if((requestedMotionType == stepping && !phase.toStand) || (phase.type != standPhase && requestedMotionType == stand && !phase.toStand))
  {
    ASSERT(phase.type == leftSupportPhase || phase.type == rightSupportPhase);
    nextPhase.type = phase.type == leftSupportPhase ? rightSupportPhase : leftSupportPhase;
  }
  else
    nextPhase.type = standPhase;

  // stand?
  if(nextPhase.type == standPhase)
  {
    ASSERT(phase.toStand || phase.type == standPhase);
    generateFirstPendulumPhase(nextPhase);
    ASSERT(nextPhase.type == standPhase);
    ASSERT(nextPhase.id == nextPhaseId);
    ASSERT(phaseBuffer.front().id == nextPhaseId);
    return;
  }

  // leftSupportPhase or rightSupportPhase
  nextPhase.id = nextPhaseId;
  const float sign = nextPhase.type == leftSupportPhase ? 1.f : -1.f;
  nextPhase.r = Vector2f(walkRef.x(), walkRef.y() * sign);
  nextPhase.k = walkK;
  nextPhase.toStand = requestedMotionType == stand && !phase.fromStand;
  nextPhase.fromStand = phase.type == standPhase;
  nextPhase.kickType = WalkRequest::none;
  nextPhase.l = Vector3f(walkLiftOffset.x(), walkLiftOffset.y() * sign, walkLiftOffset.z());
  nextPhase.cl = Vector3f(walkComLiftOffset.x(), walkComLiftOffset.y() * sign, walkComLiftOffset.z());

  if(nextPhase.toStand && (abs(phase.s.translation.x()) < walkStepSizeXPlanningLimit.max * 0.5f || theFallDownState.state != FallDownState::upright))
  {
    nextPhase.td = 0;
    nextPhase.tu = walkPhaseDuration * -0.5f;
    nextPhase.s = StepSize();
    nextPhase.x0.x() = 0;
    nextPhase.x0.y() = -nextPhase.r.y();
    nextPhase.xv0 = Vector2f::Zero();
  }
  else
  {
    nextPhase.toStand = false;
    nextPhase.td = walkPhaseDuration * 0.5f;
    nextPhase.tu = walkPhaseDuration * -0.5f;

    float kickPhaseDuration = 0.f;
    if(nextPhase.fromStand || phase.fromStand)
    {
      nextPhase.l = Vector3f::Zero();
      nextPhase.s = StepSize();
    }
    else if(phase.kickType != WalkRequest::none)
    {
      nextPhase.s = StepSize();

      float additionRotation;
      Vector3f additionalTranslation;
      kicks.getKickStepSize(phase.kickType, additionRotation, additionalTranslation);
      nextPhase.s.rotation += additionRotation;
      nextPhase.s.translation += additionalTranslation;
    }
    else if(theMotionRequest.walkRequest.kickType != WalkRequest::none && kicks.isKickMirrored(theMotionRequest.walkRequest.kickType) == (nextPhase.type == leftSupportPhase))
    {
      nextPhase.kickType = theMotionRequest.walkRequest.kickType;
      kickPhaseDuration = kicks.getKickStepDuration(nextPhase.kickType) * 0.5f;
      nextPhase.s = StepSize();

      float additionRotation;
      Vector3f additionalTranslation;
      kicks.getKickPreStepSize(nextPhase.kickType, additionRotation, additionalTranslation);
      nextPhase.r.x() += kicks.getKickRefX(nextPhase.kickType, 0.f);
      nextPhase.s.rotation += additionRotation;
      nextPhase.s.translation += additionalTranslation;
    }
    else
    {
      generateNextStepSize(nextPhase.type, nextPhase.s);
    }

    nextPhase.r.x() += abs(nextPhase.s.translation.x()) * (walkRefAtFullSpeedX.x() - walkRef.x()) / (speedMax.translation.x() * 0.5f);
    float rYSpeedFix = (abs(nextPhase.s.translation.x()) * (walkRefAtFullSpeedX.y() - walkRef.y()) / (speedMax.translation.x() * 0.5f));
    nextPhase.r.y() += sign * rYSpeedFix;

    nextPhase.l.x() += abs(nextPhase.s.translation.x()) * (walkLiftOffsetAtFullSpeedX.x() - walkLiftOffset.x()) / (speedMax.translation.x() * 0.5f);
    nextPhase.l.y() += sign * (abs(nextPhase.s.translation.x()) * (walkLiftOffsetAtFullSpeedX.y() - walkLiftOffset.y()) / (speedMax.translation.x() * 0.5f));
    nextPhase.l.z() += abs(nextPhase.s.translation.x()) * (walkLiftOffsetAtFullSpeedX.z() - walkLiftOffset.z()) / (speedMax.translation.x() * 0.5f);

    nextPhase.l.x() += abs(nextPhase.s.translation.y()) * (walkLiftOffsetAtFullSpeedY.x() - walkLiftOffset.x()) / speedMax.translation.y();
    nextPhase.l.y() += sign * abs(nextPhase.s.translation.y()) * (walkLiftOffsetAtFullSpeedY.y() - walkLiftOffset.y()) / speedMax.translation.y();
    nextPhase.l.z() += abs(nextPhase.s.translation.y()) * (walkLiftOffsetAtFullSpeedY.z() - walkLiftOffset.z()) / speedMax.translation.y();

    if(nextPhase.l.z() > 0.f)
      nextPhase.lRotation = Vector3f(
                    walkLiftRotation.x() * sign * fabs(nextPhase.s.translation.y()) / speedMax.translation.y(),
                    nextPhase.s.translation.x() > 0.f ? (walkLiftRotation.y() * nextPhase.s.translation.x() / (speedMax.translation.x() * 0.5f)) : 0,
                    walkLiftRotation.z() * sign);

    const float walkPhaseDurationX = walkPhaseDuration + abs(nextPhase.s.translation.x()) * (walkPhaseDurationAtFullSpeedX - walkPhaseDuration) / (speedMax.translation.x() * 0.5f);
    const float walkPhaseDurationY = walkPhaseDurationX + abs(nextPhase.s.translation.y()) * (walkPhaseDurationAtFullSpeedY - walkPhaseDuration) / speedMax.translation.y();
    computeNextPendulumParametersY(nextPhase, walkPhaseDurationX, kickPhaseDuration != 0.f ? kickPhaseDuration : walkPhaseDurationY);
    computeNextPendulumParametersX(nextPhase);
  }

  nextPhase.rOpt = nextPhase.rRef = nextPhase.r;
  phaseBuffer.push_front(nextPhase);
}

void WalkingEngine::updatePendulumPhase(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const
{
  if(phase.type == standPhase)
  {
    ASSERT(nextPhase.type == standPhase || nextPhase.fromStand);
    ASSERT(fabs(phase.xv0.x()) < 0.1f && fabs(phase.xv0.y()) < 0.1f);
    ASSERT(nextPhase.s.translation == Vector3f::Zero());
    return;
  }

#ifndef NDEBUG
  Vector2f px = phase.r + phase.x0;
  Vector2f pxv = phase.xv0;
#endif

  updatePendulumParametersY(phase, nextPhase);
  updatePendulumParametersX(phase, nextPhase, init);

  ASSERT(nextPhase.tu < 0.f);
  ASSERT(nextPhase.x0.x() == 0.f || nextPhase.toStand);
  ASSERT(nextPhase.xv0.y() == 0.f);

  ASSERT(fabs(phase.r.y() + phase.x0.y() * cosh(phase.k.y() * phase.td) + phase.xv0.y() * sinh(phase.k.y() * phase.td) / phase.k.y() -
    (nextPhase.s.translation.y() + nextPhase.r.y() + nextPhase.x0.y() * cosh(nextPhase.k.y() * nextPhase.tu))) < (phase.td > 0.8f ? 1.f : 0.1f));
  ASSERT(fabs(phase.x0.y() * phase.k.y() * sinh(phase.k.y() * phase.td) + phase.xv0.y() * cosh(phase.k.y() * phase.td) -
    nextPhase.x0.y() * nextPhase.k.y() * sinh(nextPhase.k.y() * nextPhase.tu)) < (phase.td > 0.8f ? 1.f : 0.1f));

  ASSERT(fabs(phase.r.x() + phase.x0.x() - px.x()) < 0.1f || phase.toStand);
  ASSERT(fabs(phase.r.y() + phase.x0.y() - px.y()) < 0.1f || phase.toStand);
  ASSERT(fabs(phase.xv0.x() - pxv.x()) < 0.1f || phase.toStand);
  ASSERT(fabs(phase.xv0.y() - pxv.y()) < 0.1f || phase.toStand);
}

void WalkingEngine::computeNextPendulumParametersY(PendulumPhase& nextPhase, float walkPhaseDurationX, float walkPhaseDurationY) const
{
  // compute tu and td using walkPhaseDurationX
  nextPhase.td = walkPhaseDurationX * 0.5f;
  nextPhase.tu = -nextPhase.td;

  // compute x0 using walkPhaseDurationY
  const float td = walkPhaseDurationY * 0.5f;
  const float r = nextPhase.r.y();
  const float k = nextPhase.k.y();
  ASSERT(cosh(k * td) != 0.f);
  const float x0 = -r / cosh(k * td);
  nextPhase.x0.y() = x0;
  nextPhase.xv0.y() = 0.f;
  ASSERT(fabs(r + x0 * cosh(k * td)) < 0.1f);
}

void WalkingEngine::computeNextPendulumParametersX(PendulumPhase& nextPhase) const
{
  const float td = nextPhase.td;
  const float ns = nextPhase.s.translation.x();
  const float k = nextPhase.k.x();
  ASSERT(sinh(k * td) != 0.f);
  const float xv0 = ns * 0.5f * k / sinh(k * td);
  nextPhase.x0.x() = 0.f;
  nextPhase.xv0.x() = xv0;
  ASSERT(fabs((xv0 * sinh(k * td) / k) - (ns * 0.5f)) < 0.1f);
}

void WalkingEngine::generateNextStepSize(PhaseType nextSupportLeg, WalkingEngine::StepSize& nextStepSize)
{
  ASSERT(nextSupportLeg == leftSupportPhase || nextSupportLeg == rightSupportPhase);

  const Pose2f speedMaxMin(0.2f, 20.f, 0.f);

  if(requestedMotionType != stepping)
  {
    nextStepSize = StepSize();
    return;
  }

  // get requested walk target and speed
  Pose2f walkTarget = requestedWalkTarget;
  Pose2f requestedSpeed = theMotionRequest.walkRequest.speed;
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode) // remove upcoming odometry offset
  {
    walkTarget -= upcomingOdometryOffset;

    requestedSpeed = Pose2f(walkTarget.rotation * 2.f / odometryScale.rotation, walkTarget.translation.array() * 2.f / odometryScale.translation.array());

    if(theMotionRequest.walkRequest.speed.translation.x() == 0.f)
      requestedSpeed.translation.x() = 0.f;
    if(theMotionRequest.walkRequest.speed.translation.y() == 0.f)
      requestedSpeed.translation.y() = 0.f;
    if(theMotionRequest.walkRequest.speed.rotation == 0.f)
      requestedSpeed.rotation = 0.f;
  }
  else if(theMotionRequest.walkRequest.mode == WalkRequest::percentageSpeedMode)
  {
    requestedSpeed.rotation *= speedMax.rotation;
    requestedSpeed.translation.x() *= (theMotionRequest.walkRequest.speed.translation.x() >= 0.f ? speedMax.translation.x() : speedMaxBackwards);
    requestedSpeed.translation.y() *= speedMax.translation.y();
  }

  // reduce speed for target walks near the target to handle limited deceleration
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
  {
    float maxSpeedForTargetX = sqrt(2.f * abs(requestedSpeed.translation.x()) * speedMaxChange.translation.x());
    if(abs(requestedSpeed.translation.x()) > maxSpeedForTargetX)
      requestedSpeed.translation.x() = requestedSpeed.translation.x() >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;

    float maxSpeedForTargetY = sqrt(2.f * abs(requestedSpeed.translation.y()) * speedMaxChange.translation.y());
    if(abs(requestedSpeed.translation.y()) > maxSpeedForTargetY)
      requestedSpeed.translation.y() = requestedSpeed.translation.y() >= 0.f ? maxSpeedForTargetY : -maxSpeedForTargetY;

    float maxSpeedForTargetR = sqrt(2.f * abs(requestedSpeed.rotation) * speedMaxChange.rotation);
    if(abs(requestedSpeed.rotation) > maxSpeedForTargetR)
      requestedSpeed.rotation = requestedSpeed.rotation >= 0.f ? maxSpeedForTargetR : -maxSpeedForTargetR;
  }

  // normalize x-y-speed
  const Pose2f maxSpeed(speedMax.rotation, requestedSpeed.translation.x() < 0.f ? speedMaxBackwards : speedMax.translation.x(), speedMax.translation.y());
  {
    Vector2f tmpSpeed(
      requestedSpeed.translation.x() / (speedMaxMin.translation.x() + maxSpeed.translation.x()),
      requestedSpeed.translation.y() / (speedMaxMin.translation.y() + maxSpeed.translation.y()));
    const float tmpSpeedAbs = tmpSpeed.norm();
    if(tmpSpeedAbs > 1.f)
    {
      tmpSpeed /= tmpSpeedAbs;
      tmpSpeed.x() *= speedMaxMin.translation.x() + maxSpeed.translation.x();
      tmpSpeed.y() *= speedMaxMin.translation.y() + maxSpeed.translation.y();
      requestedSpeed.translation.x() = Rangef(-maxSpeed.translation.x(), maxSpeed.translation.x()).limit(tmpSpeed.x());
      requestedSpeed.translation.y() = Rangef(-maxSpeed.translation.y(), maxSpeed.translation.y()).limit(tmpSpeed.y());
    }
  }

  // normalize speed (including rotation)
  {
    Vector3f tmpSpeed(
      requestedSpeed.translation.x() / (speedMaxMin.translation.x() + maxSpeed.translation.x()),
      requestedSpeed.translation.y() / (speedMaxMin.translation.y() + maxSpeed.translation.y()),
      requestedSpeed.rotation / (speedMaxMin.rotation + maxSpeed.rotation));
    const float tmpSpeedAbs = tmpSpeed.norm();
    if(tmpSpeedAbs > 1.f)
    {
      tmpSpeed /= tmpSpeedAbs;
      tmpSpeed.x() *= speedMaxMin.translation.x() + maxSpeed.translation.x();
      tmpSpeed.y() *= speedMaxMin.translation.y() + maxSpeed.translation.y();
      tmpSpeed.z() *= speedMaxMin.rotation + maxSpeed.rotation;
      requestedSpeed.translation.x() = Rangef(-maxSpeed.translation.x(), maxSpeed.translation.x()).limit(tmpSpeed.x());
      requestedSpeed.translation.y() = Rangef(-maxSpeed.translation.y(), maxSpeed.translation.y()).limit(tmpSpeed.y());
      requestedSpeed.rotation = Rangef(-maxSpeed.rotation, maxSpeed.rotation).limit(tmpSpeed.z());
    }
  }

  // max rotation speed change clipping (rotation-only)
  requestedSpeed.rotation = Rangef(lastSelectedSpeed.rotation - speedMaxChange.rotation, lastSelectedSpeed.rotation + speedMaxChange.rotation).limit(requestedSpeed.rotation);
  lastSelectedSpeed = requestedSpeed;

  // clip requested walk speed to a target walk speed limit
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
  {
    requestedSpeed.translation.x() = Rangef(-speedMax.translation.x() * theMotionRequest.walkRequest.speed.translation.x(), speedMax.translation.x() * theMotionRequest.walkRequest.speed.translation.x()).limit(requestedSpeed.translation.x());
    requestedSpeed.translation.y() = Rangef(-speedMax.translation.y() * theMotionRequest.walkRequest.speed.translation.y(), speedMax.translation.y() * theMotionRequest.walkRequest.speed.translation.y()).limit(requestedSpeed.translation.y());
    requestedSpeed.rotation = Rangef(-speedMax.rotation * theMotionRequest.walkRequest.speed.rotation, speedMax.rotation * theMotionRequest.walkRequest.speed.rotation).limit(requestedSpeed.rotation);
  }

  // generate step size from requested walk speed
  nextStepSize = StepSize(requestedSpeed.rotation, requestedSpeed.translation.x() * 0.5f, requestedSpeed.translation.y());

  // just move the outer foot, when walking sidewards or when rotating
  if((nextStepSize.translation.y() < 0.f && nextSupportLeg == leftSupportPhase) || (nextStepSize.translation.y() > 0.f && nextSupportLeg != leftSupportPhase))
    nextStepSize.translation.y() = 0.f;
  if((nextStepSize.rotation < 0.f && nextSupportLeg == leftSupportPhase) || (nextStepSize.rotation > 0.f && nextSupportLeg != leftSupportPhase))
    nextStepSize.rotation = 0.f;

  // clip to walk target
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
  {
    if((nextStepSize.translation.x() > 0.f && walkTarget.translation.x() > 0.f && nextStepSize.translation.x() * odometryScale.translation.x() > walkTarget.translation.x()) || (nextStepSize.translation.x() < 0.f && walkTarget.translation.x() < 0.f && nextStepSize.translation.x() * odometryScale.translation.x() < walkTarget.translation.x()))
      nextStepSize.translation.x() = walkTarget.translation.x() / odometryScale.translation.x();
    if((nextStepSize.translation.y() > 0.f && walkTarget.translation.y() > 0.f && nextStepSize.translation.y() * odometryScale.translation.y() > walkTarget.translation.y()) || (nextStepSize.translation.y() < 0.f && walkTarget.translation.y() < 0.f && nextStepSize.translation.y() * odometryScale.translation.y() < walkTarget.translation.y()))
      nextStepSize.translation.y() = walkTarget.translation.y() / odometryScale.translation.y();
    if((nextStepSize.rotation > 0.f && walkTarget.rotation > 0.f && nextStepSize.rotation * odometryScale.rotation > walkTarget.rotation) || (nextStepSize.rotation < 0.f && walkTarget.rotation < 0.f && nextStepSize.rotation * odometryScale.rotation < walkTarget.rotation))
      nextStepSize.rotation = walkTarget.rotation / odometryScale.rotation;
  }
}

void WalkingEngine::computeOdometryOffset()
{
  {
    Pose3f footLeft = theRobotModel.limbs[Limbs::footLeft].translated(0.f, 0.f, -theRobotDimensions.footHeight);
    Pose3f footRight = theRobotModel.limbs[Limbs::footRight].translated(0.f, 0.f, -theRobotDimensions.footHeight);
    Vector3f odometryOrigin = (footLeft.translation + footRight.translation) * 0.5f;
    if(lastOdometryOrigin.z() != 0.f)
    {
      Pose3f& footSupport = pendulumPlayer.phase.type == leftSupportPhase ? footLeft : footRight;
      Pose3f& lastFootSupport = pendulumPlayer.phase.type == leftSupportPhase ? lastFootLeft : lastFootRight;
      Pose3f odometryOffset3DinP = (Pose3f(-odometryOrigin).conc(footSupport).conc(lastFootSupport.inverse()).conc(lastOdometryOrigin)).inverse();
      Pose3f odometryOffset3D = Pose3f(theTorsoMatrix).conc(odometryOffset3DinP).conc(theTorsoMatrix.inverse());
      odometryOffset.rotation = odometryOffset3D.rotation.getZAngle() * odometryScale.rotation;
      odometryOffset.translation.x() = odometryOffset3D.translation.x() * odometryScale.translation.x();
      odometryOffset.translation.y() = odometryOffset3D.translation.y() * odometryScale.translation.y();
    }
    else
      odometryOffset = Pose2f();
    lastFootLeft = footLeft;
    lastFootRight = footRight;
    lastOdometryOrigin = odometryOrigin;
  }

  // compute upcoming odometry offset
  upcomingOdometryOffset = Pose2f((pendulumPlayer.nextPhase.s.rotation - observedStepOffset.rotation) * 0.5f * odometryScale.rotation,
                                  (pendulumPlayer.nextPhase.s.translation.head<2>() - observedStepOffset.translation).array() * 0.5f * odometryScale.translation.array());
  upcomingOdometryOffset += Pose2f(pendulumPlayer.nextPhase.s.rotation * 0.5f * odometryScale.rotation,
                                   pendulumPlayer.nextPhase.s.translation.head<2>().array() * 0.5f * odometryScale.translation.array());
  if(predictedPendulumPlayer.nextPhase.id > pendulumPlayer.nextPhase.id)
  {
    Pose2f upcomingOdometryOffsetNextPhase(predictedPendulumPlayer.nextPhase.s.rotation * odometryScale.rotation,
                                           predictedPendulumPlayer.nextPhase.s.translation.head<2>().array() * odometryScale.translation.array());
    upcomingOdometryOffset += upcomingOdometryOffsetNextPhase;
  }

  // remove odometry offset from requested walk target
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
    requestedWalkTarget -= odometryOffset;
}

void WalkingEngine::applyCorrection(PendulumPhase& phase, PendulumPhase& nextPhase)
{
  Vector3f* directError, *indirectError;
  if(phase.type == leftSupportPhase)
  {
    directError = &errorLeft;
    indirectError = &errorRight;
  }
  else
  {
    ASSERT(phase.type == rightSupportPhase);
    directError = &errorRight;
    indirectError = &errorLeft;
  }

  if(phase.type != standPhase)
  {
    const Vector2f px = phase.r + phase.x0;
    const Vector2f xv = phase.xv0;
    const Vector2f xa(phase.x0.x() * phase.k.x() * phase.k.x(), phase.x0.y() * phase.k.y() * phase.k.y());

    measuredPx = Vector2f(px.x() + directError->x(), px.y() + directError->y());
    const Vector2f measuredPx = Vector2f(px.x() + directError->x() * balanceCom.x(), px.y() + directError->y() * balanceCom.y());
    Vector2f measuredXv = xv + (measuredPx - px) / theFrameInfo.cycleTime;
    const Vector2f measuredXa = xa + (measuredXv - xv) / theFrameInfo.cycleTime;

    const Vector2f measuredX(measuredXa.x() / (phase.k.x() * phase.k.x()), measuredXa.y() / (phase.k.y() * phase.k.y()));
    measuredR = measuredPx - measuredX;

    measuredXv = Vector2f(xv.x() + errorVelocity.x() * balanceComVelocity.x(), xv.y() + errorVelocity.y() * balanceComVelocity.y());

    const float rYSign = phase.type == leftSupportPhase ? 1.f : -1.f;
    Rangef rYLimit = Rangef(phase.rOpt.y() + walkRefYLimit.min * rYSign).add(phase.rOpt.y() + walkRefYLimit.max * rYSign);

    Vector2f newR = phase.r;

    // method #1: use measured ZMP as new ref
    //newR = measuredR; // ???
    //newR.y = rYLimit.limit(newR.y);

    // method #2: p-control
    newR.x() = phase.rRef.x() - (measuredR.x() - phase.rRef.x()) * balanceRef.x(); // ????
    newR.y() = phase.rRef.y() - (measuredR.y() - phase.rRef.y()) * balanceRef.y(); // ????
    newR.y() = rYLimit.limit(newR.y());

    Vector2f newX = measuredPx - newR;
    Vector2f newXv = measuredXv;

    phase.x0 = newX;
    phase.xv0 = newXv;
    phase.r = newR;
  }

  if(nextPhase.type != standPhase && !nextPhase.toStand)
  {
    nextPhase.r.x() = nextPhase.rOpt.x() + indirectError->x() * balanceNextRef.x();
    nextPhase.r.y() = nextPhase.rOpt.y() + indirectError->y() * balanceNextRef.y();

    Rangef nrXLimit(nextPhase.rOpt.x() + walkRefXLimit.min, nextPhase.rOpt.x() + walkRefXLimit.max);
    const float nrYSign = nextPhase.type == leftSupportPhase ? 1.f : -1.f;
    Rangef nrYLimit = Rangef(nextPhase.rOpt.y() + walkRefYLimit.min * nrYSign).add(nextPhase.rOpt.y() + walkRefYLimit.max * nrYSign);

    nextPhase.r.x() = nrXLimit.limit(nextPhase.r.x());
    nextPhase.r.y() = nrYLimit.limit(nextPhase.r.y());

    float nsxapprox = 2.f * nextPhase.xv0.x() * cosh(nextPhase.k.x() * nextPhase.tu);
    nsxapprox += indirectError->x() * balanceStepSize.x();
    nextPhase.xv0.x() = nsxapprox / (2.f * cosh(nextPhase.k.x() * nextPhase.tu));

    nextPhase.s.translation.y() += indirectError->y() * balanceStepSize.y();
  }

  repairPendulumParametersY(phase, nextPhase);
  updatePendulumPhase(phase, nextPhase, false);
}

void WalkingEngine::PendulumPlayer::seek(float deltaTime)
{
  for(;;)
  {
    float dt = deltaTime;
    if(dt > phase.td)
      dt = phase.td;
    deltaTime -= dt;

    phase.td -= dt;
    phase.tu -= dt;
    switch(phase.type)
    {
      case standPhase:
        ASSERT(fabs(phase.xv0.x()) < 0.1f);
        ASSERT(fabs(phase.xv0.y()) < 0.1f);
        ASSERT(fabs(phase.x0.x()) < 0.1f);
        ASSERT(fabs(phase.x0.y()) < 0.1f);
        break;

      case leftSupportPhase:
      case rightSupportPhase:
      { // y
        const float k = phase.k.y();
        const float kDt = k * dt;
        const float coshKDt = cosh(kDt);
        const float sinhKDt = sinh(kDt);
        const float x0 = phase.x0.y();
        const float xv0 = phase.xv0.y();
        phase.x0.y() = x0 * coshKDt + xv0 / k * sinhKDt;
        phase.xv0.y() = x0 * k * sinhKDt + xv0 * coshKDt;
      }

      { // x
        const float k = phase.k.x();
        const float kDt = k * dt;
        const float coshKDt = cosh(kDt);
        const float sinhKDt = sinh(kDt);
        const float x0 = phase.x0.x();
        const float xv0 = phase.xv0.x();
        phase.x0.x() = x0 * coshKDt + xv0 / k * sinhKDt;
        phase.xv0.x() = x0 * k * sinhKDt + xv0 * coshKDt;
      }

      if(kickPlayer.isActive() && deltaTime == 0)
        kickPlayer.seekTo(-phase.tu / (phase.td - phase.tu) * kickPlayer.getLength());

      break;

      default:
        ASSERT(false);
        break;
    }

    if(deltaTime == 0.f)
      break;
    if(phase.td > 0.f)
      continue;

    // phase transition
    kickPlayer.stop();
    Vector2f px = phase.r + phase.x0;
    Vector2f pxv = phase.xv0;
    phase = nextPhase;
    engine->generateNextPendulumPhase(phase, nextPhase);
    phase.x0 = (px - Vector2f(phase.s.translation.x(), phase.s.translation.y())) - phase.r;
    phase.xv0 = pxv;
    phase.td -= phase.tu;
    phase.tu = 0.f;
    engine->updatePendulumPhase(phase, nextPhase, true);
    if(phase.kickType != WalkRequest::none)
      kickPlayer.start(*engine->kicks.getKick(phase.kickType), engine->kicks.isKickMirrored(phase.kickType));
  }
}

void WalkingEngine::PendulumPlayer::getPosture(LegPosture& stance, float* leftArmAngle, float* rightArmAngle, Pose2f* stepOffset)
{
  const WalkingEngine& p = *engine;
  switch(phase.type)
  {
    case standPhase:
      ASSERT(fabs(phase.xv0.x()) < 0.1f && fabs(phase.xv0.y()) < 0.1f);
      stance.leftOriginToCom = stance.rightOriginToCom = Vector3f(phase.r.x() + phase.x0.x(), phase.r.y() + phase.x0.y(), p.standComPosition.z);
      stance.leftOriginToFoot = Pose3f(Vector3f(0.f, p.standComPosition.y, 0.f));
      stance.rightOriginToFoot = Pose3f(Vector3f(0.f, -p.standComPosition.y, 0.f));
      // TODO: nextPhase.s?

      if(stepOffset)
        *stepOffset = Pose2f();

      break;

    default:
    {
      const float ratio = -phase.tu / (phase.td - phase.tu);
      const float swingMoveFadeIn = ratio < p.walkMovePhase.start ? 0.f : ratio > p.walkMovePhase.start + p.walkMovePhase.duration ? 1.f : blend((ratio - p.walkMovePhase.start) / p.walkMovePhase.duration);
      const float swingMoveFadeOut = 1.f - swingMoveFadeIn;
      const float swingLift = ratio < p.walkLiftPhase.start || ratio > p.walkLiftPhase.start + p.walkLiftPhase.duration ? 0.f : blend((ratio - p.walkLiftPhase.start) / p.walkLiftPhase.duration * 2.f);

      Vector3f comLift;
      if(phase.fromStand || phase.toStand)
      {
        ASSERT(phase.fromStand != phase.toStand);
        if(phase.fromStand)
          comLift = nextPhase.cl * (1.f - blend(1.f + ratio));
        else
          comLift = phase.cl * (1.f - blend(ratio));
      }
      else
        comLift = (ratio <= 0.5f ? phase.cl : nextPhase.cl) * (1.f - blend(ratio * 2.f));

      switch(phase.type)
      {
        case leftSupportPhase:
        {
          stance.leftOriginToCom = Vector3f(phase.r.x() + phase.x0.x(), phase.r.y() + phase.x0.y(), p.standComPosition.z) - comLift;
          const Vector3f& leftOriginToRightOrigin = nextPhase.s.translation;
          stance.rightOriginToCom = stance.leftOriginToCom - leftOriginToRightOrigin;

          Vector3f rightStepOffsetRotation = nextPhase.lRotation * swingLift;
          rightStepOffsetRotation.z() += nextPhase.s.rotation * swingMoveFadeIn;
          const Vector3f leftStepOffsetRotation(0.f, swingLift * (phase.rRef.x() - phase.rOpt.x()) * p.walkSupportRotation, phase.s.rotation * swingMoveFadeOut);

          stance.leftOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(leftStepOffsetRotation), Vector3f(0.f, p.standComPosition.y, 0.f));
          stance.rightOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(rightStepOffsetRotation), Vector3f(0.f, -p.standComPosition.y, 0.f) + nextPhase.l * swingLift - (nextPhase.s.translation + phase.s.translation) * swingMoveFadeOut);

          if(leftArmAngle)
            *leftArmAngle = (nextPhase.s.translation.x() * swingMoveFadeIn - phase.s.translation.x() * swingMoveFadeOut) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
          if(rightArmAngle)
            *rightArmAngle = (phase.s.translation.x() * swingMoveFadeOut - nextPhase.s.translation.x() * swingMoveFadeIn) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
        }
        break;

        case rightSupportPhase:
        {
          stance.rightOriginToCom = Vector3f(phase.r.x() + phase.x0.x(), phase.r.y() + phase.x0.y(), p.standComPosition.z) - comLift;
          const Vector3f& rightOriginToLeftOrigin = nextPhase.s.translation;
          stance.leftOriginToCom = stance.rightOriginToCom - rightOriginToLeftOrigin;

          Vector3f leftStepOffsetRotation = nextPhase.lRotation * swingLift;
          leftStepOffsetRotation.z() += nextPhase.s.rotation * swingMoveFadeIn;
          const Vector3f rightStepOffsetRotation(0.f, swingLift * (phase.rRef.x() - phase.rOpt.x()) * p.walkSupportRotation, phase.s.rotation * swingMoveFadeOut);

          stance.leftOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(leftStepOffsetRotation), Vector3f(0.f, p.standComPosition.y, 0.f) + nextPhase.l * swingLift - (nextPhase.s.translation + phase.s.translation) * swingMoveFadeOut);
          stance.rightOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(rightStepOffsetRotation), Vector3f(0.f, -p.standComPosition.y, 0.f));

          if(rightArmAngle)
            *rightArmAngle = (nextPhase.s.translation.x() * swingMoveFadeIn - phase.s.translation.x() * swingMoveFadeOut) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
          if(leftArmAngle)
            *leftArmAngle = (phase.s.translation.x() * swingMoveFadeOut - nextPhase.s.translation.x() * swingMoveFadeIn) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
        }
        break;

        default:
          ASSERT(false);
          break;
      }

      if(stepOffset)
      {
        stepOffset->translation.x() = nextPhase.s.translation.x() * swingMoveFadeIn - phase.s.translation.x() * swingMoveFadeOut;
        stepOffset->translation.y() = nextPhase.s.translation.y() * swingMoveFadeIn - phase.s.translation.y() * swingMoveFadeOut;
        stepOffset->rotation = nextPhase.s.rotation * swingMoveFadeIn - phase.s.rotation * swingMoveFadeOut;
      }

      if(kickPlayer.isActive())
        kickPlayer.applyFoot(stance.leftOriginToFoot, stance.rightOriginToFoot);
    }
  }
}

void WalkingEngine::PendulumPlayer::getPosture(Posture& stance)
{
  const WalkingEngine& p = *engine;

  // head
  stance.headJointAngles[0] = engine->theHeadJointRequest.pan == JointAngles::off ? Angle(0.f) : engine->theHeadJointRequest.pan;
  stance.headJointAngles[1] = engine->theHeadJointRequest.tilt == JointAngles::off ? Angle(0.f) : engine->theHeadJointRequest.tilt;

  // legs
  float leftArmAngle = 0.f, rightArmAngle = 0.f;
  getPosture((LegPosture&)stance, &leftArmAngle, &rightArmAngle, 0);

  // arms
  float halfArmRotation = p.walkArmRotationAtFullSpeedX * 0.5f;

  // normal WalkingEngine arm movement
  stance.leftArmJointAngles[0] = stance.walkingArms.leftArmJointAngles[0] = pi_2 - p.standArmJointAngles.y() - leftArmAngle;
  stance.leftArmJointAngles[1] = stance.walkingArms.leftArmJointAngles[1] = p.standArmJointAngles.x();
  stance.leftArmJointAngles[2] = stance.walkingArms.leftArmJointAngles[2] = -pi_2;
  stance.leftArmJointAngles[3] = stance.walkingArms.leftArmJointAngles[3] = -p.standArmJointAngles.y() - leftArmAngle - halfArmRotation;

  stance.rightArmJointAngles[0] = stance.walkingArms.rightArmJointAngles[0] = pi_2 - p.standArmJointAngles.y() - rightArmAngle;
  stance.rightArmJointAngles[1] = stance.walkingArms.rightArmJointAngles[1] = -p.standArmJointAngles.x();
  stance.rightArmJointAngles[2] = stance.walkingArms.rightArmJointAngles[2] = pi_2;
  stance.rightArmJointAngles[3] = stance.walkingArms.rightArmJointAngles[3] = p.standArmJointAngles.y() + rightArmAngle + halfArmRotation;

  if(engine->theArmMotionSelection.armRatios[ArmMotionRequest::none] < 1.f)
  {
    for(int i = 0; i < 4; ++i)
      stance.leftArmJointAngles[i] = engine->theJointRequest.angles[Joints::lShoulderPitch + i];
  }

  if(engine->theArmMotionSelection.armRatios[engine->theArmMotionSelection.rightArmRatiosOffset + ArmMotionRequest::none] < 1.f)
  {
    for(int i = 0; i < 4; ++i)
      stance.rightArmJointAngles[i] = engine->theJointRequest.angles[Joints::rShoulderPitch + i];
  }

  // kick mutations
  if(kickPlayer.isActive())
    kickPlayer.applyHeadAndArms(stance.headJointAngles, stance.leftArmJointAngles, stance.rightArmJointAngles);
}

float WalkingEngine::blend(float r)
{
  return 0.5f - cos(r * pi) * 0.5f;
}

// equation (4.60):
// rx + x0x * cosh(k * td) + xv0x * sinh(k * td) / k = nsx + nrx                         + nxv0x * sinh(nk * ntu) / nk
// ry + x0y * cosh(k * td) + xv0y * sinh(k * td) / k = nsy + nry + nx0y * cosh(nk * ntu)

// equation (4.61):
// x0x * k * sinh(k * td) + xv0x * cosh(k * td) =                              nxv0x * cosh(nk * ntu)
// x0y * k * sinh(k * td) + xv0y * cosh(k * td) = nx0y * nk * sinh(nk * ntu)

void WalkingEngine::repairPendulumParametersY(PendulumPhase& phase, const PendulumPhase& nextPhase) const
{ // this function ensures that phase.r, phase.x0 and phase.xv0 can be used to solve equation (4.60) and (4.61)

  float sign = phase.type == leftSupportPhase ? 1.f : -1.f;
  float x = phase.x0.y();
  float xv = phase.xv0.y();
  const float k = phase.k.y();
  float r = phase.r.y();
  float px = r + x;

  // check #1: ensure pendulum pivot point is atleast 1mm away from pendulum deflection
  if(x * -sign < 1.f)
  {
    x = 1.f * -sign;
    r = px - x;
  }
  ASSERT(x * -sign >= 1.f);

  // check #2: ensure there is a value for t that solves:
  // x * k * sinh(k * t) + xv * cosh(k * t) = 0

  // x * k * sinh(k * t) + xv * cosh(k * t) = 0
  // x * k * sinh(k * t) / cosh(k * t) + xv  = 0
  // x * k * tanh(k * t) + xv  = 0
  // x * k * tanh(k * t) = -xv
  // tanh(k * t) = -xv / (x * k)
  // t = atanh(-xv / (x * k)) / k

  const float tanhKTl1 = -xv / (x * k);
  float x0 = 0.f;
  float t0 = 0.f;
  if(tanhKTl1 > -1.f && tanhKTl1 < 1.f)
  {
    const float kT = atanh(tanhKTl1);
    t0 = -kT / k;
    x0 = x * cosh(kT) + xv / k * sinh(kT);
  }
  if(x0 * -sign < 1.f)
  { // ensure pendulum pivot point is atleast 1mm away from pendulum peak
    x0 = 1.f * -sign;

    // r + x0 * cosh(k * t0) = px
    // x0 * k * sinh(k * t0) = pxv

    float kT = asinh(xv / (x0 * k));
    t0 = kT / k;
    r = px - x0 * cosh(kT);
    x = px - r;
  }
  ASSERT(x0 * -sign >= 1.f);
  ASSERT(fabs(r + x0 * cosh(k * t0) - px) < 0.1f);
  ASSERT(fabs(x0 * k * sinh(k * t0) - xv) < 0.1f);

  //check #4:
  if((r + x0 - (nextPhase.r.y() + nextPhase.x0.y())) * -sign > nextPhase.s.translation.y() * -sign - 1.f)
  {
    //r + x0  = nextPhase.s.translation.y + 1.f * sign + (nextPhase.r.y + nextPhase.x0.y);
    const float px0 = nextPhase.s.translation.y() + 1.f * sign + (nextPhase.r.y() + nextPhase.x0.y());

    // r + x0 = px0
    // r + x0 * cosh(k * t) = px
    // x0 * k * sinh(k * t) = pxv

    // We can't solve this system of equations for r and x0 since px might be equal px0.
    // So, we will keep r and change x and xv:

    x0 = px0 - r;
    if(x0 * -sign < 1.f)
    {
      x0 = 1.f * -sign;
      r = px0 - x0;
    }

    // r + x0 * cosh(k * t0) = px
    // x0 * k * sinh(k * t0) = pxv

    px = r + x0 * cosh(k * t0);
    xv = x0 * k * sinh(k * t0);
    x = px - r;
  }
  else
  {
    ASSERT(fabs(r + x - px) < 0.1f);
  }

  ASSERT(x * -sign >= 1.f);
  ASSERT(x0 * -sign >= 1.f);
  ASSERT(fabs(r + x0 * cosh(k * t0) - px) < 0.1f);
  ASSERT(fabs(x0 * k * sinh(k * t0) - xv) < 0.1f);
  ASSERT((r + x0 - (nextPhase.r.y() + nextPhase.x0.y())) * -sign < nextPhase.s.translation.y() * -sign);

  phase.r.y() = r;
  phase.x0.y() = x;
  phase.xv0.y() = xv;
}

void WalkingEngine::updatePendulumParametersY(PendulumPhase& phase, PendulumPhase& nextPhase) const
{
  if(phase.toStand)
  {
    ASSERT(nextPhase.x0.y() == -nextPhase.r.y());
    ASSERT(nextPhase.xv0.y() == 0.f);

    // find x, r and xv? in:
    // r + x = px
    // r + x * cosh(k * td) + xv * sinh(k * td) / k = npx
    // x * k * sinh(k * td) + xv * cosh(k * td) = npxv

    struct Data : public FunctionMinimizer
    {
      float px;
      float npx;
      float npxv;
      float k;
      float coshKTd;
      float sinhKTd;

      virtual float func(float r) const
      {
        const float x = px - r;
        const float xv = (npxv - x * k * sinhKTd) / coshKTd;
        return fabs(r + x * coshKTd + xv * sinhKTd / k - npx);
      }
    } d;

    d.px = phase.r.y() + phase.x0.y();
    d.npx = nextPhase.r.y() + nextPhase.x0.y();
    d.npxv = nextPhase.xv0.y();
    d.k = phase.k.y();
    d.coshKTd = cosh(d.k * phase.td);
    d.sinhKTd = sinh(d.k * phase.td);

    bool clipped;
    const float r = d.minimize(-1000000.f, 1000000.f, phase.r.y(), 0.1f, 0.05f, clipped, "updatePendulumParametersYtoStand");
    float x = d.px - r;
    float xv = (d.npxv - x * d.k * d.sinhKTd) / d.coshKTd;
    if(clipped)
    {
      // find x and xv in:
      // r + x * cosh(k * td) + xv * sinh(k * td) / k = npx
      // x * k * sinh(k * td) + xv * cosh(k * td) = npxv

      // x * cosh(k * td) + xv * sinh(k * td) / k = npx - r
      // x * k * sinh(k * td) + xv * cosh(k * td) = npxv

      // x * cosh(k * td) + xv * sinh(k * td) / k = npx - r
      // xv * cosh(k * td) = npxv - x * k * sinh(k * td)

      // x * cosh(k * td) * k + xv * sinh(k * td) = (npx - r) * k
      // xv = npxv / cosh(k * td) - x * k * sinh(k * td) / cosh(k * td)

      // x * cosh(k * td) * k + (npxv / cosh(k * td) - x * k * sinh(k * td) / cosh(k * td)) * sinh(k * td) = (npx - r) * k

      // x * cosh(k * td) * k + npxv / cosh(k * td) * sinh(k * td) - x * k * sinh^2(k * td) / cosh(k * td) = (npx - r) * k

      // x * cosh(k * td) * k - x * k * sinh^2(k * td) / cosh(k * td) = (npx - r) * k - npxv / cosh(k * td) * sinh(k * td)

      // x * k * (cosh(k * td) - sinh^2(k * td) / cosh(k * td)) = (npx - r) * k - npxv / cosh(k * td) * sinh(k * td)

      x = ((d.npx - r) * d.k - d.npxv / d.coshKTd * d.sinhKTd) / (d.k * (d.coshKTd - d.sinhKTd * d.sinhKTd / d.coshKTd));
      xv = (d.npxv - x * d.k * d.sinhKTd) / d.coshKTd;
    }

    phase.r.y() = r;
    phase.x0.y() = x;
    phase.xv0.y() = xv;
    return;
  }

  // step #1: find t in:
  // x * k * sinh(k * t0) + xv * cosh(k * t0) = 0

  // x * k * sinh(k * t0) + xv * cosh(k * t0) = 0
  // x * k * sinh(k * t0) / cosh(k * t0) + xv  = 0
  // x * k * t0anh(k * t0) + xv  = 0
  // x * k * t0anh(k * t0) = -xv
  // tanh(k * t0) = -xv / (x * k)
  // t0 = atanh(-xv / (x * k)) / k

  // find td, ntu in
  // r + x0 * cosh(k * td) + xv0 * sinh(k * td) / k = ns + nr + nx0 * cosh(nk * ntu)
  // x0 * k * sinh(k * td) + xv0 * cosh(k * td) = nx0 * nk * sinh(nk * ntu)
  // (td > t0)

  const float x = phase.x0.y();
  const float xv = phase.xv0.y();
  const float k = phase.k.y();
  const float tanhKTl1 = -xv / (x * k);
  ASSERT(x != 0.f && tanhKTl1 > -1.f && tanhKTl1 < 1.f);
  const float kT = abs(tanhKTl1) < 1.f ? atanh(tanhKTl1) : tanhKTl1 * 1.e15f;
  const float t0 = kT / k;
  const float x0 = x * cosh(kT) + xv / k * sinh(kT);
  ASSERT(fabs(x0) > 0.5f);

  struct Data : public FunctionMinimizer
  {
    float x0;
    float xv0;
    float nx0;
    float nk;
    float k;
    float r;
    float nr;
    float ns;

    virtual float func(float td) const
    {
      const float kTd = k * td;
      const float sinhKTd = sinh(kTd);
      const float coshKTd = cosh(kTd);
      const float nkNtu = asinh((x0 * k * sinhKTd + xv0 * coshKTd) / (nx0 * nk));
      return fabs(r + x0 * coshKTd + xv0 * sinhKTd / k - (ns + nr + nx0 * cosh(nkNtu)));
    }
  } d;

  d.x0 = phase.x0.y();
  d.xv0 = phase.xv0.y();
  d.nx0 = nextPhase.x0.y();
  d.nk = nextPhase.k.y();
  d.k = phase.k.y();
  d.r = phase.r.y();
  d.nr = nextPhase.r.y();
  d.ns = nextPhase.s.translation.y();

  const float tdStart = t0 + asinh(d.nx0 * d.nk * sinh(d.nk * nextPhase.tu) / (x0 * d.k)) / d.k;

  bool clipped;
  const float td = d.minimize(t0, 1000000.f, max(t0 + 0.01f, tdStart), 0.001f, 0.05f, clipped, "updatePendulumParametersY");
  const float xvdy = d.x0 * d.k * sinh(d.k * td) + d.xv0 * cosh(d.k * td);
  const float ntu = asinh(xvdy / (d.nx0 * d.nk)) / d.nk;
  ASSERT(ntu < 0.f);
  ASSERT(!clipped || td > 0.8f);
  if(clipped)
  {
    ASSERT(fabs((d.r + phase.x0.y() * cosh(d.k * td) + phase.xv0.y() * sinh(d.k * td) / d.k) - (d.ns + d.nr + d.nx0 * cosh(d.nk * ntu))) < 1.f);
    ASSERT(fabs(d.x0 * d.k * sinh(d.k * td) + d.xv0 * cosh(d.k * td) - (d.nx0 * d.nk * sinh(d.nk * ntu))) < 0.1f);
  }

  phase.td = td;
  nextPhase.tu = ntu;
}

void WalkingEngine::updatePendulumParametersX(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const
{
  if(phase.toStand)
  {
    ASSERT(nextPhase.x0.x() == 0);
    ASSERT(nextPhase.xv0.x() == 0);
    ASSERT(nextPhase.r.x() == walkRef.x());

    // find x, r and xv? in:
    // r + x = px
    // r + x * cosh(k * td) + xv * sinh(k * td) / k = npx
    // x * k * sinh(k * td) + xv * cosh(k * td) = npxv

    struct Data : public FunctionMinimizer
    {
      float px;
      float npx;
      float npxv;
      float k;
      float coshKTd;
      float sinhKTd;

      virtual float func(float r) const
      {
        const float x = px - r;
        const float xv = (npxv - x * k * sinhKTd) / coshKTd;
        return fabs(r + x * coshKTd + xv * sinhKTd / k - npx);
      }
    } d;

    d.px = phase.r.x() + phase.x0.x();
    d.npx = nextPhase.r.x() + nextPhase.x0.x();
    d.npxv = nextPhase.xv0.x();
    d.k = phase.k.x();
    d.coshKTd = cosh(d.k * phase.td);
    d.sinhKTd = sinh(d.k * phase.td);

    bool clipped;
    const float r = d.minimize(-1000000.f, 1000000.f, phase.r.x(), 0.1f, 0.05f, clipped, "updatePendulumParametersXtoStand");
    float x = d.px - r;
    float xv = (d.npxv - x * d.k * d.sinhKTd) / d.coshKTd;
    if(clipped)
    {
      x = ((d.npx - r) * d.k - d.npxv / d.coshKTd * d.sinhKTd) / (d.k * (d.coshKTd - d.sinhKTd * d.sinhKTd / d.coshKTd));
      xv = (d.npxv - x * d.k * d.sinhKTd) / d.coshKTd;
    }

    phase.r.x() = r;
    phase.x0.x() = x;
    phase.xv0.x() = xv;
    return;
  }

  if(nextPhase.toStand)
  {
    // find x, r, nx, nr in:
    // r + x = px
    // r + x * cosh(k * td) + xv * sinh(k * td) / k = ns + nr + nx * cosh(nk * ntu)
    // x * k * sinh(k * td) + xv * cosh(k * td) = nx * nk * sinh(nk * ntu)
    // rn + nx = npx

    struct Data : public FunctionMinimizer
    {
      float px;
      float xv;
      float ns;
      float k;
      float coshKTd;
      float sinhKTd;
      float nk;
      float npx;
      float coshNkNtu;
      float sinhNkNtu;

      virtual float func(float r) const
      {
        const float x = px - r;
        const float nx = (x * k * sinhKTd + xv * coshKTd) / (nk * sinhNkNtu);
        const float nr = npx - nx;
        return fabs(r + x * coshKTd + xv * sinhKTd / k - (ns + nr + nx * coshNkNtu));
      }
    } d;

    d.px = phase.r.x() + phase.x0.x();
    d.xv = phase.xv0.x();
    d.ns = nextPhase.s.translation.x();
    d.k = phase.k.x();
    d.coshKTd = cosh(d.k * phase.td);
    d.sinhKTd = sinh(d.k * phase.td);
    d.nk = nextPhase.k.x();
    d.npx = nextPhase.r.x() + nextPhase.x0.x();
    d.coshNkNtu = cosh(d.nk * nextPhase.tu);
    d.sinhNkNtu = sinh(d.nk * nextPhase.tu);

    bool clipped;
    const float r = d.minimize(-1000000.f, 1000000.f, phase.r.x(), 0.1f, 0.05f, clipped, "updatePendulumParametersXnextToStand");
    const float x = d.px - r;
    const float nx = (x * d.k * d.sinhKTd + d.xv * d.coshKTd) / (d.nk * d.sinhNkNtu);
    const float nr = d.npx - nx;
    // if(clipped) // do nothing... !

    phase.r.x() = r;
    phase.x0.x() = x;
    nextPhase.r.x() = nr;
    nextPhase.x0.x() = nx;
    nextPhase.s.translation.x() = d.ns;
    return;
  }

  ASSERT(!phase.toStand && !nextPhase.toStand);

  bool clipped;
  float r, x;
  float sinhKTd = sinh(phase.k.x() * phase.td);
  float coshKTd = cosh(phase.k.x() * phase.td);

  //if(init)
  {
    // find x, r in:
    // r + x = px
    // x * k * sinh(k * td) + xv * cosh(k * td) = nxvu

    struct Data : public FunctionMinimizer
    {
      float px;
      float xv;
      float sinhKTd;
      float coshKTd;
      float k;
      float nxvu;

      virtual float func(float r) const
      {
        const float x = px - r;
        return fabs(x * k * sinhKTd + xv * coshKTd - nxvu);
      }
    } d;

    d.px = phase.r.x() + phase.x0.x();
    d.xv = phase.xv0.x();
    d.sinhKTd = sinhKTd;
    d.coshKTd = coshKTd;
    d.k = phase.k.x();
    ASSERT(nextPhase.x0.x() == 0.f);
    d.nxvu = nextPhase.xv0.x() * cosh(nextPhase.k.x() * nextPhase.tu);

    Rangef rLimit(phase.rOpt.x() + (init ? walkRefXPlanningLimit.min : walkRefXLimit.min), phase.rOpt.x() + (init ? walkRefXPlanningLimit.max : walkRefXLimit.max));
    r = d.minimize(rLimit.min, rLimit.max, phase.r.x(), 0.1f, 0.05f, clipped, "updatePendulumParametersXinit");
    x = d.px - r;
  }

  if(clipped)
  {
    float xvd = x * phase.k.x() * sinhKTd + phase.xv0.x() * coshKTd;

    // limit step size
    const Rangef& xvdxLimit = init ? walkXvdXPlanningLimit : walkXvdXLimit;
    if(!xvdxLimit.isInside(xvd))
    {
      xvd = xvdxLimit.limit(xvd);

      // find x, r in:
      // r + x = px
      // x * k * sinh(k * td) + xv * cosh(k * td) = xvd

      struct Data : public FunctionMinimizer
      {
        float px;
        float xv;
        float xvd;
        float k;
        float sinhKTd;
        float coshKTd;

        virtual float func(float r) const
        {
          const float x = px - r;
          return fabs(x * k * sinhKTd + xv * coshKTd - xvd);
        }
      } d2;

      d2.px = phase.r.x() + phase.x0.x();
      d2.xv = phase.xv0.x();
      d2.xvd = xvd;
      d2.k = phase.k.x();
      d2.sinhKTd = sinhKTd;
      d2.coshKTd = coshKTd;

      r = d2.minimize(-1000000.f, 1000000.f, phase.r.x(), 0.1f, 0.05f, clipped, "updatePendulumParametersXklhglk");
      x = d2.px - r;
      if(clipped)
        xvd = x * d2.k * d2.sinhKTd + d2.xv * d2.coshKTd;

      // TODO: improve this: reduce nsy? reduce td?
    }

    const float nxv0 = xvd / cosh(nextPhase.k.x() * nextPhase.tu);

    nextPhase.xv0.x() = nxv0;
  }

  const float ns = r + x * coshKTd + phase.xv0.x() * sinhKTd / phase.k.x() - (nextPhase.r.x() + nextPhase.xv0.x() * sinh(nextPhase.k.x() * nextPhase.tu) / nextPhase.k.x());
  nextPhase.s.translation.x() = ns;
  phase.r.x() = r;
  phase.x0.x() = x;

  if(init)
    phase.rRef = phase.r;
}

void WalkingEngine::drawW() const
{
  const float scale = 8.f;
  Vector3f p0 = Vector3f::Zero();
  Vector3f px2 = Vector3f(200.f * scale * 0.75f, 0.f, 0.f);
  Vector3f py2 = Vector3f(0.f, 200.f * scale * 0.75f, 0.f);
  Vector3f pz2 = Vector3f(0.f, 0.f, 200.f * scale * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:W", p0, px2, 2 * scale, 20 * scale, 10 * scale, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:W", p0, py2, 2 * scale, 20 * scale, 10 * scale, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:W", p0, pz2, 2 * scale, 20 * scale, 10 * scale, ColorRGBA(0, 0, 255));
}

void WalkingEngine::drawP() const
{
  Vector3f p0 = Vector3f(0.f, 0.f, 85.f);
  Vector3f px2 = Vector3f(200.f * 0.75f, 0.f, 0.f);
  Vector3f py2 = Vector3f(0.f, 200.f * 0.75f, 0.f);
  Vector3f pz2 = Vector3f(0.f, 0.f, 200.f * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3f(px2.x(), px2.y(), px2.z() + 85.f), 2, 20, 10, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3f(py2.x(), py2.y(), py2.z() + 85.f), 2, 20, 10, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:P", p0, Vector3f(pz2.x(), pz2.y(), pz2.z() + 85.f), 2, 20, 10, ColorRGBA(0, 0, 255));
}

void WalkingEngine::drawQ(const WalkingEngineOutput& walkingEngineOutput) const
{
  RobotModel robotModel(walkingEngineOutput, theRobotDimensions, theMassCalibration);
  Pose3f originToQ = robotModel.limbs[predictedPendulumPlayer.phase.type == leftSupportPhase ? Limbs::footLeft : Limbs::footRight];
  originToQ.translate(0, 0, -theRobotDimensions.footHeight);
  originToQ.conc(predictedPendulumPlayer.phase.type == leftSupportPhase ? targetPosture.leftOriginToFoot.inverse() : targetPosture.rightOriginToFoot.inverse());

  Vector3f px2 = originToQ * Vector3f(200.f * 0.75f, 0.f, 0.f);
  Vector3f p0 = originToQ * Vector3f::Zero();
  Vector3f py2 = originToQ * Vector3f(0.f, 200.f * 0.75f, 0.f);
  Vector3f pz2 = originToQ * Vector3f(0.f, 0.f, 200.f * 0.75f);
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3f(px2.x(), px2.y(), px2.z()), 2, 20, 10, ColorRGBA(255, 0, 0));
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3f(py2.x(), py2.y(), py2.z()), 2, 20, 10, ColorRGBA(0, 255, 0));
  CYLINDERARROW3D("module:WalkingEngine:Q", p0, Vector3f(pz2.x(), pz2.y(), pz2.z()), 2, 20, 10, ColorRGBA(0, 0, 255));
}

void WalkingEngine::drawZmp()
{
  Vector3f leftFoot(0.f, standComPosition.y, 0.f);
  Vector3f rightFoot(0.f, -standComPosition.y, 0.f);

  if(pendulumPlayer.phase.type == leftSupportPhase)
    rightFoot += Vector3f(pendulumPlayer.nextPhase.s.translation.x(), pendulumPlayer.nextPhase.s.translation.y(), 0.f);
  if(pendulumPlayer.phase.type == rightSupportPhase)
    leftFoot += Vector3f(pendulumPlayer.nextPhase.s.translation.x(), pendulumPlayer.nextPhase.s.translation.y(), 0.f);

  FOOT3D("module:WalkingEngine:zmp", Pose3f(leftFoot), true, ColorRGBA(0, 0, 0));
  FOOT3D("module:WalkingEngine:zmp", Pose3f(rightFoot), false, ColorRGBA(0, 0, 0));

  Vector2f r = pendulumPlayer.phase.r;
  Vector2f rOpt = pendulumPlayer.phase.rOpt;
  Vector2f px = r + pendulumPlayer.phase.x0;

  SPHERE3D("module:WalkingEngine:zmp", px.x(), px.y(), walkHeight.y(), 6, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", measuredPx.x(), measuredPx.y(), walkHeight.y(), 6, ColorRGBA(0, 255, 0));

  LINE3D("module:WalkingEngine:zmp", r.x(), r.y(), 0.f, px.x(), px.y(), walkHeight.y(), 2, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", r.x(), r.y(), 0.f, 1, ColorRGBA(0, 0, 0));

  SPHERE3D("module:WalkingEngine:zmp", pendulumPlayer.phase.rOpt.x(), pendulumPlayer.phase.rOpt.y(), 0.f, 1, ColorRGBA(0, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", pendulumPlayer.phase.rRef.x(), pendulumPlayer.phase.rRef.y(), 0.f, 1, ColorRGBA(0, 0, 255));
  SPHERE3D("module:WalkingEngine:zmp", measuredR.x(), measuredR.y(), 0.f, 1, ColorRGBA(0, 255, 0));

  SPHERE3D("module:WalkingEngine:zmp", rOpt.x() + walkRefXLimit.max, rOpt.y(), 0.f, 1, ColorRGBA(255, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", rOpt.x() + walkRefXLimit.min, rOpt.y(), 0.f, 1, ColorRGBA(255, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", rOpt.x(), rOpt.y() + walkRefYLimit.max, 0.f, 1, ColorRGBA(255, 0, 0));
  SPHERE3D("module:WalkingEngine:zmp", rOpt.x(), rOpt.y() + walkRefYLimit.min, 0.f, 1, ColorRGBA(255, 0, 0));
}

void WalkingEngine::drawStats()
{
  DEBUG_DRAWING("module:WalkingEngine:stats", "drawingOnImage")
  {
    DRAWTEXT("module:WalkingEngine:stats", 5, 15, 10, ColorRGBA::black, "max velocity:");
    DRAWTEXT("module:WalkingEngine:stats", 5, 30, 10, ColorRGBA::black, "    x: " << (speedMax.translation.x() / (walkStepDurationAtFullSpeedX / 1000.f)) << " mm/s");
    DRAWTEXT("module:WalkingEngine:stats", 5, 45, 10, ColorRGBA::black, "    y: " << (speedMax.translation.y() / (walkStepDurationAtFullSpeedY / 1000.f)) << " mm/s");
    DRAWTEXT("module:WalkingEngine:stats", 5, 60, 10, ColorRGBA::black, "  rot: " << (speedMax.rotation.toDegrees() / (walkStepDuration / 1000.f)) << " deg/s");
  }
}
