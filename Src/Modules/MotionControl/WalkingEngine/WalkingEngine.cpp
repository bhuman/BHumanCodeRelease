/**
 * @file WalkingEngine.cpp
 * Implementation of a module that creates the walking motions
 * @author Colin Graf
 * @author Alexis Tsogias
 */

#include "WalkingEngine.h"
#include "Platform/SystemCall.h"
#include "Tools/Motion/SensorData.h"
#include "Tools/Settings.h"
#include "Tools/Boundary.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Rotation.h"
#include "Tools/Optimization/FunctionMinimizer.h"
#include "Tools/Streams/InStreams.h"
#include <algorithm>

MAKE_MODULE(WalkingEngine, motionControl)

WalkingEngine::WalkingEngine() :
  wke(theWalkKicks)
{
  init();

  if(SystemCall::getMode() == SystemCall::simulatedRobot)
    observerMeasurementDelay = 60.f;

  // reset internal state
  reset();
}

void WalkingEngine::onRead()
{
  init();
}

void WalkingEngine::init()
{
  // pre compute constants
  standBodyRotation = RotationMatrix::aroundY(standBodyTilt);
  walkPhaseDuration = walkStepDuration * (0.001f * 0.5f);
  walkPhaseDurationAtFullSpeedX = walkStepDurationAtFullSpeedX * (0.001f * 0.5f);
  walkPhaseDurationAtFullSpeedY = walkStepDurationAtFullSpeedY * (0.001f * 0.5f);

  // compute walkXvdXSoftLimit and walkXvdXHardLimit
  const float td = walkPhaseDurationAtFullSpeedX * 0.5f;
  LIP lip(0.f, 0.f, walkHeight.x());

  lip.velocity = lip.requiredVelocityForPosition(walkStepSizeXPlanningLimit.max * 0.5f, td);
  walkXvdXPlanningLimit.max = lip.predict(td).velocity;
  ASSERT(std::abs(lip.predict(td).position - walkStepSizeXPlanningLimit.max * 0.5f) < 0.1f);

  lip.velocity = lip.requiredVelocityForPosition(walkStepSizeXPlanningLimit.min * 0.5f, td);
  walkXvdXPlanningLimit.min = lip.predict(td).velocity;
  ASSERT(std::abs(lip.predict(td).position - walkStepSizeXPlanningLimit.min * 0.5f) < 0.1f);

  lip.velocity = lip.requiredVelocityForPosition(walkStepSizeXLimit.max * 0.5f, td);
  walkXvdXLimit.max = lip.predict(td).velocity;
  ASSERT(std::abs(lip.predict(td).position - walkStepSizeXLimit.max * 0.5f) < 0.1f);

  lip.velocity = lip.requiredVelocityForPosition(walkStepSizeXLimit.min * 0.5f, td);
  walkXvdXLimit.min = lip.predict(td).velocity;
  ASSERT(std::abs(lip.predict(td).position - walkStepSizeXLimit.min * 0.5f) < 0.1f);
}

void WalkingEngine::reset()
{
  currentMotionType = MotionType::stand;
  requestedMotionType = MotionType::stand;

  pendulumPlayer.engine = this;
  generateFirstPendulumPhase(pendulumPlayer.phase);
  generateNextPendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase, Pose2f());
  pendulumPlayer.seek(0.f);

  lastExpectedLeftToCom = Vector3f::Zero();
  lastExpectedRightToCom = Vector3f::Zero();

  odometryOffset = Pose2f();
  upcomingOdometryOffset = Pose2f();
}

void WalkingEngine::update(WalkingEngineState& walkingEngineState)
{
  if(theLegMotionSelection.ratios[MotionRequest::walk] > 0.f || theLegMotionSelection.ratios[MotionRequest::stand] > 0.f)
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
    generateArmJointRequest();
    computeOdometryOffset();
  }
  else
    reset();
  generateWalkingEngineState(walkingEngineState);

  drawZmp();
  drawStats();
  plot();
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
  requestedMotionType = MotionType::stand;
  if(theGroundContactState.contact && theLegMotionSelection.ratios[MotionRequest::walk] >= 1.f)
    if(theMotionRequest.motion == MotionRequest::walk)
    {
      /*
      if(currentMotionType == MotionType::stand)
      {
        Vector3f angleDiff = (standBodyRotation.inverse() * theTorsoMatrix.rotation).getPackedAngleAxis();
        if(angleDiff.squaredNorm() > 0.0001f) // don't start walking until the robot stopped fluctuating
          SystemCall::playSound("doh.wav");
        requestedMotionType = MotionType::stepping;
      }
      */
      requestedMotionType = MotionType::stepping;
    }
}

void WalkingEngine::updatePendulumPlayer()
{
  // motion update
  if(pendulumPlayer.phase.type != PhaseType::standPhase || pendulumPlayer.nextPhase.type != PhaseType::standPhase)
  {
    pendulumPlayer.seek(theFrameInfo.cycleTime);
    if(pendulumPlayer.phase.type == PhaseType::standPhase && pendulumPlayer.nextPhase.type == PhaseType::standPhase)
      currentMotionType = MotionType::stand;
  }

  // change motion type from stand to stepping
  if(currentMotionType == MotionType::stand && requestedMotionType == MotionType::stepping)
  {
    generateFirstPendulumPhase(pendulumPlayer.phase);
    generateNextPendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase, Pose2f());
    updatePendulumPhase(pendulumPlayer.phase, pendulumPlayer.nextPhase, true);
    currentMotionType = MotionType::stepping;
  }
}

void WalkingEngine::computeMeasuredPosture()
{
  measuredLeftToCom = -Pose3f(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.soleLeft).translation;
  measuredRightToCom = -Pose3f(theTorsoMatrix.rotation).translate(-theRobotModel.centerOfMass).conc(theRobotModel.soleRight).translation;
}

void WalkingEngine::computeExpectedPosture()
{
  LegPosture expectedPosture;
  pendulumPlayer.getPosture(expectedPosture, 0, 0, &observedStepOffset);

  expectedLeftToCom = expectedPosture.leftOriginToCom - expectedPosture.leftOriginToFoot.translation;
  expectedRightToCom = expectedPosture.rightOriginToCom - expectedPosture.rightOriginToFoot.translation;
  expectedComVelocity = pendulumPlayer.phase.com.velocity;
}

void WalkingEngine::computeEstimatedPosture()
{
  if(lastExpectedLeftToCom.z() == 0.f)
  {
    estimatedLeftToCom = expectedLeftToCom;
    estimatedRightToCom = expectedRightToCom;
    estimatedComVelocity = expectedComVelocity;

    covX << sqr(observerProcessDeviation[0]), 0, observerProcessDeviation[0] * observerProcessDeviation[2],
      0, sqr(observerProcessDeviation[0]), observerProcessDeviation[0] * observerProcessDeviation[2],
      observerProcessDeviation[0] * observerProcessDeviation[2], observerProcessDeviation[0] * observerProcessDeviation[2], sqr(observerProcessDeviation[2]);

    covY << sqr(observerProcessDeviation[1]), 0, observerProcessDeviation[1] * observerProcessDeviation[3],
      0, sqr(observerProcessDeviation[1]), observerProcessDeviation[1] * observerProcessDeviation[3],
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
  Matrix3f a = Matrix3f::Identity();
  if(pendulumPlayer.phase.type == PhaseType::leftSupportPhase)
    a(0, 2) = theFrameInfo.cycleTime;
  else if(pendulumPlayer.phase.type == PhaseType::rightSupportPhase)
    a(1, 2) = theFrameInfo.cycleTime;

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
  }
  else
  {
    errorLeft = errorRight = Vector3f::Zero();
    errorVelocity = Vector2f::Zero();
  }
}

void WalkingEngine::correctPendulumPlayer()
{
  if(balance && pendulumPlayer.phase.type != PhaseType::standPhase)
    applyCorrection(pendulumPlayer.phase, pendulumPlayer.nextPhase);
}

void WalkingEngine::updatePredictedPendulumPlayer()
{
  predictedPendulumPlayer = pendulumPlayer;

  // predict future pendulum state
  if(pendulumPlayer.phase.type != PhaseType::standPhase || pendulumPlayer.nextPhase.type != PhaseType::standPhase)
    predictedPendulumPlayer.seek(observerMeasurementDelay * 0.001f);
}

void WalkingEngine::generateTargetPosture()
{
  predictedPendulumPlayer.getPosture(targetPosture);
}

void WalkingEngine::generateArmJointRequest()
{
  armJointRequest.angles[Joints::lShoulderPitch] = targetPosture.leftArmJointAngles[0];
  armJointRequest.angles[Joints::lShoulderRoll] = targetPosture.leftArmJointAngles[1];
  armJointRequest.angles[Joints::lElbowYaw] = targetPosture.leftArmJointAngles[2];
  armJointRequest.angles[Joints::lElbowRoll] = targetPosture.leftArmJointAngles[3];
  armJointRequest.angles[Joints::lWristYaw] = -90_deg;
  armJointRequest.angles[Joints::lHand] = 0.f;
  armJointRequest.angles[Joints::rShoulderPitch] = targetPosture.rightArmJointAngles[0];
  armJointRequest.angles[Joints::rShoulderRoll] = targetPosture.rightArmJointAngles[1];
  armJointRequest.angles[Joints::rElbowYaw] = targetPosture.rightArmJointAngles[2];
  armJointRequest.angles[Joints::rElbowRoll] = targetPosture.rightArmJointAngles[3];
  armJointRequest.angles[Joints::rWristYaw] = 90_deg;
  armJointRequest.angles[Joints::rHand] = 0.f;
}

void WalkingEngine::generateWalkingEngineState(WalkingEngineState& walkingEngineState)
{
  if(pendulumPlayer.phase.type != PhaseType::standPhase)
  {
    const float stepDuration = (pendulumPlayer.phase.td - pendulumPlayer.nextPhase.tu) * 2.f;
    walkingEngineState.unfinishedOutput.speed.translation = (pendulumPlayer.phase.stepSize.translation + pendulumPlayer.nextPhase.stepSize.translation).head<2>() / stepDuration;
    walkingEngineState.unfinishedOutput.speed.rotation = (pendulumPlayer.phase.stepSize.rotation + pendulumPlayer.nextPhase.stepSize.rotation) / stepDuration;
  }
  else
    walkingEngineState.unfinishedOutput.speed = Pose2f();

  walkingEngineState.unfinishedOutput.maxSpeed.translation.x() = (speedMax.translation.x() / (walkStepDurationAtFullSpeedX * 0.001f));
  walkingEngineState.unfinishedOutput.maxSpeed.translation.y() = (speedMax.translation.y() / (walkStepDurationAtFullSpeedY * 0.001f));
  walkingEngineState.unfinishedOutput.maxSpeed.rotation = (speedMax.rotation / (walkStepDuration * 0.001f));

  if(pendulumPlayer.phase.type == PhaseType::leftSupportPhase)
    walkingEngineState.unfinishedOutput.supportFoot = WalkingEngineOutput::SupportFoot::left;
  else if(pendulumPlayer.phase.type == PhaseType::rightSupportPhase)
    walkingEngineState.unfinishedOutput.supportFoot = WalkingEngineOutput::SupportFoot::right;
  else
    walkingEngineState.unfinishedOutput.supportFoot = WalkingEngineOutput::SupportFoot::both;

  walkingEngineState.unfinishedOutput.odometryOffset = odometryOffset;
  walkingEngineState.unfinishedOutput.upcomingOdometryOffset = upcomingOdometryOffset;
  walkingEngineState.unfinishedOutput.standing = currentMotionType == MotionType::stand;
  walkingEngineState.unfinishedOutput.isLeavingPossible = currentMotionType == MotionType::stand;
  walkingEngineState.unfinishedOutput.executedWalk = theMotionRequest.walkRequest;
  walkingEngineState.unfinishedOutput.executedWalk.walkKickRequest.kickLeg = pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? Legs::right : Legs::left;
  walkingEngineState.unfinishedOutput.executedWalk.walkKickRequest.kickType = pendulumPlayer.phase.kickType;
  walkingEngineState.targetLegPosture = targetPosture;
  walkingEngineState.standBodyRotation = standBodyRotation;
  walkingEngineState.observerMeasurementDelay = observerMeasurementDelay;
}

void WalkingEngine::generateFirstPendulumPhase(PendulumPhase& phase)
{
  phase.type = PhaseType::standPhase;
  phase.com.setLIPHeights(walkHeight);
  phase.com.position = Vector2f::Zero();
  phase.com.velocity = Vector2f::Zero();
  phase.td = observerMeasurementDelay * (0.001f * 0.5f);
  phase.tu = -phase.td;
  phase.lift = Vector3f::Zero();
  phase.comLift = Vector3f::Zero();
  phase.stepSize = StepSize();
  phase.rOpt = phase.rRef = phase.r = Vector2f(walkRef.x(), 0.f);
  phase.toStand = false;
  phase.fromStand = false;
  phase.kickType = WalkKicks::none;
  phase.id = phaseBuffer.empty() ? 1 : (phaseBuffer.front().id + 1);

  phaseBuffer.push_front(phase);
}

void WalkingEngine::generateNextPendulumPhase(const PendulumPhase& phase, PendulumPhase& nextPhase, const Pose2f& lastStepOffset)
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
  if(requestedMotionType == MotionType::stepping && phase.type == PhaseType::standPhase)
  {
    nextPhase.type = ((theMotionRequest.walkRequest.mode == WalkRequest::targetMode ? requestedWalkTarget.translation.y() : theMotionRequest.walkRequest.speed.translation.y()) > 0.f) ? PhaseType::rightSupportPhase : PhaseType::leftSupportPhase;
    // TODO: check sign
  }
  else if((requestedMotionType == MotionType::stepping && !phase.toStand) || (phase.type != PhaseType::standPhase && requestedMotionType == MotionType::stand && !phase.toStand))
  {
    ASSERT(phase.type == PhaseType::leftSupportPhase || phase.type == PhaseType::rightSupportPhase);
    nextPhase.type = phase.type == PhaseType::leftSupportPhase ? PhaseType::rightSupportPhase : PhaseType::leftSupportPhase;
  }
  else
    nextPhase.type = PhaseType::standPhase;

  // stand?
  if(nextPhase.type == PhaseType::standPhase)
  {
    ASSERT(phase.toStand || phase.type == PhaseType::standPhase);
    generateFirstPendulumPhase(nextPhase);
    ASSERT(nextPhase.type == PhaseType::standPhase);
    ASSERT(nextPhase.id == nextPhaseId);
    ASSERT(phaseBuffer.front().id == nextPhaseId);
    return;
  }

  // leftSupportPhase or rightSupportPhase
  nextPhase.id = nextPhaseId;
  const float sign = nextPhase.type == PhaseType::leftSupportPhase ? 1.f : -1.f;
  nextPhase.r = Vector2f(walkRef.x(), walkRef.y() * sign);
  nextPhase.com.setLIPHeights(walkHeight);
  nextPhase.toStand = requestedMotionType == MotionType::stand && !phase.fromStand;
  nextPhase.fromStand = phase.type == PhaseType::standPhase;
  nextPhase.kickType = WalkKicks::none;
  nextPhase.lift = Vector3f(walkLiftOffset.x(), walkLiftOffset.y() * sign, walkLiftOffset.z());
  nextPhase.comLift = Vector3f(walkComLiftOffset.x(), walkComLiftOffset.y() * sign, walkComLiftOffset.z());

  if(nextPhase.toStand && (std::abs(phase.stepSize.translation.x()) < walkStepSizeXPlanningLimit.max * 0.5f || theFallDownState.state != FallDownState::upright))
  {
    nextPhase.td = 0;
    nextPhase.tu = walkPhaseDuration * -0.5f;
    nextPhase.stepSize = StepSize();
    nextPhase.com.position.x() = 0;
    nextPhase.com.position.y() = -nextPhase.r.y();
    nextPhase.com.velocity = Vector2f::Zero();
  }
  else
  {
    nextPhase.toStand = false;
    nextPhase.td = walkPhaseDuration * 0.5f;
    nextPhase.tu = walkPhaseDuration * -0.5f;

    const WalkKickVariant& requestetWalkKick = theMotionRequest.walkRequest.walkKickRequest;
    float kickPhaseDuration = 0.f;
    if(nextPhase.fromStand || phase.fromStand)
    {
      nextPhase.lift = Vector3f::Zero();
      nextPhase.stepSize = StepSize();
    }
    else if(phase.kickType != WalkKicks::none)
    {
      const WalkKick& kick = theWalkKicks.kicks[phase.kickType];
      const Pose2f& kickStepSize = kick.stepSize;
      nextPhase.stepSize.rotation = kickStepSize.rotation;
      nextPhase.stepSize.translation << kickStepSize.translation, 0.f;

      ASSERT(phase.type != PhaseType::standPhase);
      Legs::Leg kickLeg = phase.type == PhaseType::leftSupportPhase ? Legs::right : Legs::left;
      if(kick.kickLeg != kickLeg)
      {
        nextPhase.stepSize.rotation *= -1.f;
        nextPhase.stepSize.translation.y() *= -1.f;
      }
    }
    else if(requestetWalkKick.kickType != WalkKicks::Type::none && requestetWalkKick.kickLeg == (nextPhase.type == PhaseType::leftSupportPhase ? Legs::right : Legs::left))
    {
      nextPhase.kickType = requestetWalkKick.kickType;
      const WalkKick& kick = theWalkKicks.kicks[requestetWalkKick.kickType];
      kickPhaseDuration = kick.duration;

      const Pose2f& preStepSize = kick.preStepSize;
      nextPhase.stepSize.rotation = preStepSize.rotation;
      nextPhase.stepSize.translation << preStepSize.translation, 0.f;
      nextPhase.r.x() = kick.origin.x();
      if(kick.kickLeg != requestetWalkKick.kickLeg)
      {
        nextPhase.stepSize.rotation *= -1.f;
        nextPhase.stepSize.translation.y() *= -1.f;
        nextPhase.r.y() -= kick.origin.y();
      }
      else
        nextPhase.r.y() += kick.origin.y();
    }
    else
      generateNextStepSize(nextPhase.type, nextPhase.stepSize, lastStepOffset);

    const float refOffset = (sgnPos(nextPhase.stepSize.translation.x()) > 0 ? walkRefAtFullSpeedX.x() : walkRefBackwards) - walkRef.x();
    nextPhase.r.x() += std::abs(nextPhase.stepSize.translation.x()) * refOffset / (speedMax.translation.x() * 0.5f);
    const float rYSpeedFix = (std::abs(nextPhase.stepSize.translation.x()) * (walkRefAtFullSpeedX.y() - walkRef.y()) / (speedMax.translation.x() * 0.5f));
    nextPhase.r.y() += sign * rYSpeedFix;

    nextPhase.lift.x() += std::abs(nextPhase.stepSize.translation.x()) * (walkLiftOffsetAtFullSpeedX.x() - walkLiftOffset.x()) / (speedMax.translation.x() * 0.5f);
    nextPhase.lift.y() += sign * (std::abs(nextPhase.stepSize.translation.x()) * (walkLiftOffsetAtFullSpeedX.y() - walkLiftOffset.y()) / (speedMax.translation.x() * 0.5f));
    nextPhase.lift.z() += std::abs(nextPhase.stepSize.translation.x()) * (walkLiftOffsetAtFullSpeedX.z() - walkLiftOffset.z()) / (speedMax.translation.x() * 0.5f);

    nextPhase.lift.x() += std::abs(nextPhase.stepSize.translation.y()) * (walkLiftOffsetAtFullSpeedY.x() - walkLiftOffset.x()) / speedMax.translation.y();
    nextPhase.lift.y() += sign * std::abs(nextPhase.stepSize.translation.y()) * (walkLiftOffsetAtFullSpeedY.y() - walkLiftOffset.y()) / speedMax.translation.y();
    nextPhase.lift.z() += std::abs(nextPhase.stepSize.translation.y()) * (walkLiftOffsetAtFullSpeedY.z() - walkLiftOffset.z()) / speedMax.translation.y();

    if(nextPhase.lift.z() > 0.f)
      nextPhase.liftRotation << walkLiftRotation.x() * sign * std::abs(nextPhase.stepSize.translation.y()) / speedMax.translation.y(),
      nextPhase.stepSize.translation.x() > 0.f ? (walkLiftRotation.y() * nextPhase.stepSize.translation.x() / (speedMax.translation.x() * 0.5f)) : 0,
      walkLiftRotation.z() * sign;

    const float walkPhaseDurationX = walkPhaseDuration + std::abs(nextPhase.stepSize.translation.x()) * (walkPhaseDurationAtFullSpeedX - walkPhaseDuration) / (speedMax.translation.x() * 0.5f);
    const float walkPhaseDurationY = walkPhaseDurationX + std::abs(nextPhase.stepSize.translation.y()) * (walkPhaseDurationAtFullSpeedY - walkPhaseDuration) / speedMax.translation.y();
    computeNextPendulumParametersY(nextPhase, walkPhaseDurationX, kickPhaseDuration != 0.f ? kickPhaseDuration : walkPhaseDurationY);
    computeNextPendulumParametersX(nextPhase);
  }

  nextPhase.rOpt = nextPhase.rRef = nextPhase.r;
  phaseBuffer.push_front(nextPhase);
}

void WalkingEngine::computeNextPendulumParametersY(PendulumPhase& nextPhase, float walkPhaseDurationX, float walkPhaseDurationY) const
{
  // compute tu and td using walkPhaseDurationX
  nextPhase.td = walkPhaseDurationX * 0.5f;
  nextPhase.tu = -nextPhase.td;

  // compute comPosition using walkPhaseDurationY
  const float td = walkPhaseDurationY * 0.5f;
  nextPhase.com.velocity.y() = 0.f;
  nextPhase.com.position.y() = nextPhase.com.getYLIP().requiredPositionForPosition(-nextPhase.r.y(), td);
  ASSERT(std::abs(nextPhase.r.y() + nextPhase.com.getYLIP().predict(td).position) < 0.1f);
}

void WalkingEngine::computeNextPendulumParametersX(PendulumPhase& nextPhase) const
{
  ASSERT(nextPhase.td != 0.f);
  nextPhase.com.position.x() = 0.f;
  nextPhase.com.velocity.x() = nextPhase.com.getXLIP().requiredVelocityForPosition(nextPhase.stepSize.translation.x() * 0.5f, nextPhase.td);
  ASSERT(std::abs(nextPhase.com.getXLIP().predict(nextPhase.td).position - (nextPhase.stepSize.translation.x() * 0.5f)) < 0.1f);
}

void WalkingEngine::updatePendulumPhase(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const
{
  if(phase.type == PhaseType::standPhase)
  {
    ASSERT(nextPhase.type == PhaseType::standPhase || nextPhase.fromStand);
    ASSERT(std::abs(phase.com.velocity.x()) < 0.1f && std::abs(phase.com.velocity.y()) < 0.1f);
    ASSERT(nextPhase.stepSize.translation == Vector3f::Zero());
    return;
  }

#ifndef NDEBUG
  Vector2f px = phase.r + phase.com.position;
  Vector2f pxv = phase.com.velocity;
#endif

  updatePendulumParametersY(phase, nextPhase);
  updatePendulumParametersX(phase, nextPhase, init);

  ASSERT(nextPhase.tu < 0.f);
  ASSERT(nextPhase.com.position.x() == 0.f || nextPhase.toStand);
  ASSERT(nextPhase.com.velocity.y() == 0.f);

  ASSERT(std::abs(phase.r.y() + phase.com.getYLIP().update(phase.td).position - nextPhase.stepSize.translation.y() -
    (nextPhase.r.y() + nextPhase.com.getYLIP().update(nextPhase.tu).position)) < (phase.td > 0.8f ? 1.f : 0.1f));
  ASSERT(std::abs(phase.com.getYLIP().update(phase.td).velocity - nextPhase.com.getYLIP().update(nextPhase.tu).velocity) < (phase.td > 0.8f ? 1.f : 0.1f));

  ASSERT(std::abs(phase.r.x() + phase.com.position.x() - px.x()) < 0.1f || phase.toStand);
  ASSERT(std::abs(phase.r.y() + phase.com.position.y() - px.y()) < 0.1f || phase.toStand);
  ASSERT(std::abs(phase.com.velocity.x() - pxv.x()) < 0.1f || phase.toStand);
  ASSERT(std::abs(phase.com.velocity.y() - pxv.y()) < 0.1f || phase.toStand);
}

void WalkingEngine::generateNextStepSize(PhaseType nextSupportLeg, StepSize& nextStepSize, const Pose2f& lastStepOffset)
{
  ASSERT(nextSupportLeg == PhaseType::leftSupportPhase || nextSupportLeg == PhaseType::rightSupportPhase);

  if(requestedMotionType != MotionType::stepping)
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
    float maxSpeedForTargetX = std::sqrt(2.f * std::abs(requestedSpeed.translation.x()) * speedMaxChangeBeforeTarget.translation.x());
    if(std::abs(requestedSpeed.translation.x()) > maxSpeedForTargetX)
      requestedSpeed.translation.x() = requestedSpeed.translation.x() >= 0.f ? maxSpeedForTargetX : -maxSpeedForTargetX;

    float maxSpeedForTargetY = std::sqrt(2.f * std::abs(requestedSpeed.translation.y()) * speedMaxChangeBeforeTarget.translation.y());
    if(std::abs(requestedSpeed.translation.y()) > maxSpeedForTargetY)
      requestedSpeed.translation.y() = requestedSpeed.translation.y() >= 0.f ? maxSpeedForTargetY : -maxSpeedForTargetY;

    float maxSpeedForTargetR = std::sqrt(2.f * std::abs(requestedSpeed.rotation) * speedMaxChangeBeforeTarget.rotation);
    if(std::abs(requestedSpeed.rotation) > maxSpeedForTargetR)
      requestedSpeed.rotation = requestedSpeed.rotation >= 0.f ? maxSpeedForTargetR : -maxSpeedForTargetR;
  }

  const Pose2f speedMaxMin(10_deg, 20.f, 0.f);

  // normalize x-y-speed
  const Pose2f maxSpeed(speedMax.rotation, requestedSpeed.translation.x() < 0.f ? speedMaxBackwards : speedMax.translation.x(), speedMax.translation.y());
  {
    Vector2f tmpSpeed = requestedSpeed.translation.array() / (speedMaxMin.translation + maxSpeed.translation).array();
    const float tmpSpeedAbs = tmpSpeed.norm();
    if(tmpSpeedAbs > 1.f)
    {
      tmpSpeed /= tmpSpeedAbs;
      tmpSpeed.array() *= (speedMaxMin.translation + maxSpeed.translation).array();
      requestedSpeed.translation.x() = Rangef(-maxSpeed.translation.x(), maxSpeed.translation.x()).limit(tmpSpeed.x());
      requestedSpeed.translation.y() = Rangef(-maxSpeed.translation.y(), maxSpeed.translation.y()).limit(tmpSpeed.y());
    }
  }

  // normalize speed (including rotation)
  {
    Vector3f tmpSpeed;
    tmpSpeed << requestedSpeed.translation.array() / (speedMaxMin.translation + maxSpeed.translation).array(),
      requestedSpeed.rotation / (speedMaxMin.rotation + maxSpeed.rotation);
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

  // max speed change clipping
  const Pose2f& lastSpeed = lastStepOffset;
  requestedSpeed.rotation = Rangef(lastSpeed.rotation - speedMaxChange.rotation, lastSpeed.rotation + speedMaxChange.rotation).limit(requestedSpeed.rotation);
  const Vector2f lowerBound = lastSpeed.translation - speedMaxChange.translation;
  const Vector2f upperBound = lastSpeed.translation + speedMaxChange.translation;
  Boundaryf speedClipper(Rangef(lowerBound.x(), upperBound.x()), Rangef(lowerBound.y(), upperBound.y()));
  if(requestedSpeed.translation.x() > 0.f && lastSpeed.translation.x() < 0.f)
    speedClipper.x.max = speedMaxChange.translation.x();
  else if(requestedSpeed.translation.x() < 0.f && lastSpeed.translation.x() > 0.f)
    speedClipper.x.min = -speedMaxChange.translation.x();
  speedClipper.clip(requestedSpeed.translation);

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
  if((nextStepSize.translation.y() < 0.f && nextSupportLeg == PhaseType::leftSupportPhase) || (nextStepSize.translation.y() > 0.f && nextSupportLeg != PhaseType::leftSupportPhase))
    nextStepSize.translation.y() = 0.f;
  if((nextStepSize.rotation < 0.f && nextSupportLeg == PhaseType::leftSupportPhase) || (nextStepSize.rotation > 0.f && nextSupportLeg != PhaseType::leftSupportPhase))
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
    Pose3f footLeft = theRobotModel.soleLeft;
    Pose3f footRight = theRobotModel.soleRight;
    Vector3f odometryOrigin = (footLeft.translation + footRight.translation) * 0.5f;
    if(lastOdometryOrigin.z() != 0.f)
    {
      Pose3f& footSupport = pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? footLeft : footRight;
      Pose3f& lastFootSupport = pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? lastFootLeft : lastFootRight;
      Pose3f odometryOffset3DinP = (Pose3f(-odometryOrigin).conc(footSupport).conc(lastFootSupport.inverse()).conc(lastOdometryOrigin)).inverse();
      Pose3f odometryOffset3D = Pose3f(theTorsoMatrix).conc(odometryOffset3DinP).conc(theTorsoMatrix.inverse());
      odometryOffset.rotation = odometryOffset3D.rotation.getZAngle() * odometryScale.rotation;
      odometryOffset.translation = (odometryOffset3D.translation.array().head<2>() * odometryScale.translation.array());
    }
    else
      odometryOffset = Pose2f();
    lastFootLeft = footLeft;
    lastFootRight = footRight;
    lastOdometryOrigin = odometryOrigin;
  }

  // compute upcoming odometry offset
  upcomingOdometryOffset = Pose2f((pendulumPlayer.nextPhase.stepSize.rotation - observedStepOffset.rotation) * 0.5f * odometryScale.rotation,
    (pendulumPlayer.nextPhase.stepSize.translation.head<2>() - observedStepOffset.translation).array() * 0.5f * odometryScale.translation.array());
  upcomingOdometryOffset += Pose2f(pendulumPlayer.nextPhase.stepSize.rotation * 0.5f * odometryScale.rotation,
                                   pendulumPlayer.nextPhase.stepSize.translation.head<2>().array() * 0.5f * odometryScale.translation.array());
  if(predictedPendulumPlayer.nextPhase.id > pendulumPlayer.nextPhase.id)
  {
    Pose2f upcomingOdometryOffsetNextPhase(predictedPendulumPlayer.nextPhase.stepSize.rotation * odometryScale.rotation,
                                           predictedPendulumPlayer.nextPhase.stepSize.translation.head<2>().array() * odometryScale.translation.array());
    upcomingOdometryOffset += upcomingOdometryOffsetNextPhase;
  }

  // remove odometry offset from requested walk target
  if(theMotionRequest.walkRequest.mode == WalkRequest::targetMode)
    requestedWalkTarget -= odometryOffset;
}

void WalkingEngine::applyCorrection(PendulumPhase& phase, PendulumPhase& nextPhase)
{
  ASSERT(phase.type != PhaseType::standPhase);
  const Vector2f directError = (phase.type == PhaseType::leftSupportPhase ? errorLeft : errorRight).head<2>();
  const Vector2f indirectError = (phase.type == PhaseType::leftSupportPhase ? errorRight : errorLeft).head<2>();

  if(phase.type != PhaseType::standPhase)
  {
    const Vector2f px = phase.r + phase.com.position;
    const Vector2f xv = phase.com.velocity;
    const Vector2f xa = phase.com.position.array() * phase.com.getK().abs2();

    measuredPx = px + directError;
    const Vector2f measuredPx = px.array() + directError.array() * balanceCom.array();
    Vector2f measuredXv = xv + (measuredPx - px) / theFrameInfo.cycleTime;
    const Vector2f measuredXa = xa + (measuredXv - xv) / theFrameInfo.cycleTime;

    const Vector2f measuredX = measuredXa.array() / phase.com.getK().abs2();
    measuredR = measuredPx - measuredX;

    measuredXv = xv.array() + errorVelocity.array() * balanceComVelocity.array();

    const float rYSign = phase.type == PhaseType::leftSupportPhase ? 1.f : -1.f;
    Rangef rYLimit = Rangef(phase.rOpt.y() + walkRefYLimit.min * rYSign).add(phase.rOpt.y() + walkRefYLimit.max * rYSign);

    // method #1: use measured ZMP as new ref
    // Vector2f newR = phase.r;
    //newR = measuredR; // ???
    //newR.y = rYLimit.limit(newR.y);

    // method #2: p-control
    Vector2f newR = phase.rRef.array() - (measuredR - phase.rRef).array() * balanceRef.array();
    newR.y() = rYLimit.limit(newR.y());

    phase.com.position = measuredPx - newR;
    phase.com.velocity = measuredXv;
    phase.r = newR;
  }

  if(nextPhase.type != PhaseType::standPhase && !nextPhase.toStand)
  {
    nextPhase.r = nextPhase.rOpt.array() + indirectError.array() * balanceNextRef.array();

    Rangef nrXLimit(nextPhase.rOpt.x() + walkRefXLimit.min, nextPhase.rOpt.x() + walkRefXLimit.max);
    const float nrYSign = nextPhase.type == PhaseType::leftSupportPhase ? 1.f : -1.f;
    Rangef nrYLimit = Rangef(nextPhase.rOpt.y() + walkRefYLimit.min * nrYSign).add(nextPhase.rOpt.y() + walkRefYLimit.max * nrYSign);

    nextPhase.r.x() = nrXLimit.limit(nextPhase.r.x());
    nextPhase.r.y() = nrYLimit.limit(nextPhase.r.y());

    // TODO
    float nsxapprox = 2.f * nextPhase.com.velocity.x() * std::cosh(nextPhase.com.getK().x() * nextPhase.tu); // Why???: <- == 2.f * nextPhase.com.getXLIP().update(nextPhase.tu).velocity
    nsxapprox += indirectError.x() * balanceStepSize.x();
    nextPhase.com.velocity.x() = nsxapprox / (2.f * std::cosh(nextPhase.com.getK().x() * nextPhase.tu));

    nextPhase.stepSize.translation.y() += indirectError.y() * balanceStepSize.y();
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
      case PhaseType::standPhase:
      {
        ASSERT(std::abs(phase.com.velocity.x()) < 0.1f);
        ASSERT(std::abs(phase.com.velocity.y()) < 0.1f);
        ASSERT(std::abs(phase.com.position.x()) < 0.1f);
        ASSERT(std::abs(phase.com.position.y()) < 0.1f);
        break;
      }
      case PhaseType::leftSupportPhase:
      case PhaseType::rightSupportPhase:
      {
        phase.com.update(dt);
        break;
      }
      default:
        ASSERT(false);
    }

    if(deltaTime == 0.f)
      break;
    if(phase.td > 0.f)
      continue;

    // phase transition
    const Vector2f px = phase.r + phase.com.position;
    const Vector2f pxv = phase.com.velocity;
    Pose2f lastStepOffset;
    lastStepOffset.rotation = phase.stepSize.rotation + nextPhase.stepSize.rotation;
    lastStepOffset.translation = (phase.stepSize.translation + nextPhase.stepSize.translation).head<2>();
    phase = nextPhase;
    engine->generateNextPendulumPhase(phase, nextPhase, lastStepOffset);
    phase.com.position = (px - phase.stepSize.translation.head<2>()) - phase.r;
    phase.com.velocity = pxv;
    phase.td -= phase.tu;
    phase.tu = 0.f;
    engine->updatePendulumPhase(phase, nextPhase, true);
  }
}

void WalkingEngine::PendulumPlayer::getPosture(LegPosture& stance, float* leftArmAngle, float* rightArmAngle, Pose2f* stepOffset)
{
  const WalkingEngine& p = *engine;
  if(phase.type == PhaseType::standPhase)
  {
    ASSERT(std::abs(phase.com.velocity.x()) < 0.1f && std::abs(phase.com.velocity.y()) < 0.1f);
    stance.leftOriginToCom = stance.rightOriginToCom = (Vector3f() << phase.r + phase.com.position, p.comHeight).finished();
    stance.leftOriginToFoot = Pose3f(0.f, p.feetDistance * 0.5f, 0.f);
    stance.rightOriginToFoot = Pose3f(0.f, -p.feetDistance * 0.5f, 0.f);

    if(stepOffset)
      *stepOffset = Pose2f();
  }
  else
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
        comLift = nextPhase.comLift * (1.f - blend(1.f + ratio));
      else
        comLift = phase.comLift * (1.f - blend(ratio));
    }
    else
      comLift = (ratio <= 0.5f ? phase.comLift : nextPhase.comLift) * (1.f - blend(ratio * 2.f));

    switch(phase.type)
    {
      case PhaseType::leftSupportPhase:
      {
        stance.leftOriginToCom = (Vector3f() << phase.r + phase.com.position, p.comHeight).finished() - comLift;
        stance.rightOriginToCom = stance.leftOriginToCom - nextPhase.stepSize.translation;

        Vector3f rightStepOffsetRotation = nextPhase.liftRotation * swingLift;
        rightStepOffsetRotation.z() += nextPhase.stepSize.rotation * swingMoveFadeIn;
        const Vector3f leftStepOffsetRotation(0.f, swingLift * (phase.rRef.x() - phase.rOpt.x()) * p.walkSupportRotation, phase.stepSize.rotation * swingMoveFadeOut);

        stance.leftOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(leftStepOffsetRotation), Vector3f(0.f, p.feetDistance * 0.5f, 0.f));
        stance.rightOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(rightStepOffsetRotation), Vector3f(0.f, -p.feetDistance * 0.5f, 0.f) + nextPhase.lift * swingLift - (nextPhase.stepSize.translation + phase.stepSize.translation) * swingMoveFadeOut);

        if(leftArmAngle)
          *leftArmAngle = (nextPhase.stepSize.translation.x() * swingMoveFadeIn - phase.stepSize.translation.x() * swingMoveFadeOut) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
        if(rightArmAngle)
          *rightArmAngle = (phase.stepSize.translation.x() * swingMoveFadeOut - nextPhase.stepSize.translation.x() * swingMoveFadeIn) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
      }
      break;

      case PhaseType::rightSupportPhase:
      {
        stance.rightOriginToCom = (Vector3f() << phase.r + phase.com.position, p.comHeight).finished() - comLift;
        stance.leftOriginToCom = stance.rightOriginToCom - nextPhase.stepSize.translation;

        Vector3f leftStepOffsetRotation = nextPhase.liftRotation * swingLift;
        leftStepOffsetRotation.z() += nextPhase.stepSize.rotation * swingMoveFadeIn;
        const Vector3f rightStepOffsetRotation(0.f, swingLift * (phase.rRef.x() - phase.rOpt.x()) * p.walkSupportRotation, phase.stepSize.rotation * swingMoveFadeOut);

        stance.leftOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(leftStepOffsetRotation), Vector3f(0.f, p.feetDistance * 0.5f, 0.f) + nextPhase.lift * swingLift - (nextPhase.stepSize.translation + phase.stepSize.translation) * swingMoveFadeOut);
        stance.rightOriginToFoot = Pose3f(Rotation::AngleAxis::unpack(rightStepOffsetRotation), Vector3f(0.f, -p.feetDistance * 0.5f, 0.f));

        if(rightArmAngle)
          *rightArmAngle = (nextPhase.stepSize.translation.x() * swingMoveFadeIn - phase.stepSize.translation.x() * swingMoveFadeOut) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
        if(leftArmAngle)
          *leftArmAngle = (phase.stepSize.translation.x() * swingMoveFadeOut - nextPhase.stepSize.translation.x() * swingMoveFadeIn) / p.speedMax.translation.x() * p.walkArmRotationAtFullSpeedX;
      }
      break;

      default:
        ASSERT(false);
        break;
    }

    if(stepOffset)
    {
      stepOffset->translation = nextPhase.stepSize.translation.head<2>() * swingMoveFadeIn - phase.stepSize.translation.head<2>() * swingMoveFadeOut;
      stepOffset->rotation = nextPhase.stepSize.rotation * swingMoveFadeIn - phase.stepSize.rotation * swingMoveFadeOut;
    }

    if(phase.type != PhaseType::standPhase && phase.kickType != WalkKicks::Type::none)
    {
      WalkKickEngine& wke = engine->wke;

      Legs::Leg kickLeg = phase.type == PhaseType::leftSupportPhase ? Legs::right : Legs::left;
      if(phase.kickType != wke.getCurrentKickType() || kickLeg != wke.getCurrentKickLeg())
        wke.start(phase.kickType, kickLeg);

      Vector3f additionFootTranslation, additionalFootRotation;
      wke.getState(-phase.tu / (phase.td - phase.tu), additionFootTranslation, additionalFootRotation);
      Pose3f& kickFoot = phase.type == PhaseType::leftSupportPhase ? stance.rightOriginToFoot : stance.leftOriginToFoot;
      kickFoot.conc(Pose3f(Rotation::AngleAxis::unpack(additionalFootRotation), additionFootTranslation));
    }
  }
}

void WalkingEngine::PendulumPlayer::getPosture(Posture& stance)
{
  const WalkingEngine& p = *engine;

  // legs
  float leftArmAngle = 0.f, rightArmAngle = 0.f;
  getPosture((LegPosture&)stance, &leftArmAngle, &rightArmAngle, 0);

  // arms
  float halfArmRotation = p.walkArmRotationAtFullSpeedX * 0.5f;

  // normal WalkingEngine arm movement
  stance.leftArmJointAngles[0] = pi_2 - p.standArmJointAngles.y() - leftArmAngle;
  stance.leftArmJointAngles[1] = p.standArmJointAngles.x();
  stance.leftArmJointAngles[2] = -pi_2;
  stance.leftArmJointAngles[3] = -p.standArmJointAngles.y() - leftArmAngle - halfArmRotation;

  stance.rightArmJointAngles[0] = pi_2 - p.standArmJointAngles.y() - rightArmAngle;
  stance.rightArmJointAngles[1] = -p.standArmJointAngles.x();
  stance.rightArmJointAngles[2] = pi_2;
  stance.rightArmJointAngles[3] = p.standArmJointAngles.y() + rightArmAngle + halfArmRotation;
}

float WalkingEngine::PendulumPlayer::blend(float r)
{
  return 0.5f - std::cos(r * pi) * 0.5f;
}

// equation (4.60):
// rx + x0x * cosh(k * td) + xv0x * sinh(k * td) / k = nsx + nrx                         + nxv0x * sinh(nk * ntu) / nk
// ry + x0y * cosh(k * td) + xv0y * sinh(k * td) / k = nsy + nry + nx0y * cosh(nk * ntu)

// equation (4.61):
// x0x * k * sinh(k * td) + xv0x * cosh(k * td) =                              nxv0x * cosh(nk * ntu)
// x0y * k * sinh(k * td) + xv0y * cosh(k * td) = nx0y * nk * sinh(nk * ntu)

void WalkingEngine::repairPendulumParametersY(PendulumPhase& phase, const PendulumPhase& nextPhase) const
{
  // this function ensures that phase.r, phase.com.position and phase.com.velocity can be used to solve equation (4.60) and (4.61)

  const float sign = phase.type == PhaseType::leftSupportPhase ? 1.f : -1.f;
  float x = phase.com.position.y();
  float xv = phase.com.velocity.y();
  const float k = phase.com.getK().y();
  float r = phase.r.y();
  float px = r + x;

  // check #1: ensure pendulum pivot point is atleast 1mm away from pendulum deflection
  if(x * -sign < 1.f)
  {
    x = -sign;
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
    const float kT = std::atanh(tanhKTl1);
    x0 = LIP(x, xv, phase.com.getLIPHeights().y()).update(kT / k).position;
    t0 = -kT / k;
  }
  if(x0 * -sign < 1.f)
  {
    // ensure pendulum pivot point is atleast 1mm away from pendulum peak
    x0 = 1.f * -sign;

    // r + x0 * cosh(k * t0) = px
    // x0 * k * sinh(k * t0) = pxv

    float kT = std::asinh(xv / (x0 * k));
    t0 = kT / k;
    r = px - x0 * std::cosh(kT);
    x = px - r;
  }
  ASSERT(x0 * -sign >= 1.f);
  ASSERT(std::abs(r + x0 * std::cosh(k * t0) - px) < 0.1f);
  ASSERT(std::abs(x0 * k * std::sinh(k * t0) - xv) < 0.1f);

  //check #4:
  if((r + x0 - (nextPhase.r.y() + nextPhase.com.position.y())) * -sign > nextPhase.stepSize.translation.y() * -sign - 1.f)
  {
    const float px0 = nextPhase.stepSize.translation.y() + sign + (nextPhase.r.y() + nextPhase.com.position.y());

    // r + x0 = px0
    // r + x0 * cosh(k * t) = px
    // x0 * k * sinh(k * t) = pxv

    // We can't solve this system of equations for r and x0 since px might be equal px0.
    // So, we will keep r and change x and xv:

    x0 = px0 - r;
    if(x0 * -sign < 1.f)
    {
      x0 = -sign;
      r = px0 - x0;
    }

    px = r + x0 * std::cosh(k * t0);
    xv = x0 * k * std::sinh(k * t0);
    x = px - r;
  }
  else
    ASSERT(std::abs(r + x - px) < 0.1f);

  ASSERT(x * -sign >= 1.f);
  ASSERT(x0 * -sign >= 1.f);
  ASSERT(std::abs(r + x0 * std::cosh(k * t0) - px) < 0.1f);
  ASSERT(std::abs(x0 * k * std::sinh(k * t0) - xv) < 0.1f);
  ASSERT((r + x0 - (nextPhase.r.y() + nextPhase.com.position.y())) * -sign < nextPhase.stepSize.translation.y() * -sign);

  phase.r.y() = r;
  phase.com.position.y() = x;
  phase.com.velocity.y() = xv;
}

void WalkingEngine::updatePendulumParametersY(PendulumPhase& phase, PendulumPhase& nextPhase) const
{
  if(phase.toStand)
  {
    ASSERT(nextPhase.com.position.y() == -nextPhase.r.y());
    ASSERT(nextPhase.com.velocity.y() == 0.f);

    LIP com = phase.com.getYLIP();
    updatePendulumParametersToStand(com, phase.r.y(), nextPhase.r.y() + nextPhase.com.position.y(), nextPhase.com.velocity.y(), phase.td);
    phase.com.position.y() = com.position;
    phase.com.velocity.y() = com.velocity;

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

  const float x = phase.com.position.y();
  const float xv = phase.com.velocity.y();
  const float k = phase.com.getK().y();
  const float tanhKTl1 = -xv / (x * k);
  ASSERT(x != 0.f && tanhKTl1 > -1.f && tanhKTl1 < 1.f);
  const float kT = std::abs(tanhKTl1) < 1.f ? std::atanh(tanhKTl1) : tanhKTl1 * 1.e15f;
  const float t0 = kT / k;
  const float x0 = phase.com.getYLIP().update(t0).position;
  ASSERT(std::abs(x0) > 0.5f);

  struct Data : public FunctionMinimizer::Function
  {
    const LIP com;
    const LIP nextCom;
    float origin;
    float nextOrigin;
    float stepSize;

    Data(const LIP& com, const LIP& nextCom) : com(com), nextCom(nextCom) {}

  protected:
    float function(float td) const
    {
      const float nk = nextCom.getK();
      const LIP forwardedCom = com.predict(td);
      const float ntu = std::asinh(forwardedCom.velocity / (nextCom.position * nk)) / nk;
      return std::abs(origin + forwardedCom.position - (stepSize + nextOrigin + nextCom.predict(ntu).position));
    }
  } d(phase.com.getYLIP(), nextPhase.com.getYLIP());

  d.origin = phase.r.y();
  d.nextOrigin = nextPhase.r.y();
  d.stepSize = nextPhase.stepSize.translation.y();

  const float tdStart = t0 + std::asinh(nextPhase.com.getYLIP().update(nextPhase.tu).velocity / (x0 * k)) / k;

  bool clipped;
  const float td = FunctionMinimizer::minimize(d, t0, 1000000.f, std::max(t0 + 0.01f, tdStart), 0.001f, 0.05f, clipped);
  const float nk = nextPhase.com.getK().y();
  const float ntu = std::asinh(phase.com.getYLIP().update(td).velocity / (nextPhase.com.position.y() * nk)) / nk;
  ASSERT(ntu < 0.f);
  ASSERT(!clipped || td > 0.8f);
  if(clipped)
  {
    const LIP endCom = phase.com.getYLIP().update(td);
    const LIP nextStartCom = nextPhase.com.getYLIP().update(ntu);
    ASSERT(std::abs((phase.r.y() + endCom.position) - nextPhase.stepSize.translation.y() - (nextPhase.r.y() + nextStartCom.position)) < 1.f);
    ASSERT(std::abs(endCom.velocity - nextStartCom.velocity) < 0.1f);
  }

  phase.td = td;
  nextPhase.tu = ntu;
}

void WalkingEngine::updatePendulumParametersX(PendulumPhase& phase, PendulumPhase& nextPhase, bool init) const
{
  if(phase.toStand)
  {
    ASSERT(nextPhase.com.position.x() == 0);
    ASSERT(nextPhase.com.velocity.x() == 0);
    ASSERT(nextPhase.r.x() == walkRef.x());

    LIP com = phase.com.getXLIP();
    updatePendulumParametersToStand(com, phase.r.x(), nextPhase.r.x() + nextPhase.com.position.x(), nextPhase.com.velocity.x(), phase.td);
    phase.com.position.x() = com.position;
    phase.com.velocity.x() = com.velocity;
  }
  else if(nextPhase.toStand)
  {
    // find x, r, nx, nr in:
    // r + x = px
    // r + x * cosh(k * td) + xv * sinh(k * td) / k = ns + nr + nx * cosh(nk * ntu)
    // x * k * sinh(k * td) + xv * cosh(k * td) = nx * nk * sinh(nk * ntu)
    // rn + nx = npx

    struct Data : public FunctionMinimizer::Function
    {
      const LIP com;
      float origin;
      float timeRemaining;
      float stepSize;
      float nk;
      float npx;
      float coshNkNtu;
      float sinhNkNtu;

      Data(const LIP& com) : com(com) {}

    protected:
      float function(float zmp) const
      {
        const LIP forwardedCom = com.predict(timeRemaining, zmp);
        const float nx = forwardedCom.velocity / (nk * sinhNkNtu);
        const float nr = npx - nx;
        return std::abs(origin + forwardedCom.position - (stepSize + nr + nx * coshNkNtu));
      }
    } d(phase.com.getXLIP());

    d.origin = phase.r.x();
    d.timeRemaining = phase.td;
    d.stepSize = nextPhase.stepSize.translation.x();
    d.nk = nextPhase.com.getK().x();
    d.npx = nextPhase.r.x() + nextPhase.com.position.x();
    d.coshNkNtu = std::cosh(d.nk * nextPhase.tu);
    d.sinhNkNtu = std::sinh(d.nk * nextPhase.tu);

    bool clipped;
    const float zmp = FunctionMinimizer::minimize(d, -1000000.f, 1000000.f, 0.f, 0.1f, 0.05f, clipped);
    phase.r.x() += zmp;
    phase.com.position.x() -= zmp;
    const float nx = phase.com.getXLIP().update(phase.td).velocity / (d.nk * d.sinhNkNtu);
    nextPhase.r.x() = d.npx - nx;
    nextPhase.com.position.x() = nx;
    // if(clipped) // do nothing... !
  }
  else
  {
    const LIP xCom = phase.com.getXLIP();

    ASSERT(nextPhase.com.position.x() == 0.f);
    const float targetVelocity = nextPhase.com.getXLIP().update(nextPhase.tu).velocity;
    float zmp = xCom.requiredZMPForVelocity(targetVelocity, phase.td);

    const float rDiff = phase.rOpt.x() - phase.r.x();
    Rangef zmpLimit(rDiff + (init ? walkRefXPlanningLimit.min : walkRefXLimit.min), rDiff + (init ? walkRefXPlanningLimit.max : walkRefXLimit.max));
    if(!zmpLimit.isInside(zmp))
    {
      zmp = zmpLimit.limit(zmp);
      float xvd = xCom.predict(phase.td, zmp).velocity;

      // limit step size
      const Rangef& xvdxLimit = init ? walkXvdXPlanningLimit : walkXvdXLimit;
      if(!xvdxLimit.isInside(xvd))
      {
        xvd = xvdxLimit.limit(xvd);
        zmp = xCom.requiredZMPForVelocity(xvd, phase.td);
      }
      nextPhase.com.velocity.x() = xvd / std::cosh(nextPhase.com.getK().x() * nextPhase.tu);
    }

    nextPhase.stepSize.translation.x() = phase.r.x() + xCom.predict(phase.td, zmp).position - (nextPhase.r.x() + nextPhase.com.getXLIP().update(nextPhase.tu).position);
    phase.com.position.x() -= zmp;
    phase.r.x() += zmp;

    if(init)
      phase.rRef = phase.r;
  }
}

void WalkingEngine::updatePendulumParametersToStand(LIP& com, float& r, float targetPosition, float targetVelocity, float timeRemaining) const
{
  // find x, r and xv? in:
  // r + x = px
  // r + x * cosh(k * td) + xv * sinh(k * td) / k = npx
  // x * k * sinh(k * td) + xv * cosh(k * td) = npxv

  struct Data : public FunctionMinimizer::Function
  {
    const LIP com;
    float targetPosition;
    float targetVelocity;
    float timeRemaining;
    float origin;

    Data(const LIP& com) : com(com) {};

  protected:
    float function(float zmp) const
    {
      LIP comMod(com);
      comMod.velocity = com.requiredVelocityForVelocity(targetVelocity, timeRemaining, zmp);
      return std::abs(origin + comMod.update(timeRemaining, zmp).position - targetPosition);
    }
  } d(com);

  d.targetPosition = targetPosition;
  d.targetVelocity = targetVelocity;
  d.timeRemaining = timeRemaining;
  d.origin = r;

  bool clipped;
  const float zmp = FunctionMinimizer::minimize(d, -1000000.f, 1000000.f, 0.f, 0.1f, 0.05f, clipped);
  r += zmp;
  com.position -= zmp;
  com.velocity = com.requiredVelocityForVelocity(targetVelocity, timeRemaining);
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

    const float k = com.getK();
    const float sinhkt = std::sinh(k * timeRemaining);
    const float coshkt = std::cosh(k * timeRemaining);

    com.position = ((targetPosition - r) * k - targetVelocity / coshkt * sinhkt) / (k * (coshkt - sinhkt * sinhkt / coshkt));
    com.velocity = com.requiredVelocityForVelocity(targetVelocity, timeRemaining);
  }
}

void WalkingEngine::drawZmp()
{
  DEBUG_DRAWING3D("module:WalkingEngine:zmp", "field")
  {
    Vector3f leftFoot(0.f, feetDistance * 0.5f, 0.f);
    Vector3f rightFoot(0.f, -feetDistance * 0.5f, 0.f);

    if(pendulumPlayer.phase.type == PhaseType::leftSupportPhase)
      rightFoot += Vector3f(pendulumPlayer.nextPhase.stepSize.translation.x(), pendulumPlayer.nextPhase.stepSize.translation.y(), 0.f);
    if(pendulumPlayer.phase.type == PhaseType::rightSupportPhase)
      leftFoot += Vector3f(pendulumPlayer.nextPhase.stepSize.translation.x(), pendulumPlayer.nextPhase.stepSize.translation.y(), 0.f);

    FOOT3D("module:WalkingEngine:zmp", Pose3f(leftFoot), true, ColorRGBA::black);
    FOOT3D("module:WalkingEngine:zmp", Pose3f(rightFoot), false, ColorRGBA::black);

    const Vector2f r = pendulumPlayer.phase.r;
    const Vector2f rOpt = pendulumPlayer.phase.rOpt;
    const Vector2f px = r + pendulumPlayer.phase.com.position;

    SPHERE3D("module:WalkingEngine:zmp", px.x(), px.y(), walkHeight.y(), 6, ColorRGBA::black);
    SPHERE3D("module:WalkingEngine:zmp", measuredPx.x(), measuredPx.y(), walkHeight.y(), 6, ColorRGBA::green);

    LINE3D("module:WalkingEngine:zmp", r.x(), r.y(), 0.f, px.x(), px.y(), walkHeight.y(), 2, ColorRGBA::black);
    SPHERE3D("module:WalkingEngine:zmp", r.x(), r.y(), 0.f, 1, ColorRGBA::black);

    SPHERE3D("module:WalkingEngine:zmp", pendulumPlayer.phase.rOpt.x(), pendulumPlayer.phase.rOpt.y(), 0.f, 1, ColorRGBA::black);
    SPHERE3D("module:WalkingEngine:zmp", pendulumPlayer.phase.rRef.x(), pendulumPlayer.phase.rRef.y(), 0.f, 1, ColorRGBA::blue);
    SPHERE3D("module:WalkingEngine:zmp", measuredR.x(), measuredR.y(), 0.f, 1, ColorRGBA::green);

    SPHERE3D("module:WalkingEngine:zmp", rOpt.x() + walkRefXLimit.max, rOpt.y(), 0.f, 1, ColorRGBA::red);
    SPHERE3D("module:WalkingEngine:zmp", rOpt.x() + walkRefXLimit.min, rOpt.y(), 0.f, 1, ColorRGBA::red);
    SPHERE3D("module:WalkingEngine:zmp", rOpt.x(), rOpt.y() + walkRefYLimit.max, 0.f, 1, ColorRGBA::red);
    SPHERE3D("module:WalkingEngine:zmp", rOpt.x(), rOpt.y() + walkRefYLimit.min, 0.f, 1, ColorRGBA::red);
  }
}

void WalkingEngine::drawStats()
{
  DEBUG_DRAWING("module:WalkingEngine:stats", "drawingOnImage")
  {
    DRAWTEXT("module:WalkingEngine:stats", 5, 15, 10, ColorRGBA::black, "max velocity:");
    DRAWTEXT("module:WalkingEngine:stats", 5, 30, 10, ColorRGBA::black, "    x: " << (speedMax.translation.x() / (walkStepDurationAtFullSpeedX * 0.001f)) << " mm/s");
    DRAWTEXT("module:WalkingEngine:stats", 5, 45, 10, ColorRGBA::black, "    y: " << (speedMax.translation.y() / (walkStepDurationAtFullSpeedY * 0.001f)) << " mm/s");
    DRAWTEXT("module:WalkingEngine:stats", 5, 60, 10, ColorRGBA::black, "  rot: " << (speedMax.rotation.toDegrees() / (walkStepDuration * 0.001f)) << " deg/s");
  }
}

void WalkingEngine::plot()
{
  PLOT("module:WalkingEngine:measuredCom:x", pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? measuredLeftToCom.x() : measuredRightToCom.x());
  PLOT("module:WalkingEngine:measuredCom:y", pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? measuredLeftToCom.y() : measuredRightToCom.y());
  PLOT("module:WalkingEngine:expectedCom:x", pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? expectedLeftToCom.x() : expectedRightToCom.x());
  PLOT("module:WalkingEngine:expectedCom:y", pendulumPlayer.phase.type == PhaseType::leftSupportPhase ? expectedLeftToCom.y() : expectedRightToCom.y());
  PLOT("module:WalkingEngine:stepSize:x", pendulumPlayer.nextPhase.stepSize.translation.x());
  PLOT("module:WalkingEngine:stepSize:y", pendulumPlayer.nextPhase.stepSize.translation.y());
  PLOT("module:WalkingEngine:t", pendulumPlayer.phase.td);

  // plot kick
  DECLARE_PLOT("module:WalkingEngine:kick:pos:x");
  DECLARE_PLOT("module:WalkingEngine:kick:pos:y");
  DECLARE_PLOT("module:WalkingEngine:kick:pos:z");

  WalkKickVariant selectedKickToPlot;
  MODIFY("module:WalkingEngine:selectedKickToPlot", selectedKickToPlot);
  DEBUG_RESPONSE_ONCE("module:WalkingEngine:plotKick")
  {
    wke.start(selectedKickToPlot.kickType, selectedKickToPlot.kickLeg);
    const WalkKick& kick = theWalkKicks.kicks[selectedKickToPlot.kickType];
    unsigned steps = static_cast<unsigned>(std::ceil(kick.duration / 0.01f));
    for(unsigned i = 0; i <= steps; ++i)
    {
      Vector3f pos, rot;
      wke.getState(static_cast<float>(i) / steps, pos, rot);
      PLOT("module:WalkingEngine:kick:pos:x", pos.x());
      PLOT("module:WalkingEngine:kick:pos:y", pos.y());
      PLOT("module:WalkingEngine:kick:pos:z", pos.z());
      PLOT("module:WalkingEngine:kick:rot:x", rot.x());
      PLOT("module:WalkingEngine:kick:rot:y", rot.y());
      PLOT("module:WalkingEngine:kick:rot:z", rot.z());
    }
  }
}