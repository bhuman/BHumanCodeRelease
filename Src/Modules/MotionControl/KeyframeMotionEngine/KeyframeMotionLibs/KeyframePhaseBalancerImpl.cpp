/**
 * @file KeyframePhaseBalancerImpl.cpp
 * This file declares functions for the KeyframePhase
 * Content: Implementation of all methods, that are needed for balancing
 * @author Philip Reichenberg
 */

#include "KeyframePhaseBase.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Plot.h"
#include "Framework/Settings.h"
#include "Modules/MotionControl/KeyframeMotionEngine/KeyframeMotionEngine.h"
#include "Streaming/Global.h"

void KeyframePhaseBase::pidWithCom()
{
  if(engine.theFrameInfo.getTimeSince(lastPIDCom) > 2.f * Global::getSettings().motionCycleTime * 1000.f)   // reset values, if balancing was not active for one frame
  {
    valPID_I = valPID_D = valPID_P = Vector2f::Zero();
    balancerPitchValue.clear();
    balancerRollValue.clear();
  }

  lastPIDCom = engine.theFrameInfo.time;
  const KeyframeMotionParameters::BalanceFactors& factors = engine.theKeyframeMotionParameters.balanceList[Phase(currentKeyframe.phase)].balanceFactor;
  // Base PID values

  valPID_D = (comDiff - oldCom) - goalGrowth; // (Current Measured Position - Last Measured Position) - Current Desired Velocity
  valPID_P = comDiff - currentGoal; // Current Measured Position - Current Desired Position
  if(state == EngineState::balanceOut || currentKeyframe.phase == Phase::StandStatic)
    valPID_I -= valPID_I.normalized(std::min(7000.f * Global::getSettings().motionCycleTime, valPID_I.norm()));
  else
    valPID_I += valPID_P;

  // Algorithm to calculate the balance values, which will be added on the joints
  // It is a PID based balancing, but the balance parameters scale exponentially with the size of the P-part.
  auto calcBalanceValue = [](const float comDiffBaseForMaxNegativeBalancing, const float comDiffBaseForMaxPositiveBalancing,
                             const float exponentFactorNegativeScaling, const float exponentFactorPositiveScaling,
                             const float maxValNegativeAdditionalScaling, const float maxValPositiveAdditionalScaling,
                             const float pValue, const float iValue, const float dValue,
                             const KeyframeMotionParameters::PIDParameter& pidPParameter,
                             const KeyframeMotionParameters::PIDParameter& pidDParameter,
                             LowPassFilterPR& balanceValue, float& pidD,
                             const bool isPitchCase)
  {
    // The exponential scaling
    const float nQuadMin = comDiffBaseForMaxNegativeBalancing == 0 ? 1.f : 1.f / std::pow(comDiffBaseForMaxNegativeBalancing, exponentFactorNegativeScaling);
    const float nQuadMax = comDiffBaseForMaxPositiveBalancing == 0 ? 1.f : 1.f / std::pow(comDiffBaseForMaxPositiveBalancing, exponentFactorPositiveScaling);

    const float scaling = pValue < 0
                          ? std::min(nQuadMin * std::pow(pValue, exponentFactorNegativeScaling), maxValNegativeAdditionalScaling)
                          : std::min(nQuadMax * std::pow(pValue, exponentFactorPositiveScaling), maxValPositiveAdditionalScaling);

    const float scalingD = pValue * dValue >= 0.f ? scaling : 0.f;

    // P, I and D values
    const float pidP = pValue * (pidPParameter.baseValue + std::min(scaling, pidPParameter.maxAdditionalValue));
    const float pidI = iValue * 0.02f; // TODO Eval if I-part can be removed
    pidD = (dValue / Global::getSettings().motionCycleTime) * (pidDParameter.baseValue + std::min(scalingD, pidDParameter.maxAdditionalValue)); // save D-part value for balanceOut function

    // Combine into one balancing value
    const float balanceValueBase = (pidP + pidI + pidD) * Global::getSettings().motionCycleTime;

    // Low-pass filter and clip result to prevent oscillating
    balanceValue.update(balanceValueBase);

    if(isPitchCase)
    {
      PLOT("module:KeyframeMotionEngine:pid:p", pidP * Global::getSettings().motionCycleTime);
      PLOT("module:KeyframeMotionEngine:pid:i", pidI * Global::getSettings().motionCycleTime);
      PLOT("module:KeyframeMotionEngine:pid:d", pidD * Global::getSettings().motionCycleTime);
    }
  };

  // Balance value for forward / backward tilt (pitch)
  calcBalanceValue(currentComDiffBaseForBalancing.pitchDirection.min, currentComDiffBaseForBalancing.pitchDirection.max, factors.exponentFactorScaling.pitchDirection.min, factors.exponentFactorScaling.pitchDirection.max,
                   factors.maxValAdditionalScaling.pitchDirection.min, factors.maxValAdditionalScaling.pitchDirection.max, valPID_P.x(), valPID_I.x(), valPID_D.x(),
                   factors.pidPPitchParameter, factors.pidDPitchParameter, balancerPitchValue, pidDForward, true);

  // Balance value for sidewards tilt (roll)
  float dummy = 0.f; // dummy value because the return value is not used
  calcBalanceValue(currentComDiffBaseForBalancing.rollDirection.min, currentComDiffBaseForBalancing.rollDirection.max, factors.exponentFactorScaling.rollDirection.min, factors.exponentFactorScaling.rollDirection.max,
                   factors.maxValAdditionalScaling.rollDirection.min, factors.maxValAdditionalScaling.rollDirection.max, valPID_P.y(), valPID_I.y(), valPID_D.y(),
                   factors.pidPRollParameter, factors.pidDRollParameter, balancerRollValue, dummy, false);

  // Apply balance values
  if(!engine.stepKeyframes && balancerOn)
    addBalanceFactor(balancerPitchValue.currentValue, balancerRollValue.currentValue);

  PLOT("module:KeyframeMotionEngine:pid:borderB", currentGoal.x() + currentComDiffBaseForBalancing.pitchDirection.min);
  PLOT("module:KeyframeMotionEngine:pid:borderF", currentGoal.x() + currentComDiffBaseForBalancing.pitchDirection.max);
  PLOT("module:KeyframeMotionEngine:pid:com", comDiff.x());
  PLOT("module:KeyframeMotionEngine:pid:balance", balancerPitchValue.currentValue);

  // TODO activate based on HYP error?
  if(!engine.stepKeyframes && state != EngineState::balanceOut)
  {
    const float timer = static_cast<float>(engine.theFrameInfo.time % engine.oscillationPeriod);
    const float ratio = std::sin(Constants::pi2 * timer / engine.oscillationPeriod);
    const float scaling = mapToRange(std::abs(jointDiffPredictedAngles.angles[Joints::lHipYawPitch]), static_cast<float>(engine.hypErrorOscillationScaling.min), static_cast<float>(engine.hypErrorOscillationScaling.max), 0.f, 1.f);
    jointRequestOutput.angles[Joints::lAnkleRoll] += scaling * ratio * engine.oscillationAmplitude;
    jointRequestOutput.angles[Joints::rAnkleRoll] -= scaling * ratio * engine.oscillationAmplitude;
  }
}

void KeyframePhaseBase::addBalanceFactor(const float factorY, const float factorX)
{
  auto applyBalance = [this](const std::vector<JointBalancePair>& jointFactorList, const float balanceValue, const bool forwardTiltCase)
  {
    for(JointBalancePair jointList : jointFactorList)
    {
      Joints::Joint joint = jointList.joint;
      float factor = jointList.scaling;
      JointBalancePair::JointBalanceType jointBalanceType = jointList.jointBalanceType;

      if(isMirror)
        joint = Joints::mirror(joint);
      if(isMirror &&
         ((forwardTiltCase && (joint == Joints::lAnkleRoll  // forward / backward balancing. Roll needs to switch sign, because feet are rotated around the z-axis
                               || joint == Joints::rAnkleRoll
                               || joint == Joints::lHipRoll
                               || joint == Joints::rHipRoll))
          ||
          (!forwardTiltCase && (joint == Joints::lAnklePitch // sideways balancing. Pitch needs to switch sign, because feet are rotated around the z-axis
                                || joint == Joints::rAnklePitch
                                || joint == Joints::lKneePitch
                                || joint == Joints::rKneePitch
                                || joint == Joints::lHipPitch
                                || joint == Joints::rHipPitch))))
      {
        factor *= -1.f;
        jointBalanceType = JointBalancePair::mirror(joint, jointBalanceType, forwardTiltCase);
      }
      if(targetJoints.angles[joint] == JointAngles::off || targetJoints.angles[joint] == JointAngles::ignore || targetJoints.stiffnessData.stiffnesses[joint] == 0)
        targetJointRequestWithCompensation.angles[joint] = lastUnbalanced.angles[joint];
      const Rangef qRange(engine.theJointLimits.limits[joint].min, engine.theJointLimits.limits[joint].max);
      jointRequestOutput.angles[joint] += JointBalancePair::applyJointBalanceType(jointBalanceType, Angle::fromDegrees(balanceValue * factor), forwardTiltCase, joint);
      jointRequestOutput.angles[joint] = qRange.limit(jointRequestOutput.angles[joint]);
    }
  };
  applyBalance(currentKeyframe.balanceWithJoints.jointY, factorY, true);
  applyBalance(currentKeyframe.balanceWithJoints.jointX, factorX, false);
}

void KeyframePhaseBase::calculateCOMInSupportPolygon()
{
  // The feet edge points
  std::vector<Vector3f> supportPolygon;
  const Pose3f leftFoot = engine.theTorsoMatrix * engine.theRobotModel.soleLeft;
  const Pose3f rightFoot = engine.theTorsoMatrix * engine.theRobotModel.soleRight;
  supportPolygon.push_back(((leftFoot * Vector3f(engine.supportPolygonOffsets.x(), -engine.supportPolygonOffsets.y(), 0.f))));
  supportPolygon.push_back(((rightFoot * Vector3f(engine.supportPolygonOffsets.x(), engine.supportPolygonOffsets.y(), 0.f))));
  supportPolygon.push_back(((rightFoot * Vector3f(-engine.theRobotDimensions.soleToBackEdgeLength, engine.supportPolygonOffsets.y(), 0.f))));
  supportPolygon.push_back(((leftFoot * Vector3f(-engine.theRobotDimensions.soleToBackEdgeLength, -engine.supportPolygonOffsets.y(), 0.f))));

  Vector3f sumOfEdges = Vector3f::Zero();
  for(size_t i = 0; i < supportPolygon.size(); i++)
    sumOfEdges += supportPolygon[i] / supportPolygon.size();

  // Ref values for balancing
  const Pose3f comInTorso = engine.theTorsoMatrix * engine.theRobotModel.centerOfMass;
  const Vector3f balanceDiff = comInTorso.translation - sumOfEdges;
  oldCom = comDiff;
  comDiff = { balanceDiff.x(), balanceDiff.y() };

  if(comDiff.x() >= 0)
    comDistanceToEdge.x() = std::max(comInTorso.translation.x() - supportPolygon[0].x(), comInTorso.translation.x() - supportPolygon[1].x());
  else
    comDistanceToEdge.x() = std::max(supportPolygon[2].x() - comInTorso.translation.x(), supportPolygon[3].x() - comInTorso.translation.x());

  if(comDiff.y() >= 0)
    comDistanceToEdge.y() = std::max(comInTorso.translation.y() - supportPolygon[0].y(), comInTorso.translation.y() - supportPolygon[3].y());
  else
    comDistanceToEdge.y() = std::max(supportPolygon[1].y() - comInTorso.translation.y(), supportPolygon[2].y() - comInTorso.translation.y());

  COMPLEX_DRAWING3D("module:KeyframeMotionEngine:supportPolygon")
  {
    for(size_t i = 0; i < supportPolygon.size(); i++)
    {
      const Vector3f p1 = engine.theTorsoMatrix.inverse() * supportPolygon[i];
      const Vector3f p2 = engine.theTorsoMatrix.inverse() * supportPolygon[(i + 1) % supportPolygon.size()];
      LINE3D("module:KeyframeMotionEngine:supportPolygon",
             p1.x(), p1.y(), p1.z(),
             p2.x(), p2.y(), p2.z(),
             10, ColorRGBA::red);
    }
    const Vector3f mid = engine.theTorsoMatrix.inverse() * sumOfEdges;
    const Vector3f comInFloor = engine.theTorsoMatrix.inverse() * Vector3f(comInTorso.translation.x(), comInTorso.translation.y(), sumOfEdges.z());
    CROSS3D("module:KeyframeMotionEngine:supportPolygon", mid.x(), mid.y(), mid.z(), 10, 10, ColorRGBA::red);
    CROSS3D("module:KeyframeMotionEngine:supportPolygon", comInFloor.x(), comInFloor.y(), comInFloor.z(), 10, 10, ColorRGBA::blue);
  }
}

void KeyframePhaseBase::doBalanceOut()
{
  if(currentMotion.balanceOut) // TODO
  {
    state = EngineState::balanceOut;
    currentMotion.balanceOut = false;
    balancerOn = true;
    goalGrowth = Vector2f::Zero();
    doBalanceOutTimestamp = engine.theFrameInfo.time;
  }
  ratio = 1.f;
  if(!isInBreakUpRange())
  {
    if((engine.theFrameInfo.getTimeSince(doBalanceOutTimestamp) > engine.balanceOutParams.maxTime
        || (std::abs(fluctuation.y()) < engine.balanceOutParams.minFluctuation
            && engine.theInertialData.angle.y() < engine.balanceOutParams.minForwardAngle
            && engine.theInertialData.angle.y() > engine.balanceOutParams.minBackwardAngle
            && std::abs(pidDForward) < engine.balanceOutParams.minPIDDValue
            && engine.theRobotModel.soleLeft.translation.z() < -228.f
            && engine.theRobotModel.soleRight.translation.z() < -228.f)) ||
       (engine.theRobotModel.soleLeft.translation.z() < -228.f
        && engine.theRobotModel.soleRight.translation.z() < -228.f))
    {
      state = EngineState::waitForRequest;
      finishGettingUp = false;
    }
    if(state != EngineState::finished && state != EngineState::breakUp && state != EngineState::waitForRequest)
      setNextJoints();
  }
}
