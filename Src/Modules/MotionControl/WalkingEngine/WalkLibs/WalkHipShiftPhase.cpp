#include "../WalkingEngine.h"

WalkHipShiftPhase::WalkHipShiftPhase(WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
                                     const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback, const WalkKickStep& walkKickStep) :
  WalkPhase(engine, stepTarget, lastPhase, true, createNextPhaseCallback, walkKickStep)
{
  update();
}

void WalkHipShiftPhase::update()
{
  if(!finishedShifting)
  {
    doPseudoWalkStepAdjustmentUpdate();

    // Update
    const auto& params = engine.walkHipShiftPhaseParameters;
    const Angle swingSoleRotToGround = (isLeftPhase ? engine.theRobotModel.soleLeft : engine.theRobotModel.soleRight).rotation.getYAngle() + engine.theInertialData.angle.y();
    float comInterpolationFactor = swingSoleRotToGround < params.comShiftScalingBackward.max ?
                                   mapToRange(swingSoleRotToGround, params.comShiftScalingBackward.min, params.comShiftScalingBackward.max, 0_deg, Angle(1.f)) :
                                   mapToRange(swingSoleRotToGround, params.comShiftScalingForward.min, params.comShiftScalingForward.max, Angle(1.f), 0_deg);
    if(lastPhase && lastPhase->type == MotionPhase::walk)
    {
      const auto& lastWalkPhaseDummy = static_cast<const WalkPhaseBase&>(*lastPhase);
      const float lastXStep = lastWalkPhaseDummy.step.translation.x();
      comInterpolationFactor = lastWalkPhaseDummy.step.translation.x() < 0.f ?
                               std::min(comInterpolationFactor, mapToRange(lastXStep, params.comShiftScalingWalkingBackward.min, params.comShiftScalingWalkingBackward.max, 0.f, 1.f)) :
                               std::min(comInterpolationFactor, mapToRange(lastXStep, params.comShiftScalingWalkingForward.min, params.comShiftScalingWalkingForward.max, 1.f, 0.f));
      if(walkKickStep.currentKick != WalkKicks::none)
        comInterpolationFactor = std::max(0.5f, comInterpolationFactor);
    }
    comInterpolationFactor = std::min(comInterpolationFactor, mapToRange(expectedDelta, params.deltaScaling.min, params.deltaScaling.max, 1.f, 0.f));

    if(weightShiftStatus == emergencyStand)
      comInterpolationFactor = 1.f;

    const float maxShift = walkState == starting ? params.maxShift.max : mapToRange(comInterpolationFactor, 0.f, 1.f, params.maxShift.min, params.maxShift.max);
    const float stopConditionWalking = mapToRange(comInterpolationFactor, 0.f, 1.f, params.comYStopConditionWalking.min, params.comYStopConditionWalking.max);
    finishedShifting = std::abs(leftShift) >= maxShift * 0.95f ||
                       std::abs(rightShift) >= maxShift * 0.95f ||
                       engine.theRobotStableState.comInTorso[isLeftPhase ? Legs::right : Legs::left].outerSideAbsolute >= (walkState == starting ? params.comYStopConditionStarting : stopConditionWalking);
    // When starting to walk, at least some momentum to the support leg is necessary
    if(walkState == starting && std::abs(leftShift) < params.minShiftStartWalking && std::abs(rightShift) < params.minShiftStartWalking)
      finishedShifting = false;
    if(finishedShifting)
    {
      sideL0 += leftShift;
      sideR0 += rightShift;
    }
  }
  if(finishedShifting)
    WalkPhase::update();
}

void WalkHipShiftPhase::doPseudoWalkStepAdjustmentUpdate()
{
  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;
  JointRequest jointRequest;
  jointRequest.angles.fill(JointAngles::ignore);

  engine.calcFeetPoses(forwardL0, forwardR0, sideL0 + leftShift, sideR0 + rightShift, footHL0, footHR0, turnRL0, currentWalkHipHeight, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  // Set and compensate arm position
  for(std::size_t j = Joints::firstLegJoint; j < Joints::numOfJoints; j++)
    jointRequest.angles[j] = 0_deg;
  JointRequest walkArms = jointRequest;
  setArms(leftFootForArms, rightFootForArms, jointRequest, walkArms);

  const float oldArmCompensationAfterKick = armCompensationAfterKick;
  const auto oldArmCompensationTilt = armCompensationTilt;
  compensateArms(jointRequest, walkArms);
  armCompensationAfterKick = oldArmCompensationAfterKick;
  armCompensationTilt = oldArmCompensationTilt;
  VERIFY(InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, 10.f));

  const bool oldFreezeSideMovement = freezeSideMovement;
  const auto oldWalkStepAdjustment = walkStepAdjustment;
  balanceFeetPoses(leftFootForArms, rightFootForArms);
  expectedDelta = walkStepAdjustment.delta;
  freezeSideMovement = oldFreezeSideMovement;
  walkStepAdjustment = oldWalkStepAdjustment;
}

void WalkHipShiftPhase::calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  if(finishedShifting)
  {
    WalkPhase::calcJoints(motionRequest, jointRequest, odometryOffset, motionInfo);
    return;
  }

  // left swing phase-> shift hip to right, right swing phase -> shift hip to left
  const float useShiftSpeed = walkState == starting ? engine.walkHipShiftPhaseParameters.shiftSpeedStart : engine.walkHipShiftPhaseParameters.shiftSpeed;
  const float maxSideSpeed = isLeftPhase ? useShiftSpeed * engine.motionCycleTime : -useShiftSpeed * engine.motionCycleTime;

  // Left
  leftShift += maxSideSpeed;

  // Right
  rightShift += maxSideSpeed;

  // return support foot to 0 if it was still lifted
  const Rangef range(-engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime, engine.commonSpeedParameters.fastFeetAdjustment * engine.motionCycleTime); // height can adjust to 0 this much per motion frame
  float& supportHL0 = isLeftPhase ? footHR0 : footHL0;
  supportHL0 -= range.limit(supportHL0);

  // Return soleRotations
  const Angle useSoleRotationOffsetSpeed = walkState != starting && engine.theFrameInfo.getTimeSince(timeWhenLastKick) > engine.commonSpeedParameters.soleRotationOffsetSpeedAfterKickTime ?
                                           this->engine.commonSpeedParameters.soleRotationOffsetSpeed.max :
                                           this->engine.commonSpeedParameters.soleRotationOffsetSpeed.min;
  const Rangea limitAnkleOffset(-useSoleRotationOffsetSpeed * engine.motionCycleTime, useSoleRotationOffsetSpeed * engine.motionCycleTime);
  soleRotationYL -= limitAnkleOffset.limit(soleRotationYL);
  soleRotationXL -= limitAnkleOffset.limit(soleRotationXL);
  soleRotationYR -= limitAnkleOffset.limit(soleRotationYR);
  soleRotationXR -= limitAnkleOffset.limit(soleRotationXR);

  // calculate joint request
  Pose3f leftFootForArms;
  Pose3f rightFootForArms;

  engine.calcFeetPoses(forwardL0, forwardR0, sideL0 + leftShift, sideR0 + rightShift, footHL0, footHR0, turnRL0, currentWalkHipHeight, soleRotationYL, soleRotationXL, soleRotationYR, soleRotationXR, leftFootForArms, rightFootForArms); // current request

  // Set and compensate arm position
  for(std::size_t j = Joints::firstLegJoint; j < Joints::numOfJoints; j++)
    jointRequest.angles[j] = 0_deg;
  JointRequest walkArms = jointRequest;
  setArms(leftFootForArms, rightFootForArms, jointRequest, walkArms);

  compensateArms(jointRequest, walkArms);

  VERIFY(InverseKinematic::calcLegJoints(leftFootForArms, rightFootForArms, Vector2f(0.f, -armCompensationTilt[0]), jointRequest, engine.theRobotDimensions, 10.f));

  // update regulator
  jointSpeedController->reset(jointRequest.angles, turnRL0);

  // apply gyro balancing
  static_cast<void>(addGyroBalance(jointRequest, JointAngles()));
}
