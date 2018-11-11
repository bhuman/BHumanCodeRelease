/**
 * @file WalkingEngine.cpp
 *
 * This file implements a module that is a wrapper for the UNSW walk generator.
 *
 * @author Thomas Röfer
 */

#include "WalkingEngine.h"

MAKE_MODULE(WalkingEngine, motionControl);

WalkingEngine::WalkingEngine() : walkKickEngine(theWalkKicks)
{
  theWalkGenerator.reset();
}

void WalkingEngine::update(WalkingEngineOutput& walkingEngineOutput)
{
  beginFrame(theFrameInfo.time);
  execute(OptionInfos::getOption("Root"));
  endFrame();
}

void WalkingEngine::updateOutput(bool stepStarted, WalkingEngineOutput& walkingEngineOutput)
{
  if(stepStarted)
  {
    walkingEngineOutput.speed = theWalkGenerator.speed;
    walkingEngineOutput.upcomingOdometryOffset = theWalkGenerator.upcomingOdometryOffset;
    walkingEngineOutput.executedWalk = walkRequest;
    walkingEngineOutput.standing = walkingEngineOutput.speed == Pose2f()
                                   && walkingEngineOutput.odometryOffset == Pose2f();
  }

  walkingEngineOutput.isLeavingPossible = theFrameInfo.getTimeSince(lastTimeWalking) >= minTimeInStandBeforeLeaving;

  walkingEngineOutput.odometryOffset = theWalkGenerator.odometryOffset;
  walkingEngineOutput.upcomingOdometryOffset -= walkingEngineOutput.odometryOffset;
  walkingEngineOutput.maxSpeed = theWalkGenerator.maxSpeed;
  walkingEngineOutput.isKicking = isKicking;
  walkingEngineOutput.lastTarget = lastTarget;

  Pose2f odometryOffset = theWalkGenerator.odometryOffset;
  if(theRobotInfo.hasFeature(RobotInfo::zGyro))
    odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - theOdometryData.rotation);
  target -= odometryOffset;

  walkingEngineOutput.angles = jointRequest.angles;
  walkingEngineOutput.stiffnessData = jointRequest.stiffnessData;
}
void WalkingEngine::updateWalkRequestWithoutKick()
{
  WalkRequest::WalkKickRequest walkKickRequest = walkRequest.walkKickRequest;
  walkRequest = theMotionRequest.walkRequest;
  walkRequest.walkKickRequest = walkKickRequest;
}
void WalkingEngine::walk(const Pose2f& speed, WalkGenerator::WalkMode walkMode,
                         const std::function<Pose3f(float)>& getKickFootOffset)
{
  ASSERT(walkMode != WalkGenerator::targetMode);

  // Prevent standing
  this->speed = speed == Pose2f() ? Pose2f(0.000001f, 0.f) : speed;
  this->walkMode = walkMode;
  this->isKicking = false;
  this->getKickFootOffset = getKickFootOffset;
  lastTarget = Pose2f(100000.f, 100000.f);
}

void WalkingEngine::walkToTarget(const Pose2f& speed, const Pose2f& target)
{
  // Prevent standing
  this->speed = speed;
  if(target != lastTarget)
    this->target = lastTarget = target;
  this->isKicking = false;
  this->walkMode = WalkGenerator::targetMode;
  this->getKickFootOffset = std::function<Pose3f(float)>();
}

void WalkingEngine::stand()
{
  walkRequest = theMotionRequest.walkRequest;
  speed = Pose2f();
  this->walkMode = WalkGenerator::speedMode;
  this->isKicking = false;
  this->getKickFootOffset = std::function<Pose3f(float)>();
  lastTarget = Pose2f(100000.f, 100000.f);
}
void WalkingEngine::resetController()
{
  this->speed = Pose2f(0.f, 0.f, 0.f);
  this->isKicking = false;
  measuredSpeed = Vector2f(0.f, 0.f);
  speedErrorIntegral = Vector2f(0.f, 0.f);
  lastSpeedP = Vector2f(0.f, 0.f);
}
void WalkingEngine::runUp(const Pose2f& speed, const Pose2f& target)
{
  if(theWalkGenerator.t == 0)
  {
    this->walkMode = WalkGenerator::speedMode;

    // convert from distance per second to distance per step
    Pose2f nextSpeed;
    float sD = theWalkGenerator.stepDuration != 0 ? theWalkGenerator.stepDuration : baseWalkPeriod; //can be zero while standing
    Pose2f maxSpeed(theWalkGenerator.maxSpeed.rotation * sD / odometryScale.rotation, theWalkGenerator.maxSpeed.translation * sD); //maybe not 100% accurate, because stepDuration changes
    Vector2f requiredSpeed = speed.translation * sD;
    Vector2f lastSpeed = this->speed.translation * sD;

    // compute target position in support foot corrdinate system
    Pose3f ankleInAnkle;
    if(theMotionRequest.walkRequest.walkKickRequest.kickLeg == Legs::left)
      ankleInAnkle = theRobotModel.limbs[Limbs::ankleRight].inverse() * theRobotModel.limbs[Limbs::ankleLeft];
    else
      ankleInAnkle = theRobotModel.limbs[Limbs::ankleLeft].inverse() * theRobotModel.limbs[Limbs::ankleRight];
    Pose2f odometryInFoot(ankleInAnkle.translation.x() * 0.5f, ankleInAnkle.translation.y() * 0.5f);
    Pose2f targetInFoot = odometryInFoot * target;

    // signs of target direction
    Vector2f signs(targetInFoot.translation.x() > 0 ? 1 : -1, targetInFoot.translation.y() > 0 ? 1 : -1);

    // check if target is reached exactly enough
    bool supportFootSwings = !((theMotionRequest.walkRequest.walkKickRequest.kickLeg == Legs::left && theWalkGenerator.isLeftPhase) || (theMotionRequest.walkRequest.walkKickRequest.kickLeg == Legs::right && !theWalkGenerator.isLeftPhase));
    if(targetInFoot.translation.norm() < 80.f && std::max(0.f, targetInFoot.translation.x()) < 30.f && std::fabs(targetInFoot.translation.y()) < 30.f && std::fabs(ankleInAnkle.translation.y()) < 135.f && !supportFootSwings && this->speed.translation.norm() > 0)
    {
      //OUTPUT_TEXT("Kick. Distance X: " << std::to_string(targetInFoot.translation.x()) << " Y: " << std::to_string(targetInFoot.translation.y()) << " rot: " << std::to_string(targetInFoot.rotation.toDegrees()) << " deg");
      ANNOTATION("WalkingEngine", "runUp finished");
      this->isKicking = true;
      nextSpeed.translation[0] = sD * maxSpeed.translation[0] * Rangef::ZeroOneRange().limit(0.4f + std::fabs(lastSpeed.x() / maxSpeed.translation.x()) / 2.f);
      nextSpeed.translation[1] = maxSpeed.translation[1] * sD * 0.1f * (theMotionRequest.walkRequest.walkKickRequest.kickLeg == Legs::left ? 1.f : -1.f);
      nextSpeed.rotation = 0;
      lastTarget = targetInFoot;
      this->walkMode = WalkGenerator::stepSizeMode;
    }
    else
    {
      isKicking = false;

      // Update Measurement of current speed
      Pose3f leftInRight3d = theRobotModel.soleRight.inverse() * theRobotModel.soleLeft;
      float forwardMeasurement = std::fabs(leftInRight3d.translation.x()) * (lastSpeed.x() > 0 ? 1 : -1);
      float leftMeasurement = std::fabs(std::fabs(leftInRight3d.translation.y()) - 100.f) * (lastSpeed.y() > 0 ? 1 : -1) / 2.f;

      measuredSpeed[0] += speedMeasurementMotionUpdateRate * (forwardMeasurement * odometryScale.translation[0] - measuredSpeed[0]);
      if((theWalkGenerator.isLeftPhase && lastSpeed.y() < 0.f) || (!theWalkGenerator.isLeftPhase && lastSpeed.y() > 0.f))
        measuredSpeed[1] += speedMeasurementMotionUpdateRate * (leftMeasurement * odometryScale.translation[1] - measuredSpeed[1]);

      if((targetInFoot - lastTarget).translation.norm() < 120)      // target did not jump
        measuredSpeed += speedMeasurementVisionUpdateRate * (lastTarget.translation - targetInFoot.translation - measuredSpeed);

      //If measurement or requested speed  is very low, noise will cause a high quotioent. Avoids this:
      if(std::fabs(measuredSpeed.x()) < maxSpeed.translation.x() * 0.1 || std::fabs(lastSpeed.x()) < maxSpeed.translation.x() * 0.1)
        measuredSpeed[0] = lastSpeed[0] = lastSpeed[0] + 0.1f * maxSpeed.translation.x(); //addition to prevent division by zero
      if(std::fabs(measuredSpeed.y()) < maxSpeed.translation.y() * 0.1 || std::fabs(lastSpeed.y()) < maxSpeed.translation.y() * 0.1)
        measuredSpeed[1] = lastSpeed[1] = lastSpeed[1] + 0.1f * maxSpeed.translation.y();

      // Compute approximation of steps to reach target
      float stepsX = std::max(1.f, std::round(std::fabs(targetInFoot.translation.x() / measuredSpeed.x())));
      float stepsY = std::max(1.f, std::round(std::fabs(targetInFoot.translation.y() / maxSpeed.translation.y())));
      float steps = std::max(stepsX, stepsY);
      bool incremented = false;
      float newSteps = steps;
      do
      {
        steps = newSteps;
        //Less steps for y-Direction, if possible, to reduce sidesteps directly before kicking
        Vector2f stepVector(steps, std::max({ 1.f, std::min(steps - 3.f, std::floor(steps * 0.6f)), stepsY }));

        //Speed Control: based on PID + problemspecific component (e-component)
        Vector2f targetSpeed = targetInFoot.translation.cwiseProduct(stepVector.cwiseInverse());
        Vector2f speedE = lastSpeed.cwiseProduct(targetSpeed).cwiseProduct(measuredSpeed.cwiseInverse()).cwiseAbs().cwiseProduct(signs);
        Vector2f speedP = targetSpeed - measuredSpeed;
        Vector2f speedI = speedErrorIntegral + speedP;
        Vector2f speedD = speedP - lastSpeedP;
        nextSpeed.translation = controllerKe.cwiseProduct(speedE) + controllerKp.cwiseProduct(speedP) + controllerTnI.cwiseProduct(speedI) + controllerTv.cwiseProduct(speedD);

        // correct number of steps if its to fast
        if(std::fabs(nextSpeed.translation.x()) > maxSpeed.translation.x() || std::fabs(nextSpeed.translation.y()) > maxSpeed.translation.y())
        {
          newSteps += 1;
          incremented = true;
        }
        // correct number of steps if its too slow (dont do that if it has been incremented before to avoid infinite loop)
        if(std::fabs(nextSpeed.translation.x()) < requiredSpeed.x() * 0.7 && std::fabs(nextSpeed.translation.y()) < requiredSpeed.y() * 0.7 && steps > 1 && !incremented)
          newSteps -= 1;
      }
      while(steps != newSteps);

      //Update Controller Values
      lastSpeedP = targetInFoot.translation / steps - measuredSpeed;
      speedErrorIntegral += lastSpeedP;

      //Compute Turn Speed
      float turnToTarget = targetInFoot.translation.angle();
      if(turnToTarget * targetInFoot.rotation > turnThreshold)         // turn would be good for walking and kicking,
      {
        if(turnToTarget > 0)
          nextSpeed.rotation = Rangef(-maxSpeed.rotation, maxSpeed.rotation).limit(std::min(turnToTarget, static_cast<float>(targetInFoot.rotation)));
        else
          nextSpeed.rotation = Rangef(-maxSpeed.rotation, maxSpeed.rotation).limit(std::max(turnToTarget, static_cast<float>(targetInFoot.rotation)));
      }
      if(steps > 30 && std::fabs(turnToTarget) > 10_deg)   // turn to target if it is still far away
      {
        nextSpeed.rotation = Rangef(-maxSpeed.rotation, maxSpeed.rotation).limit(turnToTarget);
        nextSpeed.translation[1] = 0;
      }
      if(steps <= 10)  // adjust orientation for kick
        nextSpeed.rotation = Rangef(-maxSpeed.rotation * 0.5f, maxSpeed.rotation * 0.5f).limit(targetInFoot.rotation);

      // Dont use controller for the last steps
      if(std::fabs(targetInFoot.translation.x()) < 40)
        nextSpeed.translation[0] = targetInFoot.translation.x() * 1.3f;//last step is okay to override
      if(std::fabs(targetInFoot.translation.y()) < 40)
        nextSpeed.translation[1] = targetInFoot.translation.y(); // optimize y exact in the end
      if(std::fabs(targetInFoot.translation.y()) < 40 && targetInFoot.translation.x() < 60)
        nextSpeed.translation[1] = targetInFoot.translation.y() * 0.25f; //no big sidewards steps before kick. Bad in case of inWalk-Sidekick
      if(targetInFoot.translation.norm() < 80)
        nextSpeed.translation[0] = std::max(0.f, nextSpeed.translation[0]); //turnSpeed: dont go backwards x

      //Debug Output
      //if (supportFootSwings) OUTPUT_TEXT("planned steps: " << steps);
      //OUTPUT_TEXT("[" << forwardMeasurement << "," << leftMeasurement << "," << lastSpeed.x() << "," << lastSpeed.y() << ", " << targetInFoot.translation.y() << "," << measuredSpeed[0] << "," << measuredSpeed[1] << "," << targetInFoot.translation.x() << "," << steps << "],");

      //Pose2f targetOnField = theRobotPose * target;
      //Vector2f posToTarget = theRobotPose.translation - targetOnField.translation;
      //OUTPUT_TEXT("" << posToTarget.x() << "," << posToTarget.y() << "," << static_cast<float>(target.rotation));
    }

    // Prepare data for walkGenerator
    if(supportFootSwings)
      lastTarget = targetInFoot;
    this->speed.rotation = nextSpeed.rotation / sD * odometryScale.rotation;
    this->speed.translation = nextSpeed.translation / sD; //no odometry scale, because its automaticly controlled
    this->getKickFootOffset = std::function<Pose3f(float)>();

    ASSERT(std::isfinite(this->speed.translation.x()) && std::isfinite(this->speed.translation.y()) &&  std::isfinite(this->speed.rotation));
  }
}
