/**
 * @file CalibrationEngine.cpp
 *
 * This module lets the robot stand on one leg and calibrates the offset between the support soles rotation and the torso rotation.
 * We assume, that in an ideal world, where everything is calibrated perfectly, both rotations in the x- and y-axis should be the same.
 * In reality this is not the case. Therefore the offset is determined (2 times, to calculate an average) and used by the
 * InertialDataProvider. This modules determines the torso rotation based on the IMU values. With the calibrated offsets we
 * can improve the torso rotation estimate whenever a new step is started. In such an event the support sole rotation can be used
 * as a kind of "ground truth" rotation.
 *
 * @author Philip Reichenberg
 */

#include "CalibrationEngine.h"
#include "Debugging/Annotation.h"
#include "Debugging/DebugDrawings3D.h"
#include "Platform/File.h"
#include "Math/Eigen.h"
#include "Math/Rotation.h"

MAKE_MODULE(CalibrationEngine, motionControl);

CalibrationEngine::CalibrationEngine()
{
  calcDefaultValues();
  calcStandingOnOneLeftPosition(true, false);
  calcStandingOnOneLeftPosition(false, false);
  calcStandingOnOneLeftPosition(true, true);
  calcStandingOnOneLeftPosition(false, true);

  footStable[Legs::left].left.y() += 5.f;
  footStable[Legs::right].right.y() -= 5.f;

  InMapFile stream("footSoleRotationCalibration.cfg");
  ASSERT(stream.exists());
  stream >> const_cast<FootSoleRotationCalibration&>(theFootSoleRotationCalibration);
  this->notCalibratable = theFootSoleRotationCalibration.notCalibratable;
}

void CalibrationEngine::update(FootSoleRotationCalibration& footSoleRotationCalibration)
{
  footSoleRotationCalibration.notCalibratable = notCalibratable;
  footSoleRotationCalibration.calibrateSoleRotation = [this, &footSoleRotationCalibration]
  {
    const Legs::Leg leg = theFootSupport.support > 0.f ? Legs::left : Legs::right;
    if(!footSoleRotationCalibration.footCalibration[leg].isCalibrated)
    {
      const InertialSensorData& data = useInertialSensorData ? theInertialSensorData : theInertialData;
      // Calibrate the rotation offset with quaternions, because we need to subtract the goal rotation (InertialData) and the measured one (sole rotation)
      const Quaternionf quatIMU = Rotation::Euler::fromAngles(data.angle.x(), data.angle.y(), 0.f);
      const Quaternionf quatFeet((leg == Legs::left ? theRobotModel.soleLeft : theRobotModel.soleRight).rotation);
      const Quaternionf quatDiff = quatIMU * quatFeet.inverse();
      footSoleRotationCalibration.footCalibration[leg].rotationOffset *= static_cast<float>(footSoleRotationCalibration.id);
      footSoleRotationCalibration.footCalibration[leg].rotationOffset += Rotation::AngleAxis::pack(AngleAxisf(quatDiff)).head<2>().cast<Angle>();
      footSoleRotationCalibration.footCalibration[leg].rotationOffset /= static_cast<float>(footSoleRotationCalibration.id + 1);
      numberOfCalibrations[leg] += 1;
      footSoleRotationCalibration.footCalibration[leg].isCalibrated = numberOfCalibrations[leg] >= theCalibrationRequest.numOfFootSoleCalibration;
      if(leg == Legs::right)
        footSoleRotationCalibration.id += 1;
    }
  };
}

void CalibrationEngine::update(CalibrationGenerator& calibrationGenerator)
{
  DECLARE_DEBUG_DRAWING3D("module:CalibrationEngine:CheckWalkArea", "robot");
  calibrationGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    return std::make_unique<CalibrationPhase>(*this);
  };
}

void CalibrationEngine::calcDefaultValues()
{
  leftFootDefaultPosition = Pose3f(xTranslation, theRobotDimensions.yHipOffset, -feetHeight);
  rightFootDefaultPosition = Pose3f(xTranslation, -theRobotDimensions.yHipOffset, -feetHeight);
  leftFootOneLegDefaultPosition = Pose3f(xTranslation, theRobotDimensions.yHipOffset, -feetHeight + liftOffset);
  rightFootOneLegDefaultPosition = Pose3f(xTranslation, -theRobotDimensions.yHipOffset, -feetHeight + liftOffset);
  static_cast<void>(InverseKinematic::calcLegJoints(leftFootDefaultPosition, rightFootDefaultPosition, Vector2f::Zero(), defaultJointRequest, theRobotDimensions));
  defaultJointRequest.stiffnessData.stiffnesses[Joints::headPitch] = headStiffness;
  defaultJointRequest.stiffnessData.stiffnesses[Joints::headYaw] = headStiffness;
  for(std::size_t i = Joints::firstArmJoint; i < Joints::firstLegJoint; i++)
  {
    defaultJointRequest.stiffnessData.stiffnesses[i] = armStiffness;
    defaultJointRequest.angles[i] = 0;
  }
  for(std::size_t i = Joints::firstLegJoint; i < Joints::numOfJoints; i++)
    defaultJointRequest.stiffnessData.stiffnesses[i] = legStiffness;
  defaultJointRequest.angles[Joints::lShoulderPitch] = defaultJointRequest.angles[Joints::rShoulderPitch] = 90_deg;
  defaultJointRequest.angles[Joints::rShoulderRoll] = -7_deg;
  defaultJointRequest.angles[Joints::lShoulderRoll] = 7_deg;
  defaultJointRequest.angles[Joints::lWristYaw] = -90_deg;
  defaultJointRequest.angles[Joints::rWristYaw] = 90_deg;
  defaultJointRequest.angles[Joints::headYaw] = 0_deg;
  defaultJointRequest.angles[Joints::headPitch] = headPitch;
}

void CalibrationEngine::calcStandingOnOneLeftPosition(const bool isLeft, const bool isOneLeg)
{
  Pose3f stableLeft;
  Pose3f stableRight;

  stableLeft = leftFootDefaultPosition;
  stableRight = rightFootDefaultPosition;

  if(isOneLeg && isLeft)
    stableRight = rightFootOneLegDefaultPosition;
  else if(isOneLeg && !isLeft)
    stableLeft = leftFootOneLegDefaultPosition;

  JointAngles jointAngles = defaultJointRequest;
  // Find a good default position based on the physics of the robot
  for(int i = 0; i < 10; i++)
  {
    static_cast<void>(InverseKinematic::calcLegJoints(stableLeft, stableRight, Vector2f::Zero(), jointAngles, theRobotDimensions));
    const RobotModel model(jointAngles, theRobotDimensions, theMassCalibration);
    const float comPositionDiff = model.centerOfMass.y() - (isLeft ? model.soleLeft : model.soleRight).translation.y();
    stableLeft.translation.y() += comPositionDiff * searchForIdealRefDefaultPosFactor;
    stableRight.translation.y() += comPositionDiff * searchForIdealRefDefaultPosFactor;
  }
  const StableFeetPair feetPair(stableLeft.translation.head<2>(), stableRight.translation.head<2>());
  const Legs::Leg leg = isLeft ? Legs::left : Legs::right;
  if(isOneLeg)
    footStableOneLeg[leg] = feetPair;
  else
    footStable[leg] = feetPair;
}

CalibrationPhase::CalibrationPhase(CalibrationEngine& engine) :
  MotionPhase(MotionPhase::calibration),
  engine(engine)
{
  leftFoot = engine.leftFootDefaultPosition;
  rightFoot = engine.rightFootDefaultPosition;
  jointRequestOutput = engine.defaultJointRequest;
  startRequest = engine.theJointRequest;
  goalRequest.angles = startRequest.angles;
  FOREACH_ENUM(Joints::Joint, joint)
    startRequest.angles[joint] = engine.theJointRequest.stiffnessData.stiffnesses[joint] == 0 ? engine.theJointAngles.angles[joint] : startRequest.angles[joint];
  update();
}

void CalibrationPhase::update()
{
  comInTorso = engine.theTorsoMatrix * engine.theRobotModel.centerOfMass;
  leftFootInTorso = (engine.theTorsoMatrix * engine.theRobotModel.soleLeft).translation;
  rightFootInTorso = (engine.theTorsoMatrix * engine.theRobotModel.soleRight).translation;
  checkForWalkStep();
}

bool CalibrationPhase::isDone(const MotionRequest& motionRequest) const
{
  return (engine.theFootSoleRotationCalibration.footCalibration[Legs::left].isCalibrated
          && engine.theFootSoleRotationCalibration.footCalibration[Legs::right].isCalibrated)
         || startWalking || motionRequest.motion != MotionRequest::calibration;
}

void CalibrationPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo&)
{
  // Execute behavior
  beginFrame(engine.theFrameInfo.time);
  execute(OptionInfos::getOption("Root"));
  endFrame();
  jointRequest = jointRequestOutput;
}

std::unique_ptr<MotionPhase> CalibrationPhase::createNextPhase(const MotionPhase&) const
{
  if(state == Idle)
    return std::unique_ptr<MotionPhase>();
  return engine.theWalkGenerator.createPhase(Pose2f(0, 0.1f, 0.f), *this);
}

void CalibrationPhase::setFeet(const float stateTime)
{
  switch(state)
  {
    case Idle:
    {
      static_cast<void>(InverseKinematic::calcLegJoints(engine.leftFootDefaultPosition, engine.rightFootDefaultPosition, Vector2f::Zero(), jointRequestOutput, engine.theRobotDimensions));
      break;
    }
    case Reset:
    case LeftCalibration:
    case RightCalibration:
    case LeftOneStand:
    case RightOneStand:
    {
      const float interpolatationTime = state == Reset ? engine.waitDuration : engine.interpolateOneLegTime;
      float ratio = Rangef::ZeroOneRange().limit(stateTime / interpolatationTime);
      ratio = (std::sin(-Constants::pi_2 + Constants::pi * ratio) + 1.f) / 2.f;
      MotionUtilities::interpolate(startRequest, goalRequest, ratio, jointRequestOutput, engine.theJointAngles);
      break;
    }
    default:
    {
      // iterpolate back
      float ratio = Rangef::ZeroOneRange().limit(stateTime / engine.interpolateOneLegTime);
      ratio = (std::sin(-Constants::pi_2 + Constants::pi2 * ratio) + 1.f) / 2.f;
      MotionUtilities::interpolate(startRequest, engine.defaultJointRequest, ratio, jointRequestOutput, engine.theJointAngles);
      break;
    }
  }
  addGyroBalance();
}

void CalibrationPhase::addGyroBalance()
{
  jointRequestOutput.angles[Joints::lAnkleRoll] += engine.theInertialData.gyro.x() * 0.02f;
  jointRequestOutput.angles[Joints::rAnkleRoll] += engine.theInertialData.gyro.x() * 0.02f;
  jointRequestOutput.angles[Joints::lHipRoll] += engine.theInertialData.gyro.x() * 0.01f;
  jointRequestOutput.angles[Joints::rHipRoll] += engine.theInertialData.gyro.x() * 0.01f;

  jointRequestOutput.angles[Joints::lAnklePitch] += engine.theInertialData.gyro.y() * 0.02f;
  jointRequestOutput.angles[Joints::rAnklePitch] += engine.theInertialData.gyro.y() * 0.02f;
  jointRequestOutput.angles[Joints::lHipPitch] += engine.theInertialData.gyro.y() * 0.01f;
  jointRequestOutput.angles[Joints::rHipPitch] += engine.theInertialData.gyro.y() * 0.01f;
}

void CalibrationPhase::moveCOMToMiddleOfFoot(const bool isLeft)
{
  Pose3f leftFoot = engine.leftFootDefaultPosition;
  Pose3f rightFoot = engine.rightFootDefaultPosition;

  const Legs::Leg leg = isLeft ? Legs::left : Legs::right;
  leftFoot.translation.head<2>() =  engine.footStableOneLeg[leg].left;
  rightFoot.translation.head<2>() = engine.footStableOneLeg[leg].right;

  Vector3f comInFoot = comInTorso;
  comInFoot.z() = 0.f;
  comInFoot = engine.theTorsoMatrix.inverse() * comInFoot;

  Pose3f& support = isLeft ? leftFoot : rightFoot;
  Pose3f& swing = isLeft ? rightFoot : leftFoot;

  swing.translation.z() += engine.liftOffset;
  const Range minDiff(-3.f, 3.f);
  Vector2f diff = comInFoot.head<2>() - (support.translation.head<2>() + shiftToCom);
  if(!(minDiff.isInside(diff.x()) && minDiff.isInside(diff.y())))
  {
    engine.lastNotMoving = engine.theFrameInfo.time;
    const Rangef speed(-0.015f, 0.015f);
    shiftToCom.x() += speed.limit(diff.x());
    shiftToCom.y() += speed.limit(diff.y());
  }
  else if(std::abs(engine.theInertialData.gyro.x()) > 0.5_deg || std::abs(engine.theInertialData.gyro.y()) > 0.5_deg)
    engine.lastNotMoving = engine.theFrameInfo.time;

  support.translation += Vector3f(shiftToCom.x(), shiftToCom.y(), 0.f);
  static_cast<void>(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), goalRequest, engine.theRobotDimensions));
  jointRequestOutput = goalRequest;
  addGyroBalance();
}

void CalibrationPhase::calcGoalRequest(const bool isLeft, const bool liftLeg)
{
  engine.lastNotMoving = engine.theFrameInfo.time;
  shiftToCom = Vector2f::Zero();
  startRequest = jointRequestOutput;
  Pose3f leftFoot = engine.leftFootDefaultPosition;
  Pose3f rightFoot = engine.rightFootDefaultPosition;

  const Legs::Leg leg = isLeft ? Legs::left : Legs::right;
  leftFoot.translation.head<2>() = !liftLeg ? engine.footStable[leg].left : engine.footStableOneLeg[leg].left;
  rightFoot.translation.head<2>() = !liftLeg ? engine.footStable[leg].right : engine.footStableOneLeg[leg].right;
  if(liftLeg && isLeft)
    rightFoot.translation.z() += engine.liftOffset;
  if(liftLeg && !isLeft)
    leftFoot.translation.z() += engine.liftOffset;

  static_cast<void>(InverseKinematic::calcLegJoints(leftFoot, rightFoot, Vector2f::Zero(), goalRequest, engine.theRobotDimensions));
}

void CalibrationPhase::checkForWalkStep()
{
  const Vector3f& leftSide = state < LeftOneStand ? leftFootInTorso : (state == LeftOneStand ? leftFootInTorso : rightFootInTorso);
  const Vector3f& rightSide = state < LeftOneStand ? rightFootInTorso : (state == LeftOneStand ? leftFootInTorso : rightFootInTorso);
  const Rangef& xDevi = state == Idle ? engine.xMaxCOMDeviationStanding : engine.xMaxCOMDeviation;
  const Rangef& yDevi = state == Idle ? engine.yMaxCOMDeviationStanding : engine.yMaxCOMDeviation;
  const Rangef xRange(leftSide.x() + xDevi.min, rightSide.x() + xDevi.max);
  const Rangef yRange(rightSide.y() + yDevi.min, leftSide.y() + yDevi.max);
  startWalking = !(xRange.isInside(comInTorso.x()) && yRange.isInside(comInTorso.y()));

  if(startWalking && state >= LeftCalibration)
  {
    Legs::Leg leg = state == LeftCalibration || state == LeftOneStand ? Legs::left : Legs::right;
    StableFeetPair& feetPair = state >= LeftOneStand ? engine.footStableOneLeg[leg] : engine.footStable[leg];
    float& base = engine.calibrateStableFeetPoseBaseFactor;
    const Rangef& maxAdjust = engine.calibrateStableFeetPose;
    const float xAdjustment = maxAdjust.limit((comInTorso.x() - xRange.limit(comInTorso.x())) * base);
    const float yAdjustment = maxAdjust.limit((comInTorso.y() - yRange.limit(comInTorso.x())) * base);
    feetPair.left.x() += xAdjustment;
    feetPair.left.y() += yAdjustment;
    feetPair.right.x() += xAdjustment;
    feetPair.right.y() += yAdjustment;
    base *= engine.calibrateStableFeetPoseFactor;
    OUTPUT_TEXT("Adjusted Stable Feet Position: x with " << xAdjustment << " and y with " << yAdjustment);
  }

  DEBUG_DRAWING3D("module:CalibrationEngine:CheckWalkArea", "robot")
  {
    Vector3f topLeft = leftSide;
    Vector3f bottomLeft = leftSide;
    topLeft.x() += xDevi.max;
    topLeft.y() += yDevi.max;
    bottomLeft.x() += xDevi.min;
    bottomLeft.y() += yDevi.max;
    Vector3f topRight = rightSide;
    Vector3f bottomRight = rightSide;
    topRight.x() += xDevi.max;
    topRight.y() += yDevi.min;
    bottomRight.x() += xDevi.min;
    bottomRight.y() += yDevi.min;

    const Vector3f topLeftInRobot = engine.theTorsoMatrix.inverse() * topLeft;
    const Vector3f bottomLeftInRobot = engine.theTorsoMatrix.inverse() * bottomLeft;
    const Vector3f topRightInRobot = engine.theTorsoMatrix.inverse() * topRight;
    const Vector3f bottomRightInRobot = engine.theTorsoMatrix.inverse() * bottomRight;

    Vector3f comInFoot = comInTorso;
    comInFoot.z() = 0.f;
    comInFoot = engine.theTorsoMatrix.inverse() * comInFoot;

    LINE3D("module:CalibrationEngine:CheckWalkArea", topLeftInRobot.x(), topLeftInRobot.y(), topLeftInRobot.z(), bottomLeftInRobot.x(), bottomLeftInRobot.y(), bottomLeftInRobot.z(), 10, ColorRGBA::blue);
    LINE3D("module:CalibrationEngine:CheckWalkArea", bottomRightInRobot.x(), bottomRightInRobot.y(), bottomRightInRobot.z(), bottomLeftInRobot.x(), bottomLeftInRobot.y(), bottomLeftInRobot.z(), 10, ColorRGBA::blue);
    LINE3D("module:CalibrationEngine:CheckWalkArea", topLeftInRobot.x(), topLeftInRobot.y(), topLeftInRobot.z(), topRightInRobot.x(), topRightInRobot.y(), topRightInRobot.z(), 10, ColorRGBA::blue);
    LINE3D("module:CalibrationEngine:CheckWalkArea", topRightInRobot.x(), topRightInRobot.y(), topRightInRobot.z(), bottomRightInRobot.x(), bottomRightInRobot.y(), bottomRightInRobot.z(), 10, ColorRGBA::blue);
    POINT3D("module:CalibrationEngine:CheckWalkArea", comInFoot.x(), comInFoot.y(), comInFoot.z(), 10, ColorRGBA::red);
  }
}

bool CalibrationPhase::checkFSR() const
{
  const Legs::Leg leg = state == LeftOneStand ? Legs::left : Legs::right;
  const unsigned int hasPressure = engine.theFsrData.legInfo[leg].hasPressure;
  return (engine.theFsrData.legInfo[leg].forwardPressure == hasPressure ||
          engine.theFsrData.legInfo[leg].backwardPressure == hasPressure) &&
         (engine.theFsrData.legInfo[leg].leftPressure == hasPressure &&
          engine.theFsrData.legInfo[leg].rightPressure == hasPressure) &&
         hasPressure == engine.theFsrData.lastUpdateTimestamp;
}
