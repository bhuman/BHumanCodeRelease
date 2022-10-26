/**
 * @file CalibrationEngine.h
 *
 * This module lets the robot stand on one leg and calibrates the offset between the support soles rotation and the torso rotation.
 * We assume, that in an ideal world, where everything is calibrated perfectly, both rotations in the x- and y-axis should be the same.
 * In reality this is not the case. Therefore the offset is determined (2 times, to calculate an average) and used by the
 * InertialDataProvider. This modules determines the torso rotation based on the IMU values. With the calibrated offsets we
 * can improve the torso rotation estimate whenever a new step is started. In such an event the support sole rotation can be used
 * as a kind of "ground truth" rotation.
 *
 * TODO: The behavior should always start the calibration (and ignore the isCalibrated flag)
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/CalibrationGenerator.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/FsrData.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Cabsl.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Framework/Module.h"

STREAMABLE(StableFeetPair,
{
  StableFeetPair() = default;
  StableFeetPair(Vector2f leftIn, Vector2f rightIn),

  (Vector2f) left,
  (Vector2f) right,
});

inline StableFeetPair::StableFeetPair(Vector2f leftIn, Vector2f rightIn) : left(std::move(leftIn)), right(std::move(rightIn)) {};

MODULE(CalibrationEngine,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(FootSupport),
  REQUIRES(FsrData),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(InertialSensorData),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(TorsoMatrix),
  USES(WalkGenerator),
  PROVIDES(FootSoleRotationCalibration),
  REQUIRES(FootSoleRotationCalibration),
  PROVIDES(CalibrationGenerator),
  DEFINES_PARAMETERS(
  {,
    (float)(0.f) xTranslation,
    (float)(230.f) feetHeight,
    (int)(100) legStiffness,
    (int)(10) armStiffness,
    (int)(50) headStiffness,
    (Angle)(10_deg) headPitch,
    (float)(5000.f) interpolateOneLegTime,
    (float)(2000.f) waitDuration,
    (Rangef)(Rangef(-10.f, 10.f)) slowRange, /**< If difference to goal target is inside this range, the interpolation speed is reduced. */
    (Rangef)(Rangef(-20.f, 50.f)) xMaxCOMDeviation, /**< If COM is deviates over <Footcenter.x + threshold>, then execute a walk step, to prevent falling. */
    (Rangef)(Rangef(-20.f, 20.f)) yMaxCOMDeviation, /**< If COM is deviates over <Footcenter.y + threshold>, then execute a walk step, to prevent falling. */
    (Rangef)(Rangef(-30.f, 40.f)) xMaxCOMDeviationStanding, /**< When standing, the com should stay in the middle, otherwise execute a walk step. */
    (Rangef)(Rangef(-0.f, 0.f)) yMaxCOMDeviationStanding, /**< When standing, the com should stay in the middle, otherwise execute a walk step. */
    (Rangef)(Rangef(-10.f, 10.f)) desiredPosDeviation, /**< After interpolating to the pose for standing on one leg, the com may deviate by this value. */
    (Rangef)(Rangef(-5.f, 5.f)) calibrateStableFeetPose, /**< In case the robot needed to abort the calibrate, calibrate the stable feet poses. */
    (float)(0.9f) calibrateStableFeetPoseFactor, /**< Factor to reduce calibrateStableFeetPose over time. */
    (float)(1.f) calibrateStableFeetPoseBaseFactor, /**< Factor to reduce calibrateStableFeetPose. */
    (float)(0.7f) searchForIdealRefDefaultPosFactor, /**< Factor for searching the correct feet position for standing on one leg. */
    (float)(13.f) liftOffset, /**< Lift leg by this amount (in mm). */
    (bool)(false) useInertialSensorData, /**< Use the InertialSensorData or the InertialData? */
  }),
});

class CalibrationEngine : public CalibrationEngineBase
{
public:

  CalibrationEngine();

  void update(CalibrationGenerator& calibrationGenerator) override;
  void update(FootSoleRotationCalibration& footSoleRotationCalibration) override;

  bool notCalibratable = false;
  unsigned numberOfCalibrations[Legs::numOfLegs] = {0, 0};
  unsigned lastNotMoving = 0;

  ENUM_INDEXED_ARRAY(StableFeetPair, Legs::Leg) footStable;
  ENUM_INDEXED_ARRAY(StableFeetPair, Legs::Leg) footStableOneLeg;
  float leftOffset = 0.f;
  float rightOffset = 0.f;
  JointRequest defaultJointRequest;
  Pose3f leftFootDefaultPosition;
  Pose3f rightFootDefaultPosition;
  Pose3f leftFootOneLegDefaultPosition;
  Pose3f rightFootOneLegDefaultPosition;

private:
  void calcStandingOnOneLeftPosition(const bool isLeft, const bool isOneLeg);
  void calcDefaultValues();
};

struct CalibrationPhase : MotionPhase, public Cabsl<CalibrationPhase>
{
  explicit CalibrationPhase(CalibrationEngine& engine);

private:
  void update() override;
  bool isDone(const MotionRequest& motionRequest) const override;
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;
  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultPhase) const override;

  void setFeet(const float stateTime);
  void calcGoalRequest(const bool isLeft, const bool liftLeg);
  void checkForWalkStep();
  bool checkFSR() const;

  void moveCOMToMiddleOfFoot(const bool isLeft);
  void addGyroBalance();

  bool startWalking = false;

  JointAngles startRequest;
  JointRequest jointRequestOutput;
  JointRequest goalRequest;

  Pose3f leftFoot;
  Pose3f rightFoot;

  Vector2f shiftToCom;

  ENUM(CalibrationState,
  {,
    Idle,
    Reset,
    Calibrating,
    LeftCalibration,
    RightCalibration,
    LeftOneStand,
    RightOneStand,
  });
  CalibrationState state = Idle;
  Vector3f comInTorso;
  Vector3f leftFootInTorso;
  Vector3f rightFootInTorso;

  CalibrationEngine& engine;

  option(Root)
  {
    initial_state(initial)
    {
      transition
      {
        if(state_time > engine.waitDuration)
        {
          jointRequestOutput = engine.defaultJointRequest;
          goto waiting;
        }
      }
      action
      {
        if(state_time == 0)
          startRequest = goalRequest;
        state = Reset;
        goalRequest = engine.defaultJointRequest;
        setFeet(static_cast<float>(state_time));;
      }
    }

    state(waiting)
    {
      transition
      {
        goto calibrateNextFoot;
      }
      action
      {
        leftFoot = engine.leftFootDefaultPosition;
        rightFoot = engine.rightFootDefaultPosition;
      }
    }

    state(calibrateNextFoot)
    {
      transition
      {
        if(state_time > 100)
        {
          if(!engine.theFootSoleRotationCalibration.footCalibration[Legs::left].isCalibrated && engine.numberOfCalibrations[Legs::left] <= engine.numberOfCalibrations[Legs::right])
          {
            calcGoalRequest(true, false);
            goto calibrateLeft;
          }
          if(!engine.theFootSoleRotationCalibration.footCalibration[Legs::right].isCalibrated)
          {
            calcGoalRequest(false, false);
            goto calibrateRight;
          }
        }
      }
      action
      {
      }
    }

    state(calibrateLeft)
    {
      transition
      {
        if(state_time > engine.interpolateOneLegTime)
        {
          goto liftLeftLeg;
        }
      }
      action
      {
        state = LeftCalibration;
        setFeet(static_cast<float>(state_time));
      }
    }

    state(calibrateRight)
    {
      transition
      {
        if(state_time > engine.interpolateOneLegTime)
        {
          goto liftRightLeg;
        }
      }
      action
      {
        state = RightCalibration;
        setFeet(static_cast<float>(state_time));
      }
    }

    state(liftLeftLeg)
    {
      transition
      {
        if(state_time > engine.interpolateOneLegTime)
          goto calibrateRotation;
      }
      action
      {
        state = LeftOneStand;
        if(state_time == 0)
        {
          calcGoalRequest(true, true);
        }
        setFeet(static_cast<float>(state_time));
      }
    }

    state(liftRightLeg)
    {
      transition
      {
        if(state_time > engine.interpolateOneLegTime)
          goto calibrateRotation;
      }
      action
      {
        state = RightOneStand;
        if(state_time == 0)
        {
          calcGoalRequest(false, true);
        }
        setFeet(static_cast<float>(state_time));
      }
    }

    state(calibrateRotation)
    {
      const Legs::Leg leg = state == LeftOneStand ? Legs::left : Legs::right;
      transition
      {
        if(engine.theFootSoleRotationCalibration.footCalibration[leg].isCalibrated ||
           !(((leg == Legs::left && engine.numberOfCalibrations[Legs::left] <= engine.numberOfCalibrations[Legs::right])) ||
             (leg == Legs::right && engine.numberOfCalibrations[Legs::right] < engine.numberOfCalibrations[Legs::left])))
          goto initial; // Calibration worked!
      }
      action
      {
        if(state_time > engine.waitDuration && checkFSR() && engine.theFrameInfo.getTimeSince(engine.lastNotMoving) > 500 &&
           (((leg == Legs::left && engine.numberOfCalibrations[Legs::left] <= engine.numberOfCalibrations[Legs::right])) ||
            (leg == Legs::right && engine.numberOfCalibrations[Legs::right] < engine.numberOfCalibrations[Legs::left])))
          engine.theFootSoleRotationCalibration.calibrateSoleRotation();
        moveCOMToMiddleOfFoot(state == LeftOneStand);
      }
    }
  }
};
