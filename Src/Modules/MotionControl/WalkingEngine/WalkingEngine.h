/**
 * @file WalkingEngine.h
 *
 * This file declares a module that generates walking motions.
 *
 * The basic ideas of using the FSRs to determine the support change and the gyroscope
 * for directly proportional feedback to the ankle joints are borrowed from the 2014
 * walking engine from rUNSWift written by Bernhard Hengst.
 *
 * @author Thomas Röfer
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Configuration/FootSoleRotationCalibration.h"
#include "Representations/Configuration/GlobalOptions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/WalkModifier.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Module/Module.h"
#include "Tools/Motion/WalkKickStep.h"
#include "WalkStepAdjustment.h"

STREAMABLE(WalkingEngineCommon,
{,
  (Vector2f) maxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Pose2f) slowMaxSpeed, /**< Maximum speeds in mm/s and degrees/s. Slower for demo games. */
  (float) slowMaxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. Slower for demo games. */
  (Vector2f) slowMaxAcceleration, /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). Slower for demo games. */
  (float) insideTurnRatio, /**< The ratio of turn that can be done in the inner step of a double-step. */
  (Vector2a) reduceTranslationFromRotation, /**< The rotation from which the translation per step will be reduced. */
  (Vector2a) noTranslationFromRotation, /**< The rotation from which no translation is possible at all. */
  (Angle) noTranslationYFromRotationFastInner, /**< When walking fast with a rotation outwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) noTranslationYFromRotationFastOuter, /**< When walking fast with a rotation inwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) reduceTranslationYFromRotationFast, /**< When walking fast, the rotation from which the translation per step will be reduced is higher for the y translation. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (float) minXBackwardTranslationFast,/**< The backward step size has a minimum for the fast translation polygon. */
  (Rangef) supportSwitchPhaseRange, /**< In which range of the walk phase can the support foot change? */
  (float) emergencyStepSize, /**< The size of emergency sideways steps in mm. */
  (float) emergencyStepHeightFactor, /** During an emergency step, the step height is multiplied by this factor. */
  (int) emergencyNotMovingCounter, /** Gyro state measures low deviation and mean values so many times in a row, to start a new step early. */
  (Angle) emergencyMaxGyroMean, /** Gyro state must measure gyro values below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxGyroDeviation, /** Gyro state must measure gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxZGyroDeviation, /** Gyro state must measure z gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
  (Pose2f) odometryScale, /**< Scale measured speeds so that they match the executed speeds. */
  (int) walkStiffness, /**< Joint stiffness while walking in %. */
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (float) comTiltFactor, /**< Factor between the correct torso shift and the one actually used. Used to tilt the torso to compensate the arm positions in high stand. */
  (int) standStiffnessDelay, /**< The time in stand before the stiffness is lowered (in ms). */
  (bool) useFootSupportSwitchPrediction, /**< Use predicted support foot switches. */
  (Angle) turnThresholdFootSupportPrediction, /**< Use predicted support foot switches as long as the request turn speed is lower than this threshold. */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (Rangef) desiredFootArea, /**< In which area of the feet can the com move, before the feet must be adjusted to ensure stability (in %)? */
  (float) hipBalanceBackwardFootArea, /**< In which area of the feet can the com move, while the hip pitch is used to balancing when walking unstable. */
  (float) unstableWalkThreshold, /**< Thresholds for the lastLeftAdjustment and lastRightAdjustment values to trigger unstable sound. */
  (int) hipBalanceSteps,  /**< Threshold counter, after so many steps, the hip is no longer used for balancing. */
  (float) maxVelForward, /**< How fast are the feet allowed to move forward, when they get adjusted to ensure stability (in mm/s)? Should be higher than forward maxSpeed / 2. */
  (float) minVelForward, /**< Min additional feet speed, that the walk adjustment can make in both directions (in mm/s). */
  (float) removeAdjustmentVel, /**< The walk adjustment can be removed with this extra speed (in mm/s). */
  (float) comLowPassRatio, /**< To which ratio keep old com measurement? */
  (Rangef) clipForward, /**< Clip forward/backward feet movement, to prevent assert in the inverse kinematic calculation (and to prevent damage). */
  (int) lowStiffnessDelay, /**< Low stiffness delay after high stand interpolation started. */
  (float) fastFeetAdjustment, /**< Feet can move this fast to adjust to a specific pose. */
  (float) slowFeetAdjustment, /**< Feet shall move this slow to adjust to a specific pose. */
  (float) clipAtBallDistanceX, /**< Step adjustment is not allowed to touch the ball. */
  (float) clipAtBallDistance, /**< Ball is ignored when it further away than this threshold. */
  (float) afterKickFeetHeightAdjustment, /**< Feet height can change this much after a specific kick. */
  (Angle) standHighTorsoPitch, /**< The torso pitch angle for standHigh. */
  (Angle) standInterpolationVelocity, /**< The interpolation speed to interpolate to stand (in degree/s). */
  (int) standHighInterpolationDuration, /**< The duration to interpolate from/to stand to/from stand high. */
  (int) lowStiffnessLegs, /**< Low stiffness setting for leg joints (except AnklePitch). */
  (int) lowStiffnessAnklePitch, /** < Low stiffness setting for ankle pitch joints. */
  (Angle) soleRotationOffsetSpeed, /**< Sole rotation can return to neutral position with this speed (degree/s). */
  (float) armInterpolationTime, /**< Interpolation time for the arms, when the WalkPhase can set them again. */
  (int) standHighNotMovingCounter, /**< Robot must be standing still for so many checks in a row, to allow high stand. */
  (float) reduceSwingHeightStartingFactor, /**< Reduce max swing foot height for first walking step. */
  (float) stretchSwingHeightAfterThisOvertime, /**< After so much time after the planned step duration, start to stretch out the swing foot (in s). */
  (float) stretchSwingHeightValue, /**< Stretch swing foot maximal for this much (in mm). */
  (float) stretchSwingHeightSpeed, /**< Stretch swing foot with this speed (in mm/s). */
  (Rangef) translationPolygonSafeRange, /**< For the calculation of the translation polygon, the feet are adjusted by this amount in x translation,
                                        to ensure a possible pose even when the arms are on the back or the feet are balanced. */
  (Vector2f) gyroForwardBalanceFactorHipPitch, /**< Reduce factor of gyroForwardBalanceFactor, to add gyro balancing on the hipPitch of the support foot. x = negative values, y = positiv values. */
  (int) noFastTranslationPolygonStepsNumber, /**< Number of steps before big steps near the target are allowed. */
  (Angle) soleCompensationMinTorsoRotation, /**< The torso must have a minimum of this rotation in y axis, to allow the swing sole rotation compensation in y axis. */
  (Angle) soleCompensationMinSupportSoleRotation, /**< The support sole rotation must have a minimum of this rotation in y axis, to allow the swing sole rotation compensation in y axis. */
  (float) soleBackwardsCompensationTorsoFactor,
  (float) soleBackwardsCompensationReturnZeroRatio,
  (float) soleForwardCompensationReturnZeroRation,
  (float) soleBackwardsCompensationFeetXDifference,
  (float) soleBackwardsCompensationFeetShift,
  (float) soleCompensationReduction,
  (float) soleCompensationIncreasement,

  (float) walkSpeedReductionFactor, /**< Reduce the walking speed by this amount (only x translation). */
  (int) reduceWalkingSpeedTimeWindow, /**< If the step adjustment adjusted the feet too much two separat times in this time duration, then reduce the walking speed. */
  (int) reduceWalkingSpeedStepAdjustmentSteps, /**< If the step adjustment adjusted the feet too much two separat times, then reduce the walking speed for this number of walking steps. */

  (float) gyroBalanceKneeBalanceFactor, /**< How much are gyro measurements added to hip and knee joint angles to compensate falling forwards while walking? */
  (Angle) maxGyroBalanceKneeValue, /**< Max balance value when balancing the with knee and hip pitch. */
  (float) gyroBalanceKneeOvertimeInterpolation, /**< After this much overtime relative to the step duration, start the hip and knee pitch gyro balancing. */
  (Angle) gyroBalanceKneeNegativeGyroAbort, /**< Stop knee and hip pitch balancing when gyro measures less than this value. */
  (float) unstableBackWalkThreshold, /**< When last max step adjustment was lower than this value, start the gyro balancing with the knee and hip pitch. */

  (float) maxObstacleDistance, /**< An obstacle must stand near us to allow the check, if we stepped on another robots foot (in mm). */
  (float) minFeetHeightDifference, /**< Feet height difference sample must be lower than this value (in mm). */
  (float) maxFeetHeightDifferenceVelocity, /**< Feet height difference sample are allowed to change this much from the first to the last one (in mm). */
  (float) maxFeetHeightDifferenceScaling, /**< Scale the allowed change of the samples the higher the first sample was. */
  (float) maxLastBackwardStepAdjustment, /**< Last walking step adjustment must be lower than this value. */
  (unsigned) feetHeightDifferenceNumberOfSamples, /**< Number of samples. */
  (float) stepSizeXAfterWalkingOnOpponentFeet, /**< After a detected step on another robots feet, execute a walking step with this step size in the x-translation. */
  (float) maxSupportSwitchPhaseRangeAfterSteppingOnOpponentFeet, /**< Execute stepSizeXAfterWalkingOnOpponentFeet if the last step duration was below this ratio. */
  (bool) useSteppingOnOpponentFootBehavior, /**< Use detection for walking on other robots feet? */

  (bool) blockStoppingWithStepAdjustment,
});

MODULE(WalkingEngine,
{,
  REQUIRES(CalibrationRequest),
  REQUIRES(EnergySaving),
  REQUIRES(FootOffset),
  REQUIRES(FootSoleRotationCalibration),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GlobalOptions),
  REQUIRES(GroundContactState),
  REQUIRES(GyroState),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  USES(OdometryData),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(StiffnessSettings),
  REQUIRES(TorsoMatrix),
  USES(WalkKickGenerator),
  REQUIRES(WalkLearner),
  REQUIRES(WalkModifier),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
    (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
    (float) baseWalkPeriod, /**< Duration of a single step, i.e. half of a walk cycle (in ms). */
    (float) sidewaysWalkHeightPeriodIncreaseFactor, /**< Additional duration for the swing height, when walking sideways at maximum speed (in ms). */
    (float) sidewaysHipShiftFactor, /**< Apply the requested sidewards target with this factor on the support foot. */
    (float) walkHipHeight, /**< Walk hip height above ankle joint in mm - seems to work from 200 mm to 235 mm. */
    (float) baseFootLift, /**< Base foot lift in mm. */
    (float) torsoOffset, /**< The base forward offset of the torso relative to the ankles in mm. */
    (float) gyroLowPassRatio, /**< To which ratio keep old gyro measurements? */
    (float) gyroForwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling forwards while walking? */
    (float) gyroBackwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling backwards while walking? */
    (float) gyroSidewaysBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling sideways while standing? */
  }),
});

class WalkingEngine : public WalkingEngineBase, public WalkingEngineCommon
{
  void update(WalkStepData& walkStepData) override;
  void update(StandGenerator& standGenerator) override;
  void update(WalkGenerator& walkGenerator) override;
  void update(WalkingEngineOutput& walkingEngineOutput) override;

public:

  /**
   * Calculate the step duration based on the side translation amount of requested side speed
   * @param sideSpeed Next planned walk translation speed to the side
   */
  float stepDurationSpeedTarget(const float sideSpeed) const;

  /**
   * Calculate the side speed given the side target or the current step
   * @param sideTarget Next planned walk translation to the side
   */
  float getSideSpeed(const float sideTarget) const;

  /**
   * Calculate the current translation polygon
   * @param polygon the translation polygon
   * @param backRight the max allowed back and right translation
   * @param frontLeft the max allowed front and left translation
   */
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft);

  /**
   * Calculates the pose of the feet.
   * @param forwardL x translation position for the left foot, without torso shift
   * @param forwardR x translation position for the right foot, without torso shift
   * @param sideL y translation position for the left foot, without hip shift
   * @param sideR y translation position for the right foot, without hip shift
   * @param footHL foot height
   * @param footHR right foot height
   * @param turn z rotation
   * @param leftFoot left foot pose
   * @param rightFoot right foot pose
   */
  void calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR, const float footHL, const float footHR, const Angle turn, Pose3f& leftFoot, Pose3f& rightFoot);

  Angle filteredGyroX, /**< low-pass filtered gyro values. */
        filteredGyroY,
        lastFilteredGyroY;

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */

  MassCalibration lightMassCalibration; /**< MassCalibration without the legs masses. */

  /** The constructor loads the common parameters. */
  WalkingEngine();

  /** Register common parameters. */
  static void reg();

  static constexpr float mmPerM = 1000.f;
};

struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

struct WalkPhase : MotionPhase
{
  WalkPhase(const WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
            const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
            const WalkKickStep& walkKickStep = WalkKickStep());

  std::tuple<Pose2f, Pose2f> getLastFeetRequest();

public:
  std::vector<Vector2f> getTranslationPolygon();

private:

  void update() override;

  bool isDone(const MotionRequest& motionRequest) const override;

  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) override;

  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const override;

  unsigned freeLimbs() const override
  {
    return bit(MotionPhase::head) | bit(MotionPhase::leftArm) | bit(MotionPhase::rightArm);
  }

  /**
   * Calculate the current odometry.
   */
  Pose2f getOdometryOffset() const;

  /**
   * Calculate the translation and rotation variables for both feet while walking and standing.
   * @param swingSign Sign of the swing foot. 1 for left foot is the swing foot and -1 for the right foot is the swing foot.
   * @param ratio Current time in the step (in s)
   * @param duration Planned duration of the step (in s)
   * @param heightDuration Used duration for the step height (in s). Side steps use a longer step duration for the height interpolation.
   * @param forwardSwing0 Start value for the forward position of the swing foot.
   * @param forwardSupport0 Start value for the forward position of the support foot.
   * @param forwardSwing Current forward position of the swing foot.
   * @param forwardSupport Current forward position of the support foot.
   * @param sideSwing0 Start value for the side position of the swing foot.
   * @param sideSupport0 Start value for the side position of the support foot.
   * @param sideSwing Current side position of the swing foot.
   * @param sideSupport Current side position of the support foot.
   * @param footHeightSwing0 Start height of the swing foot.
   * @param footHeightSupport0 Start height of the support foot.
   * @param footHeightSwing Current height of the swing foot.
   * @param footHeightSupport Current height of the support foot.
   * @param turnVal Current rotation value of the feet.
   * @param useForwardSpeed Requested forward step size.
   * @param useSideSpeed Requested side step size.
   * @param useTurnSpeed Requested turn step size.
   */
  void calcFootOffsets(const float swingSign, const float ratio, const float duration, const float heightDuration,
                       const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                       float& forwardSupport, const float sideSwing0, const float sideSupport0,
                       float& sideSwing, float& sideSupport, float& footHeightSwing0,
                       float& footHeightSupport0, float& footHeightSwing,
                       float& footHeightSupport, Angle& turnVal,
                       const float useForwardSpeed, const float useSideSpeed, const Angle useTurnSpeed);

  /**
   * Calculate the translation and rotation variables for both feet kicking.
   * @param swingSign Sign of the swing foot. 1 for left foot is the swing foot and -1 for the right foot is the swing foot.
   * @param ratio Current time in the step (in s)
   * @param forwardSwing0 Start value for the forward position of the swing foot.
   * @param forwardSupport0 Start value for the forward position of the support foot.
   * @param forwardSwing Current forward position of the swing foot.
   * @param forwardSupport Current forward position of the support foot.
   * @param sideSwing0 Start value for the side position of the swing foot.
   * @param sideSupport0 Start value for the side position of the support foot.
   * @param sideSwing Current side position of the swing foot.
   * @param sideSupport Current side position of the support foot.
   * @param footHeightSwing0 Start height of the swing foot.
   * @param footHeightSupport0 Start height of the support foot.
   * @param footHeightSwing Current height of the swing foot.
   * @param footHeightSupport Current height of the support foot.
   * @param turnRL0 Start rotation value of the feet.
   * @param turnVal Current rotation value of the feet.
   * @param walkKickStep Describes all important parameters for the step, like multiple step sizes and their interpolation
   */
  void calcWalkKickFootOffsets(const float swingSign, const float ratio,
                               float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                               float& forwardSupport, float& sideSwing0, float& sideSupport0,
                               float& sideSwing, float& sideSupport, float& footHeightSwing0,
                               float& footHeightSupport0, float& footHeightSwing,
                               float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                               Angle& turnRL0);

  /**
   * Apply the step adjustment to keep the robot balanced.
   * @param leftFoot Requested left foot pose.
   * @param rightFoot Requested right foot pose.
   * @param jointRequest Requested joint positions.
   */
  void balanceFeetPoses(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest);

  /**
   * Apply all balancing based on the gyro measurement.
   * @param jointRequest Requested joint positions.
   */
  void addGyroBalance(JointRequest& jointRequest);

  /**
   * Returns values on a parabola with f(0) = f(1) = 0, f(0.5) = 1.
   * @param f A value between 0 and 1.
   * @return The value on the parabola for "f".
   */

  float parabolicFootHeight(float f) const;

  /**
   * Returns values on a parabola with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the parabola for "time".
   */

  float parabolicStep(float time, float period) const;

  /**
   * Calculates if the next step can be a standing phase.
   * @param forwardL x translation position for the left foot, without torso shift.
   * @param forwardR x translation position for the right foot, without torso shift.
   * @param sideL y translation position for the left foot, without hip shift.
   * @param sideR y translation position for the right foot, without hip shift.
   * @param turn z rotation.
   * @param balanceAdjustmentLeft Left foot step adjustment value.
   * @param balanceAdjustmentRight Reft foot step adjustment value.
   */
  bool isStandingPossible(float forwardL, float forwardR, float sideL, float sideR, float turnRL, float balanceAdjustmentLeft, float balanceAdjustmentRight) const;

  /**
   * Calculates the pose of the feet.
   * @param forwardL x translation position for the left foot, without torso shift
   * @param forwardR x translation position for the right foot, without torso shift
   * @param sideL y translation position for the left foot, without hip shift
   * @param sideR y translation position for the right foot, without hip shift
   * @param footHL foot height
   * @param footHR right foot height
   * @param turn z rotation
   * @param leftFoot left foot pose
   * @param rightFoot right foot pose
   */
  void calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR, const float footHL, const float footHR, const Angle turn, Pose3f& leftFoot, Pose3f& rightFoot);

  /**
   * Set the joint request for the arms, or use the given values
   * @param leftFoot Left foot pose.
   * @param rightFoot Right foot pose.
   * @param jointRequest The given arm positions, which may be overriden bei walkArms if they should be ignored.
   * @param walkArms The arm positions based on the current walk.
   */
  void setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms);

  /**
   * Compensates for the changed position of the COM resulting from arm motion.
   * The torso is tilted to move the COM.
   * @param leftFoot The pose of the left foot's sole relative to the torso.
   * @param rightFoot The pose of the right foot's sole relative to the torso.
   * @param jointRequest The joint request as determined by the walk generator.
   *                     The joint request is changed to compensate for the
   *                     effect of external arm movements.
   * @param walkArms The joint request with arms set by the walking engine.
   */
  void compensateArms(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest, const JointRequest& walkArms);

  /**
   * Checks if theGyroState updated and updates the robotIsNotMoving, so the robot might need to do a emergency step
   * @param ignoreXGyro if true, the x gyro value is ignored
   */
  void checkGyroState(const bool ignoreXGyro);

  /**
   * Apply joint offsets of the current in walk kick
   * @param jointRequest Current joint request.
   * @param time Current time of the step (in s).
   */
  void applyWalkKickLongKickOffset(JointRequest& jointRequest, const float time);

  /**
   * Override the feet position parameters based on the measured positions
   * @param isLeftPhase Is left foot the swing foot.
   * @param time Current time (in s).
   * @param duration Planned step duration (in s).
   * @param walkKickStep Walk kick parameters.
   */
  void adjustRequestByMeasuredPosition(const bool isLeftPhase, const float time, const float duration, const WalkKickStep& walkKickStep);

  /**
   * Detect if the last walking step stepped on the foot of another robot. If so, set stepHeightDuration to 0
   * and earlySupportSwitchAllowed to true and continue the walk step as normal.
   * This prevents the swing foot to reduce the height. Otherwise an early foot support change happens and the robot just falls over,
   * because of new support foot height is a lot lower than the old support foot one.
   */
  void detectWalkingOnOpponentFeet();

  /**
   * Calculate the ball position relative to the swing foot zero position
   */
  void calculateBallPosition();

  const WalkingEngine& engine;

  WalkStepAdjustment walkStepAdjustment;

  std::vector<float> supportSwingHeightDifference; /**< Height difference between swing and support foot. */
  float t = Constants::motionCycleTime; /**< Current time in the step (in s). */
  float stepDuration = 0.f; /**< Duration of the current step (in s). */
  float stepHeightDuration = 0.f; /**< Duration of the current step (in s). */
  bool isLeftPhase = false; /**< Is the left foot the swing foot? */
  bool earlySupportSwitchAllowed = false; /**< Allow an early switch of support and swing foot. */
  Pose2f step; /**< The step made in this phase (could be reconstructed from forwardStep, sideStep and turnStep). */
  float standInterpolationDuration; /**< The interpolation duration to interpolate into stand (in ms). */

  float forwardStep = 0.f; /**< Forward speed in m/step. Forward is positive. */
  float forwardL = 0.f; /**< The forward offset of the left foot (in m). */
  float forwardR = 0.f; /**< The forward offset of the right foot (in m). */
  float forwardL0 = 0.f; /**< Forward offset of the left foot when the support changed (in m). */
  float forwardR0 = 0.f; /**< Forward offset of the right foot when the support changed (in m). */

  float sideStep = 0.f; /**< Sideways speed in m/s. Left is positive. */
  float lastSideStep = 0.f; /**< Previous Sideways speed in m/s. Left is positive. */
  float sideL = 0.f; /**< The sideways offset of the left foot (in m). */
  float sideR = 0.f; /**< The sideways offset of the right foot (in m). */
  float sideL0 = 0.f; /**< Recovery offset for side stepping of left foot (in m). */
  float sideR0 = 0.f; /**< Recovery offset for side stepping of right foot (in m). */

  Angle turnStep = 0_deg; /**< Turn speed in radians/step. Anti-clockwise is positive. */
  Angle turnRL = 0_deg; /**< The turn angle for both feet (in radians). */
  Angle turnRL0 = 0_deg; /**< The turn angle for both feet when the support changed (in radians). */

  float maxFootHeight = 0.f; /**< Maximum foot height in current step (in m). */
  float footHL0 = 0.f; /**< Left foot height at the start of the current step (in m). */
  float footHR0 = 0.f; /**< Right foot height at the start of the current step (in m). */
  float footHL = 0.f; /**< Current height of left foot. */
  float footHR = 0.f; /**< Current height of right foot. */

  float prevSideL = 0.f; /**< The value of "sideL" in the previous cycle. For side start value for the next step. */
  float prevSideR = 0.f; /**< The value of "sideR" in the previous cycle. For side start value for the next step. */
  float lastPrevSideL; /**< The value of "prevSideL" of the previous cycle. For side start value for the next step. */
  float lastPrevSideR; /**< The value of "prevSideR" of the previous cycle. For side start value for the next step. */
  Angle prevTurn = 0.f; /**< The value of "turn" in the previous cycle. For odometry calculation. */

  Pose2f lastLeftSole; /**< Last pose of the left sole. For odometry calculation. */
  Pose2f lastRightSole; /**< Last pose of the right sole. For odometry calculation. */
  Pose2f currentLeftSole; /**< current pose of the left sole. For odometry calculation. */
  Pose2f currentRightSole; /**< current pose of the right sole. For odometry calculation. */

  WalkKickStep walkKickStep;

  Angle soleRotationYL; /**< Left ankle pitch after motion phase change. */
  Angle soleRotationYR; /**< Right ankle pitch after motion phase change. */
  Angle soleRotationXL; /**< Left ankle roll after motion phase change. */
  Angle soleRotationXR; /**< Right ankle roll after motion phase change. */

  Angle currentWalkPhaseKneeHipBalance = 0_deg; /**< Knee and hip pitch balance value from the last motion frame. */
  Angle lastWalkPhaseKneeHipBalance = 0_deg; /**< Knee and hip pitch balance value from the previous step. */

  int robotIsNotMoving = 0; /**< Robot is standing or hanging still. Used to detect when robot is stuck at a goal post.*/

  unsigned int gyroStateTimestamp = 0; /**< Last update of GyroState. */
  unsigned int timeWhenStandBegan = 0; /**< The timestamp when standing began (only valid if \c standFactor == 0). */
  unsigned int timeWhenStandHighBegan = 0; /**< The timestamp when standing high began (only valid if \c standFactor == 1). */
  unsigned int leftArmInterpolationStart = 0; /**< The timestamp the last time the left arms where not set by the WalkPhase. */
  unsigned int rightArmInterpolationStart = 0; /**< The timestamp the last time the right arms where not set by the WalkPhase. */
  unsigned int annotationTimestamp = 0; /**< The timestamp for the annotation and sound, when the balancer adjusted over a given threshold. */

  JointAngles startJointAngles; /**< The joint angles when the phase started (in order to interpolate to standing). */
  float standFactor = 0.f; /**< Value in [-1, 1]. -1 = start joint angles, 0 = stand, 1 = stand high. */
  float armCompensationAfterKick = 0.f;
  float leftArmInterpolationTime; /**< The interpolation duration for the left arm. */
  float rightArmInterpolationTime; /**< The interpolation duration for the right arm. */
  float slowDownHipKneeBalanceFactor = 0.f; /**< Additional gyro balance with the hip and knee when slowing down the walking speed. */

  bool afterKickWalkPhase = false; /**< Is the current step a motion phase after a get up or kick phase? */
  bool wasStandingStillOnce = false; /**< Needs to be true, so standHigh can start */
  bool afterWalkKickPhase = false; /**< Is the current step a motion phase after an inWalkKick phase? */
  bool useSlowSupportFootHeightInterpolation = false; /**< Shall the support foot height get interpolated back to 0 with a slower speed? */

  Pose3f lastComInFoot; /**< Last COM relativ to the current support foot. */
  int doBalanceSteps = 0; /**< Number of balance steps. Possible step range is replaced by how the COM moved in the last step. */
  int noFastTranslationPolygonSteps = 0; /**< Number of steps before big steps near the target are allowed. */
  Angle armCompensationTilt = 0_deg; /**< Torso tilt to compensate arm position in high stand. */

  JointRequest leftArm; /**< The last left arm request, that was not set by the WalkPhase. */
  JointRequest rightArm; /**< The last right arm request, that was not set by the WalkPhase. */

  Vector2f ball; /**< Relative ball position to the swing foot. */

  WalkStepAdjustment::HipGyroBalanceState hipGyroBalance = WalkStepAdjustment::noHipBalance;

  enum WalkState
  {
    standing,
    starting,
    walking,
    stopping
  } walkState = standing;

  enum { weightDidShift, weightDidNotShift, emergencyStand, emergencyStep } weightShiftStatus;

  WalkGenerator::CreateNextPhaseCallback createNextPhaseCallback;
  friend class WalkingEngine;
};
