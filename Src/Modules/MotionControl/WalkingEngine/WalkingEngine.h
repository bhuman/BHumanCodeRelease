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

#include "Framework/Module.h"
#include "Framework/Settings.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/MotionControl/InterceptBallGenerator.h"
#include "Representations/MotionControl/KeyframeMotionGenerator.h"
#include "Representations/MotionControl/KickGenerator.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/StandGenerator.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkLearner.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointAnglePred.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/JointPlayTranslation.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/RobotStableState.h"
#include "Representations/Sensing/SolePressureState.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Motion/JointSpeedController.h"
#include "Streaming/Global.h"
#include "WalkLibs/WalkPhaseBase.h"

STREAMABLE(BalanceParameters,
{,
  (float) gyroLowPassRatio, /**< To which ratio keep old gyro measurements? */
  (float) gyroForwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling forwards while walking? */
  (float) gyroBackwardBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling backwards while walking? */
  (float) gyroSidewaysBalanceFactor, /**< How much are gyro measurements added to ankle joint angles to compensate falling sideways while standing? */
  (float) gyroHipKneeBalanceFactor, /**< Very small gyro balancing for the knee and hip pitch. */
  (float) gyroHipKneeKickBalanceFactor, /**< Very small gyro balancing for the knee and hip pitch. */
  (Angle) slowdownTorsoOffset, /**< Start slowing down the walking speed at this torso rotation. */
  (float) slowdownFactor, /**< Slow down walking speed down to this factor. */
  (Angle) minTorsoRotation, /**< If the torso is this much further back, reduce the walking speed. The reduction is interpolated. */
  (bool) allowSlowDownWhenWalkingBackwards, /**< If true, also slow down when walking backwards. */
  (Rangea) sideTorsoRange, /**< Scale side gyro balancing based on error in the torso x rotation. */
  (Rangef) sideComRange, /**> Scale side gyro balancing based on com position. */
  (float) gyroSideTorsoBalance, /**< How much are gyro measurements added to the hip roll? */
  (Angle) maxGyroAnkleBalance, /**< The sole y rotation relative to the estimated ground is max allowed to be +- this value from the gyro balancing. */
});

/**< The maximum possible speed. DO NOT use for walking. Only meant for special cases like kicks. */
STREAMABLE(WalkSpeedParams,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
});

STREAMABLE(CommonSpeedParameters,
{,
  (Vector2f) maxAcceleration,  /**< Maximum acceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (Vector2f) maxDeceleration, /**< (Positive) maximum deceleration of forward and sideways speed at each leg change to ratchet up/down in (mm/s/step). */
  (float) fastFeetAdjustment, /**< Feet can move this fast to adjust to a specific pose. */
  (float) slowFeetAdjustment, /**< Feet shall move this slow to adjust to a specific pose. */
  (Rangea) soleRotationOffsetSpeed, /**< Sole rotation can return to neutral position with this speed (degree/s). */
  (int) soleRotationOffsetSpeedAfterKickTime, /**< After a kick use a lower soleRotationOffsetSpeed to interpolation the sole rotation. */
  (float) walkSpeedReductionFactor, /**< Reduce the walking speed by this amount (only x translation). */
  (int) reduceWalkingSpeedStepAdjustmentSteps, /**< If the step adjustment adjusted the feet too much two separate times, then reduce the walking speed for this number of walking steps. */
  (float) afterKickFeetHeightAdjustment, /**< Feet height can change this much after a specific kick. */
  (float) maxSwingStretchLength, /**< If a step takes too long, strecht swing sole to catch the robot earlier and prevent more destabilization (in mm). */
  (float) maxSwingSideStretch, /**< Move swing sole sideways if a step takes to long, to prevent more destabilization (in mm). */
  (float) swingStretchAfterOvertime, /**< A step is considered too long after this additional time (in ms). */
});

STREAMABLE(ConfiguredParameters,
{,
  (Pose2f) maxSpeed, /**< Maximum speeds in mm/s and degrees/s. */
  (Pose2f) minSpeed, /**< Minimum speeds in mm/s and degrees/s. */
  (float) maxSideWithMaxForwardSpeed, /**< Do not allow the 100% forward and side speed, but reduce the forward speed for full side speed. */
  (float) maxSpeedBackwards, /**< Maximum backwards speed. Positive, in mm/s. */
  (float) minSpeedBackwards, /**< Minimum backwards speed. Positive, in mm/s. */
  (int) emergencyNotMovingTime, /**< Gyro state measures low deviation and mean values so many times in a row, to start a new step early. */
  (Rangef) supportSwitchPhaseRange, /**< In which range of the walk phase can the support foot change? */
  (Pose2f) thresholdStopStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (Pose2f) thresholdDiveStandTransition, /**< Threshold to be able to switch from walk state stopping to standing. */
  (WalkSpeedParams) walkSpeedParams, /**< Walkspeed parameters for the theoretical max possible speed. ONLY USE use for walking IF you know what you doing! */
  (JointPlayOffsetParameters) jointPlayOffsetParameters, /**< Parameters for the joint play offset algorithmic. */
  (bool) dynamicStepUpdate, /**< Update interception steps while being executed? */
  (bool) transitionWalkToKick, /**< Start in walk kicks after walk step started. */
  (bool) dynamicKickPoseUpdate, /**< Update interception steps while being executed in the rolling ball challenge. */
  (bool) dynamicKickDelayUpdate, /**< Update kick delay based on ball roll path. */
  (bool) dynamicKickStepUpdate, /**< Update kick parameters at the beginning of the walk step. */
  (bool) energySavingWalk,
});

STREAMABLE(StepSizeParameters,
{,
  (float) insideTurnRatio, /**< The ratio of turn that can be done in the inner step of a double-step. */
  (Vector2a) reduceTranslationFromRotation, /**< The rotation from which the translation per step will be reduced. */
  (Vector2a) noTranslationFromRotation, /**< The rotation from which no translation is possible at all. */
  (Angle) noTranslationYFromRotationFastInner, /**< When walking fast with a rotation outwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) noTranslationYFromRotationFastOuter, /**< When walking fast with a rotation inwards, the rotation from which no translation is possible at all is higher for the y translation. */
  (Angle) reduceTranslationYFromRotationFast, /**< When walking fast, the rotation from which the translation per step will be reduced is higher for the y translation. */
  (float) minXTranslationStep, /**< The forward and backward step size has a minimum. */
  (float) minXForwardTranslationFast, /**< The forward step size has a minimum for the fast translation polygon. */
  (Rangef) minXBackwardTranslationFastRange,/**< The backward step size has a minimum for the fast translation polygon. */
  (Rangef) translationPolygonSafeRange, /**< For the calculation of the translation polygon, the feet are adjusted by this amount in x translation,
                                        to ensure a possible pose even when the arms are on the back or the feet are balanced. */
  (int) noFastTranslationPolygonStepsNumber, /**< Number of steps before big steps near the target are allowed. */
});

STREAMABLE(ArmParameters,
{,
  (Angle) armShoulderRoll, /**< Arm shoulder angle in radians. */
  (float) armShoulderRollIncreaseFactor, /**< Factor between sideways step size (in m) and additional arm roll angles. */
  (float) armShoulderPitchFactor, /**< Factor between forward foot position (in m) and arm pitch angles. */
  (float) comTiltFactor, /**< Factor between the correct torso shift and the one actually used. Used to tilt the torso to compensate the arm positions in high stand. */
  (float) armInterpolationTime, /**< Interpolation time for the arms, when the WalkPhase can set them again. */
  (Angle) armShoulderPitch, /**< Arm shoulder pitch angle. */
  (Angle) armElbowYaw, /**< Arm elbow yaw angle. */
});

STREAMABLE(EmergencyStepParameters,
{,
  (float) emergencyStepSize, /**< The size of emergency sideways steps in mm. */
  (float) emergencyStepHeightFactor, /**< During an emergency step, the step height is multiplied by this factor. */
  (Angle) emergencyMaxGyroMean, /**< Gyro state must measure gyro values below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxGyroDeviation, /**< Gyro state must measure gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
  (Angle) emergencyMaxZGyroDeviation, /**< Gyro state must measure z gyro value deviations below this threshold, to increase emergencyNotMovingCounter. */
  (int) emergencyAfterStepDuration, /**< Allow forced support foot switch after stepDuration + this time. */
});

STREAMABLE(KinematicParameters,
{,
  (float) baseWalkPeriod, /**< Duration of a single step, i.e. half of a walk cycle (in ms). */
  (float) sidewaysWalkHeightPeriodIncreaseFactor, /**< Additional duration for the swing height, when walking sideways at maximum speed (in ms). */
  (float) sidewaysHipShiftFactor, /**< Apply the requested sidewards target with this factor on the support foot. */
  (Rangef) walkHipHeight, /**< Walk hip height above ankle joint in mm. Min value is the default, max value is used when walking slow to reduce heating up. */
  (WalkSpeedParams) energyEfficientWalkSpeed, /**< Scale hip height to a higher value based on slower walking speed. */ // TODO should be the same as in WalkToPoint.cpp! Move both into the same config
  (WalkSpeedParams) noEfficientWalkSpeed, /**< Scale hip height to a higher value based on slower walking speed. */ // TODO should be the same as in WalkToPoint.cpp! Move both into the same config
  (Rangef) hipHeightChangeSpeed, /**< Change hip height with these speeds (mm/s). Min is used to reduce the height, Max is used to increase the height. */
  (float) baseFootLift, /**< Base foot lift in mm. */
  (float) torsoOffset, /**< The base forward offset of the torso relative to the ankles in mm. */
  (float) legLengthClipThreshold, /**< After special movements like a kick the inverse kinematic is allowed to clip the legs by up to this amount (in mm). */
  (float) yHipOffset, /**< Shift legs this far outwards (in mm). */
});

STREAMABLE(ParabolicFootHeightParameters,
{,
  (float) maxHeightAfterTime,
  (float) maxHeightAfterTimePercent,
});

STREAMABLE(WalkDelayParameters,
{,
  (float) minDelay, /**< Min duration for the delay phase. */
  (Rangef) heightOffset, /**< Min and max height adjustment for the swing foot (in mm). */
  (float) endHeightShift, /**< In the last few frames, apply a big chunk of the height interpolation. */
  (Rangef) delayInterpolation, /**< Interpolation range for the swing foot height (in s). */
  (Rangef) sideShift, /**< How much is the support foot shifted (in mm). */
  (Rangef) sideShiftDelayInterpolation, /**< Use more side shift for higher delays. */
  (float) translationBuffer, /**< When checking if the delay is possible, add this much translation on top of the swing foot. */
  (Rangef) kickTimeOffset, /**< For kicks we update the delay. This offset is the time from the start of the kick and reaching the ball, scaled based on the x-translation distance to the ball when hit. */
  (Rangef) kickTimeBallDistanceScaling, /**< Scale the time offset of the kick based on the x-translation distance of the ball when hit. */
});

STREAMABLE(SideStabilizeParameters,
{,
  (Rangea) turnIncreaseRange, /**< Increase threshold range for WalkDelayPhase if feet are positive rotated. */
  (float) increaseThreshold, /**< Increase threshold range for WalkDelayPhase by this much (in mm). */
  (float) minOuterSide, /**< Min threshold for CoM in new swing foot, to start WalkDelayPhase. */
  (float) minOuterSideStop, /**< Min threshold for CoM in new swing foot when fully stopped, to start WalkDelayPhase. */
  (Rangef) sideHipShiftStepSizeRange, /**< Shift the hip in the y-axis, depending on this range of a previous side step size (of closing feet together). */
  (float) maxSideHipShift, /**< Shift the hip in the y-axis by max this value. */
  (float) maxSideHipShiftStepSize, /**< Reduce the hip shift in the y-axis based on the current side step size. Less side step -> more hip shift. */
  (Rangef) comInOuterInterpolationRange,
  (Rangef) heightRange,
  (float) firstStepHeightRange, /**< The first step after standing shall use a delay phase with this ratio for the height. */
  (float) lastLastWalkPhaseStepDuration, /**< Threshold to handle left over side momentum, if last last step execution duration was too long. */
  (float) lastWalkPhaseStepDuration, /**< Threshold to handle left over side momentum, if last step execution duration was too long. */
  (Rangef) delayRange, /**< Used delay (in ms). Min for normal walk, max for after kicks. */
  (float) minHipShiftRatioForDelay, /** Last side hip shift must be bigger than this % value of the max hip shift. */
  (float) maxNextSideStepSize, /**< The next side step must be below this step size to allow for a delay. */
});

STREAMABLE(WalkHipShiftPhaseParameters,
{,
  (float) shiftSpeed, /**< Adjusting speed for the side translation (in mm/s) */
  (float) shiftSpeedStart, /**< Adjusting speed for the side translation for the first walk step (in mm/s) */
  (Rangef) comYStopConditionWalking, /**< When the com is this much (in mm) more towards the support sole, then stop shifting and continue walking. */
  (float) comYStopConditionStarting, /**< When the com is this much (in mm) more towards the support sole, then stop shifting and start walking. */
  (Rangef) maxShift, /**< Max shift (in mm). */
  (Rangea) comShiftScalingBackward, /**< Based on the rotation error of the sole to the ground, reduce the hip shifting to a minimum to allow reacting fast if necessary. */
  (Rangea) comShiftScalingForward, /**< Based on the rotation error of the sole to the ground, reduce the hip shifting to a minimum to allow reacting fast if necessary. */
  (Rangef) comShiftScalingWalkingBackward, /**< Based on the walk step size reduce the hip shifting to a minimum to allow reacting fast if necessary. */
  (Rangef) comShiftScalingWalkingForward, /**< Based on the walk step size reduce the hip shifting to a minimum to allow reacting fast if necessary. */
  (Rangef) deltaScaling, /**< Based on the delta of the walk step adjustment reduce the hip shifting to a minimum to allow reacting fast if necessary. */
  (float) minShiftStartWalking, /**< When starting to walk, apply at least this much of a shift. */
  (bool) useHipShift, /**< If true, this WalkHipShiftPhase is always used between steps and the WalkDelayPhase is disabled. */
});

STREAMABLE(SideCatchWalkParameters,
{,
  (Rangef) allowedAdjustmentDuringTimeRatio, /**< During this time of the walk step (tBase / duration) (in %), the side catch is active. */
  (Rangef) comYThreshold, /**< Interpolate the side catch speed based on the com position. */
  (float) maxAdjustment,
});

STREAMABLE(SupportCompensationParameters,
{,
  (float) lowPassFilterSlow, /**< Slow filter factor. */
  (float) lowPassFilterFast, /**< Fast filter factor. */
  (bool) isActive, /**< Is compensation active? */
  (float) scaling, /**< Take this much of the error as compensation. Value should be between 0 and 1. */
  (Rangef) timeScaling, /**< Take less error at the beginning and full error at max time. */
});

STREAMABLE(WalkingEngineCommon,
{,
  (float) minPhaseForStopWithWrongGroundContact, /**< If the swing foot has ground contact and enough time has passed since the start of the walk phase, freeze all movement. */
  (float) clipAtBallDistanceX, /**< Step adjustment is not allowed to touch the ball. */
  (float) clipAtBallDistance, /**< Ball is ignored when it further away than this threshold. */
  (Angle) standHighTorsoPitch, /**< The torso pitch angle for standHigh. */
  (Angle) standInterpolationVelocity, /**< The interpolation speed to interpolate to stand (in degree/s). */
  (float) standHighInterpolationDuration, /**< The duration to interpolate from/to stand to/from stand high (in ms). */
  (float) maxStandInterpolationDuration, /**< The max duration to interpolate to stand after play dead (in ms). */
  (int) standHighNotMovingTime, /**< Robot must be standing still for so long (in ms), to allow high stand. */

  (float) minTimeForEarlySwitch, /**< An early support foot switch, when the wrong foot is the supporting one, is allowed after this much time (in s) has passed. */

  (Angle) maxWalkDirectionForFootPrediction, /**< The direction of the step target must be lower than angle to allow for foot support switch prediction. */
  (bool) useFootSupportSwitchPrediction, /**< Use predicted support foot switches. */
  (bool) blockStoppingWithStepAdjustment, /**< Step adjustment can not move freely forwards, if the swing foot would collide with the ball. */
  (bool) useJointPlayScaling, /**< Scale the walking speed based on the joint play? */

  (CommonSpeedParameters) commonSpeedParameters, /**< Some common parameters that are related to speed. */
  (KinematicParameters) kinematicParameters,
  (EmergencyStepParameters) emergencyStep, /**< Parameters for the emergency step. Needed if the robot is stuck at a goal post. */
  (ArmParameters) armParameters, /**< Arm parameters. Normal arm swinging and arm compensation. */
  (StepSizeParameters) stepSizeParameters, /**< Step size parameters. */
  (BalanceParameters) balanceParameters, /**< Balancing parameters. */
  (WalkStepAdjustmentParams) walkStepAdjustmentParams, /**< Walk step adjustment parameters. */
  (SoleRotationParameter) soleRotationParameter, /**< Sole rotation compensation parameters, to keep the swing foot parallel to the ground. */
  (ParabolicFootHeightParameters) parabolicFootHeightParameters,  /**< Foot height interpolation parameters. */
  (WalkStiffnessParameters) walkStiffnessParameters, /**< Stiffness regulation parameters.*/
  (SpeedControlParams) speedControlParams, /**< Speed and threshold parameters for the joint speed regulation. */
  (WalkDelayParameters) walkDelayParameters, /**< Parameters for the walk delay phase. */
  (WalkSpeedParams) walkSpeedParamsWalkStep, /**< Walk speed parameters for the theoretical max possible speed for WalkStepAdjustment. DO NOT use for walking! */
  (SideStabilizeParameters) sideStabilizeParameters, /**< Conditions to start a WalkDelayPhase and values to shift the hip on the y-axis. */
  (WalkHipShiftPhaseParameters) walkHipShiftPhaseParameters, /**< Support sole will have some errors. Compensate them with the swing sole. */
  (SideCatchWalkParameters) sideCatchWalkParameters, /**< The parameters for the side catch. */
  (SupportCompensationParameters) supportCompensationParameters, /**< Support sole will have some errors. Compensate them with the swing sole. */
});

MODULE(WalkingEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CalibrationRequest),
  REQUIRES(EnergySaving),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(IMUValueState),
  REQUIRES(InertialData),
  USES(InterceptBallGenerator),
  REQUIRES(JointAnglePred),
  REQUIRES(JointAngles),
  REQUIRES(JointPlay),
  REQUIRES(JointPlayTranslation),
  USES(JointRequest),
  USES(KeyframeMotionGenerator),
  REQUIRES(KickGenerator),
  REQUIRES(KickInfo),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(RobotStableState),
  REQUIRES(SolePressureState),
  REQUIRES(StiffnessSettings),
  REQUIRES(TorsoMatrix),
  USES(WalkKickGenerator),
  REQUIRES(WalkLearner),
  PROVIDES(WalkStepData),
  USES(WalkStepData),
  PROVIDES(WalkGenerator),
  REQUIRES(WalkGenerator),
  PROVIDES(StandGenerator),
  PROVIDES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (ConfiguredParameters) configuredParameters,
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
   * @param useMaxPossibleStepSize generate polygon with theoretical bigger step size?
   */
  void generateTranslationPolygon(std::vector<Vector2f>& polygon, const Vector2f& backRight, const Vector2f& frontLeft, const bool useMaxPossibleStepSize);

  /**
   * Filter the polygon to not contain the points that are not needed.
   */
  void filterTranslationPolygon(std::vector<Vector2f>& polygonOut, std::vector<Vector2f>& polygonIn, const std::vector<Vector2f>& polygonOriginal);

  /**
   * Calculates the pose of the feet.
   * @param forwardL x translation position for the left foot, without torso shift
   * @param forwardR x translation position for the right foot, without torso shift
   * @param sideL y translation position for the left foot, without hip shift
   * @param sideR y translation position for the right foot, without hip shift
   * @param footHL foot height
   * @param footHR right foot height
   * @param turn z rotation
   * @param currentWalkHipHeight current walk height
   * @param soleRotationYL left foot y rotation
   * @param soleRotationXL left foot x rotation
   * @param soleRotationYR right foot y rotation
   * @param soleRotationXR right foot x rotation
   * @param leftFoot left foot pose
   * @param rightFoot right foot pose
   * @param torsoShift Current torso shift
   */
  void calcFeetPoses(const float forwardL, const float forwardR, const float sideL, const float sideR,
                     const float footHL, const float footHR, const Angle turn, const float currentWalkHipHeight,
                     const Angle soleRotationYL, const Angle soleRotationXL, const Angle soleRotationYR, const Angle soleRotationXR,
                     Pose3f& leftFoot, Pose3f& rightFoot) const;

  /**
   * Based on the rotation, get the max possible step size in %
   * @param rotation The rotation
   * @param isFastWalk Is fast walk, e.g. more translation in rotation, allowed?
   * @param isLeftPhase Is the next walk step a left phase?
   * @param customReduceOffset Extra offset for the rotation. Currently when the arms are on the back, more translation is allowed
   * @param walkSpeedRatio The max translation speed that is allowed
   */
  Vector2f getStepSizeFactor(const Angle rotation, const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& walkSpeedRatio);

  /**
   * Get max possible rotation, without reducing the step size
   * @param isFastWalk Is fast walk, e.g. more translation in rotation, allowed?
   * @param isLeftPhase Is the next walk step a left phase?
   * @param customReduceOffset Extra offset for the rotation. Currently when the arms are on the back, more translation is allowed
   * @param stepRatio The % values of the step size relative to the maximum possible in the current walking step
   */
  Rangea getMaxRotationToStepFactor(const bool isFastWalk, const bool isLeftPhase, const Angle customReduceOffset, const Vector2f& stepRatio);

  /**
   * Get the current walk height based on the executed step
   * @param walkStep The executed step
   * @param stepDuration The duration of the step
   * @return The max walk height
   */
  float updateWalkHipHeight(const Pose2f& walkStep, const float stepDuration) const;

  Angle filteredGyroX, /**< low-pass filtered gyro values. */
        filteredGyroY;

  std::vector<Vector2f> translationPolygon; /**< The polygon that defines the max allowed translation for the step size. */
  std::vector<Vector2f> translationPolygonBig; /**< The polygon that defines the max allowed translation for the step size with much higher values. DO NOT use for walking. */
  std::vector<Vector2f> translationPolygonBigNotClipped; /**< The polygon that defines the max allowed translation for the step size with much higher values without clipping. DO NOT use for walking. */
  std::vector<Vector2f> translationPolygonAfterKick; /**< The polygon that defines the max allowed translation after a kick from the KickEngine. */

  /** The constructor loads the common parameters. */
  WalkingEngine();

  /** Register common parameters. */
  static void reg();

  const float motionCycleTime = Global::getSettings().motionCycleTime;
  static constexpr float mmPerM = 1000.f;
};

struct DummyPhase : MotionPhase
{
  using MotionPhase::MotionPhase;

  bool isDone(const MotionRequest&) const override { return true; }
  void calcJoints(const MotionRequest&, JointRequest&, Pose2f&, MotionInfo&) override {}
};

struct WalkPhase : WalkPhaseBase
{
  WalkPhase(WalkingEngine& engine, Pose2f stepTarget, const MotionPhase& lastPhase, const bool isDelay = false,
            const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
            const WalkKickStep& walkKickStep = WalkKickStep());

protected:
  void update();

  bool isDone(const MotionRequest& motionRequest) const;

  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo);

  std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const;

  /**
   * Calculate the translation and rotation variables for both feet while walking and standing.
   * @param swingSign Sign of the swing foot. 1 for left foot is the swing foot and -1 for the right foot is the swing foot.
   * @param ratio Current time in the step with slow down time (in s)
   * @param ratioSide Current time in step (in s) for side translation
   * @param ratioBase Current time in step (in s)
   * @param duration Planned duration of the step (in s)
   * @param heightDuration Used duration for the step height (in s). Side steps use a longer step duration for the height interpolation.
   * @param forwardSwing0 Start value for the forward position of the swing foot.
   * @param forwardSupport0 Start value for the forward position of the support foot.
   * @param forwardSwing Current forward position of the swing foot.
   * @param forwardSupport Current forward position of the support foot.
   * @param sideSwing0 Start value for the side position of the swing foot.
   * @param sideSupport0 Start value for the side position of the support foot.
   * @param sideSwing Current side position of the swing foot.
   * @param sideSwingStretch Current swing side adjustment for too long steps.
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
  void calcFootOffsets(const float swingSign, const float ratio, const float ratioSide, const float ratioBase,
                       const float duration, const float heightDuration,
                       const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                       float& forwardSupport, const float sideSwing0, const float sideSupport0,
                       float& sideSwing, float& sideSwingStretch, float& sideSupport, float& footHeightSwing0,
                       float& footHeightSupport0, float& footHeightSwing,
                       float& footHeightSupport, Angle& turnVal,
                       const float useForwardSpeed, const float useSideSpeed, const Angle useTurnSpeed) final;

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
   * @param sideSwingStretch Current swing side adjustment for too long steps.
   * @param sideSupport Current side position of the support foot.
   * @param footHeightSwing0 Start height of the swing foot.
   * @param footHeightSupport0 Start height of the support foot.
   * @param footHeightSwing Current height of the swing foot.
   * @param footHeightSupport Current height of the support foot.
   * @param turnVal Current rotation value of the feet.
   * @param walkKickStep Describes all important parameters for the step, like multiple step sizes and their interpolation
   * @param turnRL0 Start rotation value of the feet.
   */
  void calcWalkKickFootOffsets(const float swingSign, const float ratio,
                               float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                               float& forwardSupport, float& sideSwing0, float& sideSupport0,
                               float& sideSwing, float& sideSwingStretch, float& sideSupport, float& footHeightSwing0,
                               float& footHeightSupport0, float& footHeightSwing,
                               float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                               Angle& turnRL0) final;

  /**
   * Set the joint request for the arms, or use the given values
   * @param leftFoot Left foot pose.
   * @param rightFoot Right foot pose.
   * @param jointRequest The given arm positions, which may be overridden bei walkArms if they should be ignored.
   * @param walkArms The arm positions based on the current walk.
   */
  void setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms) final;

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
  void compensateArms(JointRequest& jointRequest, const JointRequest& walkArms) final;

  void calculateCurrentBaseStepVariables();

  /**
   * Update step target dynamically while it is already executed. Only used when intercepting.
   */
  void updateDynamicStep();

  /**
   * The current walk phase is not executing a kick. Check for the first calculation cycles, whether a kick could still start
   * and step up everything it is the case.
   */
  void transitionWalkToKick();

  /**
   * Calculate the compensation for translation errors of the support sole.
   */
  void calculateSupportSideCompensation();

  WalkGenerator::CreateNextPhaseCallback createNextPhaseCallback;

  bool wasInterceptingLastFrame = false;

  bool isInWalkKickTransitionAllowed = false;

  LowPassFilterPR supportForwardCompensation;

  friend struct WalkDelayPhase;
  friend struct WalkHipShiftPhase;
};

// A walk phase with a start delay
struct WalkDelayPhase : WalkPhase
{
  WalkDelayPhase(WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase, const float delay, const float height = 1.f,
                 const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
                 const WalkKickStep& walkKickStep = WalkKickStep());

  static bool isWalkDelayPossible(float forwardL0, float forwardR0, float sideL0, float sideR0,
                                  float footHL0, float footHR0, const float turnRL0, const float currentWalkHipHeight,
                                  const Angle soleRotationYL, const Angle soleRotationXL,
                                  const Angle soleRotationYR, const Angle soleRotationXR,
                                  const bool isLeftPhase, const bool lastPhaseWasKick, const float delay,
                                  const WalkDelayParameters& params, const WalkingEngine& engine,
                                  const bool usedWalkDelay = false);

  static float getSideShift(const bool lastPhaseWasKick, const float delay, const WalkDelayParameters& params);

private:
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) final;

  void update() final;

  /**
   * For kicks the delay shall be updated every frame.
   * If a delay is big, the perception of the ball dynamic will be off.
   * Without an update, the timing of the kick will therefore be off otherwise.
   */
  float updatedDelayForKick();

  float delay; /**< The defined delay before the walk phase starts. */
  float timeActive = 0.f; /**< How much time has passed on the delay phase? */
  float originalFootHL0 = 0.f; /**< Original left foot height. */
  float originalFootHR0 = 0.f; /**< Original right foot height. */
  bool lastPhaseWasKick = false; /**< Last motion phase was a kick. */
  const float percentHeight = 0.f;
  bool finishedDelay = false;
  unsigned lastBallTimestamp = 0;
  float ballTimeOffset = 0.f;
};

// A walk phase between two walking steps to shift the hip
struct WalkHipShiftPhase : WalkPhase
{
  WalkHipShiftPhase(WalkingEngine& engine, const Pose2f& stepTarget, const MotionPhase& lastPhase,
                    const WalkGenerator::CreateNextPhaseCallback& createNextPhaseCallback = WalkGenerator::CreateNextPhaseCallback(),
                    const WalkKickStep& walkKickStep = WalkKickStep());

private:
  void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) final;

  void update() final;

  void doPseudoWalkStepAdjustmentUpdate();

  bool finishedShifting = false;
  float leftShift = 0.f;
  float rightShift = 0.f;
  float expectedDelta = 0.f;
};
