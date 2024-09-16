/**
 * @file WalkPhaseBase.h
 * This file declares helper functions for the WalkingEngine.
 * @author Philip Reichenberg
 */

#pragma once
#include "WalkStepAdjustment.h"
#include "Platform/SystemCall.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/RobotStableState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Motion/ForwardKinematic.h"
#include "Tools/Motion/InverseKinematic.h"
#include "Tools/Motion/JointSpeedRegulator.h"
#include "Tools/Motion/JointPlayOffsetRegulator.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Motion/WalkKickStep.h"

class WalkingEngine;

struct WalkPhaseBase : MotionPhase
{
  WalkPhaseBase(WalkingEngine& engine, const WalkKickStep& walkKickStep);

  std::vector<Vector2f> getTranslationPolygon(const float maxBack, const float maxFront, const float maxSide, const bool useSafeSpace);

  struct SupportSwitch
  {
    bool isPredictedSwitch = false;
    bool isNormalSwitch = false;
    bool isAbortSwitch = false;
    bool isStuckSwitch = false;
    bool isOvertimeSwitch = false;
    bool isFeetStepAbort = false;
    bool isFootSupportSwitch = false;
  };

  SupportSwitch supportSwitchInfo;

protected:

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
  virtual void calcFootOffsets(const float swingSign, const float ratio, const float ratioSide, const float ratioBase,
                               const float duration, const float heightDuration, const float stepDurationSide,
                               const float forwardSwing0, const float forwardSupport0, float& forwardSwing,
                               float& forwardSupport, const float sideSwing0, const float sideSupport0,
                               float& sideSwing, float& sideSupport, float& footHeightSwing0,
                               float& footHeightSupport0, float& footHeightSwing,
                               float& footHeightSupport, Angle& turnVal,
                               const float useForwardSpeed, const float useSideSpeed, const Angle useTurnSpeed) = 0;

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
  virtual void calcWalkKickFootOffsets(const float swingSign, const float ratio,
                                       float& forwardSwing0, float& forwardSupport0, float& forwardSwing,
                                       float& forwardSupport, float& sideSwing0, float& sideSupport0,
                                       float& sideSwing, float& sideSupport, float& footHeightSwing0,
                                       float& footHeightSupport0, float& footHeightSwing,
                                       float& footHeightSupport, Angle& turnVal, WalkKickStep& walkKickStep,
                                       Angle& turnRL0) = 0;

  /**
   * Apply the step adjustment to keep the robot balanced.
   * @param leftFoot Requested left foot pose.
   * @param rightFoot Requested right foot pose.
   */
  void balanceFeetPoses(Pose3f& leftFoot, Pose3f& rightFoot);

  /**
   * Apply all balancing based on the gyro measurement.
   * @param jointRequest Requested joint positions.
   */
  JointAngles addGyroBalance(JointRequest& jointRequest, const JointAngles& lastRequest);

  /**
   * Returns values on a parabola with 0% = 100% = 0, currentTime / walkHeightDuration == maxHeightAfterTime / walkDuration = 1
   * @param currentTime Current time in the walk phase
   * @param walkHeightDuration planned duration for the walk height
   * @param walkDuration planned walk duration
   * @return The value on the parabola for "f".
   */
  float parabolicFootHeight(const float currentTime, const float walkHeightDuration) const;

  /**
   * Returns values on a parabola with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the parabola for "time".
   */
  float parabolicStep(float time, float period) const;

  /**
   * Returns values on a sinus curve from 0 to pi_2 with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the sinus curve for "time".
   */
  float sinusMaxToZeroStep(float time, float period) const;

  /**
   * Returns values on a linear interpolation with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the linear interpolation for "time".
   */
  float linearStep(float time, float period) const;

  /**
   * Returns values on a cosine curve from (-pi_2 to 0) + 1 with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the cosine curve for "time".
   */
  float cosinusZeroToMaxStep(float time, float period) const;

  /**
   * Returns values with f(0) = 0, f(period) = 1.
   * @param time A value between 0 and "period".
   * @param period The duration of a period.
   * @return The value on the cosine curve for "time".
   */
  float swingInterpolation(float time, float period, const WalkKickStep::InterpolationType type) const;

  /**
   * Calculates if the next step can be a standing phase.
   * @param forwardL x translation position for the left foot, without torso shift.
   * @param forwardR x translation position for the right foot, without torso shift.
   * @param sideL y translation position for the left foot, without hip shift.
   * @param sideR y translation position for the right foot, without hip shift.
   * @param turn z rotation.
   * @param balanceAdjustmentLeft Left foot step adjustment value.
   * @param balanceAdjustmentRight Reft foot step adjustment value.
   * @param highestAdjustmentX Highest adjustment value in the last step
   * @param isDive Is a dive motion requested?
   */
  bool isStandingPossible(const float forwardL, const float forwardR, const float sideL, const float sideR, const float turnRL,
                          const float balanceAdjustmentLeft, const float balanceAdjustmentRight, const float highestAdjustmentX,
                          const float sideHipShift, const bool isDive) const;

  /**
   * Set the joint request for the arms, or use the given values
   * @param leftFoot Left foot pose.
   * @param rightFoot Right foot pose.
   * @param jointRequest The given arm positions, which may be overridden bei walkArms if they should be ignored.
   * @param walkArms The arm positions based on the current walk.
   */
  virtual void setArms(Pose3f& leftFoot, Pose3f rightFoot, JointRequest& jointRequest, JointRequest& walkArms) = 0;

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
  virtual void compensateArms(Pose3f& leftFoot, Pose3f& rightFoot, JointRequest& jointRequest, const JointRequest& walkArms) = 0;

  /**
   * Apply joint offsets of the current in walk kick
   * @param jointRequest Current joint request.
   * @param time Current time of the step (in s).
   */
  void applyWalkKickLongKickOffset(JointRequest& jointRequest, const float time);

  /**
   * Calculate the ball position relative to the swing foot zero position
   */
  void calculateBallPosition(const bool isLeftPhase);

  /**
   * Copy values from last walk phase.
   */
  void constructorHelperInitVariables();

  /**
   * Init variables with the last walk phase
   * @param lastWalkPhaseDummy last walk phase
   * @param resetWalkStepAdjustment reset the walkStepAdjustment?
   */
  void constructorHelperInitVariablesLastPhase(const WalkPhaseBase& lastWalkPhaseDummy, const bool resetWalkStepAdjustment);

  /**
   * Last motion phase was a walk phase. Init new walk phase accordingly.
   * @param lastPhase Last motion phase
   * @param useStepTarget The to be executed step target
   * @param lastStepTarget Step target of the previous walk phase
   * @param standRequested Is a stand phase requested
   */
  void constructorWalkCase(const MotionPhase& lastPhase, const Pose2f& useStepTarget, Pose2f& lastStepTarget, const bool standRequested);

  /**
   * Last motion phase was a stand phase. Init new walk phase accordingly.
   * @param standRequested Is a stand phase requested?
   * @param lastPhase Last motion phase
   * @param useStepTarget To be executed step target
   */
  void constructorStandCase(const bool standRequested, const MotionPhase& lastPhase, const Pose2f& useStepTarget);

  /**
   * Last motion phase was a kick and get up phase. Init new walk phase accordingly.
   */
  void constructorAfterKickOrGetUpCase(const MotionPhase& lastPhase);

  /**
   * Last motion phase was something else (i.e. playDead)
   */
  void constructorOtherCase();

  /**
   * Walkstate is not standing. Robot is still walking, so initialize walk parameters.
   */
  void constructorHelperInitWalkPhase(const Pose2f& useStepTarget, const Pose2f& lastStepTarget, const bool afterKickOrGetUp, const bool standRequested, const MotionPhase& lastPhase);

  /**
   * Handle the weight shift status.
   */
  void constructorWeightShiftStatus();

  /**
   * The robot had a support switch while the step adjustment still has a high delta.
   * Initialize the swing/support foot to prevent that the feet move delayed the the last requested position.
   * Goal:
   * - high positive delta -> swing foot should get a boosted start forward (more negative hip pitch value)
   * - high negative delta -> support foot should get a boosted start forward (more negative hip pitch value), swing foot a boosted start backward (more positive knee pitch value)
   */
  void constructorInitWithJointRequest(const Pose2f& useStepTarget, const WalkKickStep::OverrideFoot left, const WalkKickStep::OverrideFoot right);

  /**
   * This method recalculated the walk parameters given the jointRequest and the desired armCompensation value
   * @param jointAngles The desired joint request
   * @param theArmCompensation The desired arm compensation
   */
  void constructorArmCompensation(const JointAngles& jointAngles, const Angle theArmCompensation);

  void constructorJointSpeedRegulator();

  /**
   * When the walkstate is standing, handle the interpolation into standing and high stand.
   */
  void calcJointsHelperInterpolateStanding(const MotionRequest& motionRequest);

  /**
   * Some debug drawings
   * @param Handle the feet sole rotations
   */
  void calcJointsHelperFootSoleRotations(const JointRequest& jointRequest, const Angle hipRotation);

  /**
   * In case the robot is standing still and the CoM is relative in the middle, force a support foot switch
   */
  bool canForceFootSupportSwitch();

  /**
   * Apply offset for specific joint for InWalkKicks
   */
  JointAngles applyJointSpeedRegulation(JointRequest& jointRequest, const Angle supportSoleRotErrorMeasured);

  /**
   * Convert positions and rotations to the corresponding joint angles and the 3D poses
   */
  bool generateJointRequest(JointRequest& jointRequest, const Pose2f& stepTarget,
                            const bool convertStepTarget, const bool useIsLeftPhase,
                            const float ratio, float fL0, float fR0, float fL, float fR,
                            float sL0, float sR0, float sL, float sR,
                            float& fLH0, float& fRH0, float fHL, float fHR,
                            Angle& turn, Pose3f& leftFoot, Pose3f& rightFoot);

  /**
   * Init the joint speed control with the current max allowed rotational speed
   */
  void initMaxRotationSpeedPerMotionStep(const Pose2f& targetStep);

  /**
   * Converts the step target based on the rotation, to get the corrected step target (which is then executed)
   * This is due the fact that if the robot turns the swing foot will drift away (because only one foot has ground contact).
   * This results in an error in the position
   */
  Pose2f convertStep(const Pose2f& step, const Angle turnOffset);

  /**
   * To reduce the time of overheating, shift the soles depending on the temperatures of the knee and ankle pitch.
   */
  float temperatureShiftHandling(const Pose2f& stepSizeRequest);

  WalkingEngine& engine;

  SpeedRegulatorParams speedRegulatorParams;

  WalkStepAdjustment walkStepAdjustment;

  float tWalk = Constants::motionCycleTime; /**< Current time in the step (in s) but slowed down for forward translation and rotation. */
  float tWalkSide = Constants::motionCycleTime; /**< Current time in the step (in s) but slowed down for side translation. */
  float tBase = Constants::motionCycleTime; /**< Current time in the step (in s). */
  float stepDuration = 0.f; /**< Duration of the current step (in s). */
  float stepHeightDuration = 0.f; /**< Duration of the current step (in s). */
  float stepDurationSide = 0.f; /**< Duration for the side interpolation. Needs to be lower for smaller speed, otherwise swing foot gets stuck. */
  bool isLeftPhase = false; /**< Is the left foot the swing foot? */
  Pose2f step; /**< The step made in this phase (could be reconstructed from forwardStep, sideStep and turnStep). */
  Pose2f originalStep; /**< The original values of step. */
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

  float measuredSideL0 = 0.f;
  float measuredSideR0 = 0.f;

  float sideAcc = 0.f; /**< When dynamically changing the step for intercepting the ball, change the side direction with this acceleration. */

  Angle turnStep = 0_deg; /**< Turn speed in radians/step. Anti-clockwise is positive. */
  Angle turnRL = 0_deg; /**< The turn angle for both feet (in radians). */
  Angle turnRL0 = 0_deg; /**< The turn angle for both feet when the support changed (in radians). */

  Angle kneeBalance = 0_deg;
  bool disableSpeedControl = false;

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

  WalkKickStep walkKickStep;

  Angle soleRotationYL = 0_deg; /**< Left ankle pitch after motion phase change. */
  Angle soleRotationYR = 0_deg; /**< Right ankle pitch after motion phase change. */
  Angle soleRotationXL = 0_deg; /**< Left ankle roll after motion phase change. */
  Angle soleRotationXR = 0_deg; /**< Right ankle roll after motion phase change. */

  Angle currentWalkPhaseKneeHipBalance = 0_deg; /**< Knee and hip pitch balance value from the last motion frame. */
  Angle lastWalkPhaseKneeHipBalance = 0_deg; /**< Knee and hip pitch balance value from the previous step. */

  int wrongSupportFootCounter = 0;

  unsigned int gyroStateTimestamp = 0; /**< Last update of GyroState. */
  unsigned int timeWhenStandBegan = 0; /**< The timestamp when standing began (only valid if \c standFactor == 0). */
  unsigned int timeWhenStandHighBegan = 0; /**< The timestamp when standing high began (only valid if \c standFactor == 1). */
  unsigned int leftArmInterpolationStart = 0; /**< The timestamp the last time the left arms where not set by the WalkPhase. */
  unsigned int rightArmInterpolationStart = 0; /**< The timestamp the last time the right arms where not set by the WalkPhase. */
  unsigned int annotationTimestamp = 0; /**< The timestamp for the annotation and sound, when the balancer adjusted over a given threshold. */
  unsigned int timeWhenLastKick = 0; /**< The timestamp when the last KickEngine kick or get up happen. */

  JointAngles startJointAngles; /**< The joint angles when the phase started (in order to interpolate to standing). */
  float standFactor = 0.f; /**< Value in [-1, 1]. -1 = start joint angles, 0 = stand, 1 = stand high. */
  float armCompensationAfterKick = 0.f;
  float leftArmInterpolationTime; /**< The interpolation duration for the left arm. */
  float rightArmInterpolationTime; /**< The interpolation duration for the right arm. */

  bool afterKickWalkPhase = false; /**< Is the current step a motion phase after a get up or kick phase? */
  bool wasStandingStillOnce = false; /**< Needs to be true, so standHigh can start */
  bool afterWalkKickPhase = false; /**< Is the current step a motion phase after an inWalkKick phase? */
  bool useSlowSupportFootHeightAfterKickInterpolation = false; /**< Shall the support foot height get interpolated back to 0 with a slower speed? */
  float sideHipShift = 0.f; /**< How much the hip is shifted to the side in the current walking step. If the feet are moving together, shift the hip in the opposite direction. */
  bool freezeSideMovement = true; /**< Freeze the sideL and sideR calculation, because of stability reasons? */

  bool increaseInverseKinematicClipThreshold = false; /**< After special movements like kicks we allow for some clipping. */

  int doBalanceSteps = 0; /**< Number of balance steps. Possible step range is replaced by how the COM moved in the last step. */
  int noFastTranslationPolygonSteps = 0; /**< Number of steps before big steps near the target are allowed. */
  RingBuffer<Angle, 3> armCompensationTilt; /**< Torso tilt to compensate arm position in high stand. */

  JointRequest leftArm; /**< The last left arm request, that was not set by the WalkPhase. */
  JointRequest rightArm; /**< The last right arm request, that was not set by the WalkPhase. */

  Vector2f ball; /**< Relative ball position to the swing foot. */

  std::unique_ptr<JointSpeedRegulator> jointSpeedRegulator;
  std::unique_ptr<JointPlayOffsetRegulator> jointPlayOffsetRegulator;

  float armPitchShift[Legs::numOfLegs] = { 0.f, 0.f };

  JointRequest lastJointRequest;

  float torsoShift = 0.f;

  // AnklePitch shall not go further positive with the gyro balancing
  // Two entries needed because when standing both ankles need to be clipped
  Angle currentMaxAnklePitch[Legs::numOfLegs] = {0_deg, 0_deg};

  std::vector<Vector2f> previousInterceptTranslationPolygon; /**< The polygon that defines the max allowed translation for the step size from the previous walk phase. */

  enum WalkState
  {
    standing,
    starting,
    walking,
    stopping
  } walkState = standing;

  enum { weightDidShift, weightDidNotShift, emergencyStand, emergencyStep } weightShiftStatus;

  friend class WalkingEngine;
};
