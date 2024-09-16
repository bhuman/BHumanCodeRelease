/**
 * @file WalkKickEngine.h
 *
 * This file declares a module that provides a walk kick generator.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FootOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Tools/Motion/WalkKickType.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Framework/Module.h"
#include "RobotParts/Legs.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(KickKeyframePart,
{,
  (Vector2f)(Vector2f::Zero()) relativePositionToBallMax, /**< Max relative position to the ball for the origin of the swing foot sole. */
  (Vector2f)(Vector2f::Zero()) relativePositionToBallMin, /**< Min relative position to the ball for the origin of the swing foot sole. */
  (Vector2f)(Vector2f::Zero()) offsetSwingFootMax, /**< Max hip shift backward. */
  (Vector2f)(Vector2f::Zero()) offsetSwingFootMin, /**< Min hip shift backward. */
  (Vector2f)(Vector2f::Zero()) minStepTarget, /**< Min size of the step target. */
  (std::optional<float>) relativeBackwardStep,
  (float)(1.f) directionRatio, /**< Reach direction with this ratio. */
  (float)(0.5f) reachPositionRatio, /**< Reach the relativePositionToBall after this % part of the step. (Between 0 and 1. One step takes 250 ms)*/
  (Rangef)(Rangef(1.f, 1.f)) speedUpSwingPositionXScaling, /**< Swing foot x-translation shall reach the step target faster by this factor, scaled by the kick power. */
  (Rangef)(Rangef(1.f, 1.f)) speedUpSwingPositionYScaling, /**< Swing foot y-translation shall reach the step target faster by this factor, scaled by the kick power. */
  (bool)(false) useDefaultReachPositionRatio, /**< If true, use reachPositionRatio. */
  (bool)(false) holdXTranslationWhenMovingBackward, /**< If true: If the step target would move the swing foot backwards again, clip it. */
  (bool)(false) holdYTranslationWhenFeetTogether, /**< If true: If the step target would move the swing foot y translation back to zero, clip it. */
  (bool)(false) holdXSwingTarget, /**< If true: The swing foot x translation will not change. */
  (bool)(false) holdXSupportTarget, /**< If true: The support foot x translation will not change. */
  (bool)(false) returnXToZero, /**< Set step target for x translation to 0. */
  (bool)(false) returnYToZero, /**< Set step target for y translation to 0. */
  (bool)(false) returnRotationToZero, /**< Set step target rotation to 0. */
  (bool)(false) hold, /**< Hold current position. Only used if returnToZero is false. */
});

STREAMABLE(KickKeyframe,
{,
  (float)(0.f) reduceSwingFootHeight, /**< Swing foot height is clipped at the end of the step by this amount. */
  (float)(1.f) increaseSwingHeightFactor, /**< Increase max height of the swing foot for this step. */
  (float)(1000.f) maxSideStep, /**< Max allowed side step target. Values is based on the left foot (so only positive values make sense). */
  (bool)(false) useSlowSupportFootHeightAfterKickInterpolation, /**< Interpolate the support foot height slower back to 0 in the next walking step. */
  (bool)(false) clipMaxSpeed, /**< Clip forward step target based on last step target. */
  (float)(0.f) useSlowSwingFootHeightInterpolation, /**< Interpolate the swing foot height as if it was a large side step. */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSwingFoot, /**<  Override the previous swing foot position variables based on the last measured/requested angles? */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSupportFoot, /**<  Override the previous support foot position variables based on the last measured/requested angles? */
  (int) numOfBalanceSteps, /**< Number of balance steps that need to follow after the inWalkKick. */
  (bool)(false) useLastKeyframeForSupportFootXTranslation, /**< Use only the last keyframe to interpolate the support foot. */
  (std::vector<KickKeyframePart>) keyframes, /**< The list of swing foot poses relative to the ball, and their extra information. */
  (std::vector<WalkKickStep::LongKickParams>) jointOffsets, /**< Joint offsets. */
});

STREAMABLE(WalkKickParameters,
{,
  // All kicks are based on the left foot as kick foot
  (Rangef) maxXDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) maxYDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) additionalXDeviation, /**< Additional x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is notPrecise. */
  (Rangef) additionalYDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is notPrecise. */
  (Rangef) additionalXDeviationJustHitTheBall, /**< Additional x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is justHitTheBall. */
  (Rangef) additionalYDeviationJustHitTheBall, /**< Max x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is justHitTheBall. */
  (Angle) maxAngleDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) maxClipBeforeAbortX, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
  (Rangef) maxClipBeforeAbortY, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
  (Rangea) maxClipBeforeAbortRot, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
  (Rangef) maxClipBeforeAbortJustHitTheBallX, /**< Step targets can be clipped for so much, before the in walk kick is aborted, when precision "JustHitTheBall" is set. */
  (Rangef) maxClipBeforeAbortJustHitTheBallY, /**< Step targets can be clipped for so much, before the in walk kick is aborted, when precision "JustHitTheBall" is set. */
  (Rangea) maxClipBeforeAbortJustHitTheBallRot, /**< Step targets can be clipped for so much, before the in walk kick is aborted, when precision "JustHitTheBall" is set. */
  (Rangef) sideDeviationScalingBasedOnLastStep, /**< Min -> negative scaling, Max -> positive scaling. */
  (std::vector<KickKeyframe>) kickKeyFrame, /**< Feet pose relative to ball. */
});

STREAMABLE(DeviationValues,
{,
  (Rangef) maxXDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) maxYDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Angle) maxAngleDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
});

MODULE(WalkKickEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FootOffset),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(LibDemo),
  REQUIRES(MotionRequest),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkStepData),
  REQUIRES(TorsoMatrix),
  PROVIDES(WalkKickGenerator),
  LOADS_PARAMETERS(
  {,
    (bool) playKickSounds, /**< Say which kick is currently executed. */
    (bool) allowDiagonalKicks, /**< Diagonal kicks are allowed. */
    (Rangea) maxTurnLeftFoot, /**< Max allowed turn target. */
    (Rangef) maxForward, /**< Max allowed forward target. */
    (Rangef) maxSide, /**< Max allowed side target. */
    (Pose2f) maxSpeed, /**< Max "allowed" speed. */
    (float) maxForwardAcceleration, /**< Max "allowed" acceleration forward per step. */
    (std::vector<WalkKicks::Type>) walkKicksWithRestrictedStart, /**< These walk kicks can not start if the previous step had a large y translation. */
    (Rangef) stabilizationWalkDelay, /**< A walk delay is used to stabilize the kick, to prevent a fall. This is the min and max duration for such delay. */
    (float) restrictedYTranslationOfPreStepWithStabilization, /**< A kick != justHiTheBall is still be allowed to be executed with this high side translation change, but a stabilization in the kick is needed. */
    (Vector2f) restrictedYTranslationOfPreStep, /**< If the y translation was above this value, the kicks in walkKicksWithRestrictedStart can not start. */
    (float) sidewardsOuterForwardRestriction, /**< When executing the sidewardOuter, the previous forward step size is not allowed to be to high. */
    (Rangef) forwardLongFirstStepAnkleCompensationRange, /**< Step size of first step part for the kick foot for the InWalkKick forwardLong. */
    (Angle) forwardLongFirstStepAnkleCompensationAngle, /**< Max angle offset for the ankle pitch of the kick foot of first step part for the InWalkKick forwardLong. */
    (float) forwardLongMaxStepSize, /**< The forwardLong kick uses a smaller allowed forward step size. */
    (float) forwardLongMaxStepSizeDemo, /**< In Demo, use smaller walk step size. */
    (float) justHitTheBallMaxStepSize, /**< For justHitTheBall the step size is smaller. */
    (float) forwardTurnPreciseMaxStepSize, /**< If the forward/turn kick are request to be precise, the previous max forward step size must be lower than this value. */
    (float) turnKickPreviousMaxXStepSize, /**< When executing turn kicks, the previous allowed forward step size gets interpolated. */
    (Angle) forwardStealVFeetAngle, /**< Consider V-Shape rotation of the feet, when executing the forward steal. */
    (Vector2f) forwardStealWaitingKickPose, /**< Feet kick pose for the forward steal when no rotation is applied yet. */
    (float) forwardStealVFeetAngleFactor, /**< Factor for forwardStealVFeetAngle to start the the kick pose for the forward steal. */
    (Angle) forwardPreStepSkipMaxKickAngle, /**< Skip pre step for forward kicks, if the kick angle is small. */
    (float) forwardStealMinXAbortThreshold, /**< Abort forwardSteal if the ball is too far away. */
    (float) timeSinceBallLastSeenThreshold, /**< Abort forwardSteal if the ball was not seen for this time. */
    (Pose2f) maxClipBeforeAbortForwardSteal, /**< For the forwardSteal, use different abort thresholds after the first keyframe. */
    (float) forwardStealMaxPreviousYTranslation, /**< The needed y-translation to executed this step is not allowed to be above this threshold. */
    (float) stepDuration, /**< Assumed duration of one walking step. */
    (float) timeToHitBall, /**< Time to hit the ball in the kick step. */
    (ENUM_INDEXED_ARRAY(WalkKickParameters, WalkKicks::Type)) walkKicksList, /**< List of in walk kicks. */
  }),
});

class WalkKickEngine : public WalkKickEngineBase
{
public:
  WalkKickEngine();

private:
  OdometryData odometryAtStart; /**< Odometry at the start of the kick. */
  bool enableDrawings = false; /**< If true, drawings are calculated. */

  /** For the diagonal kicks some reference values are determined,
   * to calculate on the fly a working kick in all directions, from
   * -90deg to +90deg.
   */
  struct WalkKickLengthPair
  {
    float ballDistanceAlign = -1.f; /**< Distance to ball middle point to align the foot */
    float ballDistanceKick = -1.f; /**< Kick foot shall move this distance. */
    float kickLength = -1.f;
    std::array<float, 2> executionTime = {-1.f, -1.f};
  };

  // Array of size 2, for min and max range
  std::array<WalkKickLengthPair, 2> forwardReferenceKick;
  std::array<WalkKickLengthPair, 2> forwardLongReferenceKick;
  std::array<WalkKickLengthPair, 2> forwardAlternativReferenceKick;
  std::array<WalkKickLengthPair, 2> sideReferenceKick;

  // The sole edges. Uses to calculate the diagonal kicks
  Rangef soleForwardWithBall;
  Rangef leftSoleSideWithBall;
  Rangef rightSoleSideWithBall;

  Rangef soleForward;
  Rangef leftSoleSide;
  Rangef rightSoleSide;

  void update(WalkKickGenerator& walkKickGenerator) override;

  /**
   * If the previous walk step was too large, only allow a kick in justHitTheBall mode
   * @param isStepSizeTooLarge The previous step was too large and a kick now could be unstable
   * @param canKick A kick is allowed
   * @param stabilizationNecessary A stabilization is needed because the previous step was too large
   * @param isLeftPhase Next phase is a left phase
   * @param walkKickVariant The requested kick
   * @param lastExecutedStep Previous step size
   * @param lastStepChange Previous change in step size
   * @param lastPhase Last motion phase. Nullptr in case the current kick is updated in the current walk phase
   */
  void handleLargeStepSize(bool& isStepSizeTooLarge, bool& canKick, bool& stabilizationNecessary,
                           const bool isLeftPhase, const WalkKickVariant& walkKickVariant, const Pose2f& lastExecutedStep,
                           const Pose2f& lastStepChange, const MotionPhase* lastPhase);

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param walkKickStep The requested kick
   * @param kick All information to the requested kick
   * @param isLeftPhase Is left foot the swing foot?
   * @params lastPhase The last motion phase
   */
  bool canKickStepSize(const WalkKickStep& walkKickStep, WalkKickVariant& kick, const bool isLeftPhase, const MotionPhase& lastPhase);

  /**
   * Sets some information based on the last motion phase.
   * This information is needed when the kick is updated while being executed afterwards.
   * @param walkKickStep Requested kick information, but allowed to be modified
   * @param currentKick Requested kick information, but not allowed to be modified
   * @param lastPhase The last motion phase
   */
  void setWalkKickStepInitial(WalkKickStep& walkKickStep, const WalkKickVariant& currentKick, const MotionPhase& lastPhase);

  /**
   * Should the current kick be aborted?
   * @param lastPhase The last motion phase
   * @param walkKickStep The requested kick information
   */
  bool abortKick(const MotionPhase& lastPhase, const WalkKickStep& walkKickStep);

  /**
   * Helper function to create the next kick step.
   * Used initially when created the kick, and also afterwards to update it while being executed.
   * @param walkKickStep The last kick information
   * @param nextWalkKickStep The new kick information
   * @param lastPhase The last motion phase. Nullptr in case the kick is being updated while being executed.
   * @return False if aborted. True if calculation worked. If a kick is created, it shall abort. But when updated, it shall just keep the old values.
   */
  bool createWalkKickStep(const WalkKickStep& walkKickStep, WalkKickStep& nextWalkKickStep, const MotionPhase* lastPhase);

  /**
   * Creates the next walk phase, that executes the desired in walk kick
   * @params lastPhase The last walk phase
   * @param walkKickStep Kick information
   * @return Next walk phase. Nullpointer, if no walk phase is needed for the in walk kick or the kick was aborted
   */
  std::unique_ptr<MotionPhase> createPhaseWithNextPhase(const MotionPhase& lastPhase, const WalkKickStep& walkKickStep);

  /**
   * Can the kick be executed or is one of the step targets too much clipped?
   * @param originalPoses The original planned step targets
   * @param clippedPoses The clipped step targets
   * @param maxClipBeforeAbort Parameters to decide, if the clippedPoses differentiate too much from the originalPoses
   * @param mirror Is the kick mirrored (instead of left leg is kicking, right leg is kicking)
   * @param kick The kick information
   * @param isLeftPhase Is current phase left phase? (e.g. left leg is the swing leg)
   * @param lastPhase Last motion phase. Nullptr, in case the kick is currently updated while being executed.
   * @param lastExecutedStep Previous step size.
   * @return True if executable, else false
   */
  bool canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforeAbortX,
                  const Rangef& maxClipBeforeAbortY, const Rangea& maxClipBeforeAbortRot, const bool mirror, const WalkKickVariant& kick,
                  const bool isLeftPhase, const MotionPhase* lastPhase, const Pose2f& lastExecutedStep);

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param precisionRange Precision of the kick direction
   * @param walkKickVariant All information to the requested kick
   * @param lastExecutedStep Previous step size
   * @param precision Ball needs to be as close to the ref position as possible
   * @param isPreStep Is the current walk step a pre step?
   * @return Kick is possible
   */
  bool findBestKickExecutionDiagonal(const Rangea& precisionRange,
                                     WalkKickVariant& walkKickVariant,
                                     const Pose2f& lastExecutedStep, const KickPrecision precision,
                                     const bool isPreStep);

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param canKick previous check, if kikc is possible
   * @param precisionRange Precision of the kick direction
   * @param ballModel the ball relative to the kicking foot
   * @param walkKickVariant all information to the requested kick
   * @param lastExecutedStep Previous step size
   * @param precision Ball needs to be as close to the ref position as possible
   * @param turnKickAllowed Turn kicks are allowed?
   * @param isPreStep Is the current walk step a pre step?
   * @return Kick is possible
   */
  bool findBestKickExecution(const bool canKick, const Rangea& precisionRange,
                             const Vector2f& ballModel, WalkKickVariant& walkKickVariant,
                             const Pose2f& lastExecutedStep, const KickPrecision precision,
                             const bool turnKickAllowed, const bool isPreStep);

  /**
   * Get clip ranges based on kick and precision
   * @param maxClip Clip ranges
   * @param kick The executed kick
   * @param kickIndex Index in the kick
   * @param isJustHitTheBall Precision request is "just hit"
   */
  void getClipRanges(Rangef& maxClipX, Rangef& maxClipY, Rangea& maxClipRot, const WalkKickVariant& kick,
                     const int kickIndex, const bool isJustHitTheBall);

  /**
   * Clip step target
   * @param original Original step target
   * @param newPose New step target
   * @param isLeftPhase Is left foot the swing foot?
   * @param keyframePart Special parameters
   * @param kick The kick information
   * @param lastExecutedStep Last executed step
   * @param maxSideStep max allowed side step for this step
   */
  void clipStepTarget(Pose2f& original, Pose2f& newPose,
                      const bool isLeftPhase, const KickKeyframePart keyframePart,
                      const WalkKickVariant& kick, const Pose2f& lastExecutedStep,
                      const float maxSideStep);

  /**
   * Apply the following clipping: minimal step target, hold position
   * @param kick current kick info
   * @param isLeftPhase is the left foot the swing foot
   * @param index current kick index
   * @param stepTargets the calculated step targets for the kick
   * @param lastExecutedStep step target of the previous step
   * @param isDiagonalKick If true, skip some checks
   */
  void applyAllClipping(const WalkKickVariant& kick, const bool isLeftPhase, const int index,
                        std::vector<Pose2f>& stepTargets, Pose2f& lastExecutedStep, const bool isDiagonalKick = false);

  /**
   * Clip rotation of step target. This method is used for the step target calculation, so the correct rotation is used, but the clipping still clips on the same value.
   * Other words: The calculation knows beforhand that the rotation will be clipped and adjusts the calculation accordingly.
   * @param rotation The to be clipped rotation
   * @param isLeftPhase Is left foot the swing foot?
   */
  void clipRotation(Angle& rotation, const bool isLeftPhase);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param isLeftPhase Is left leg the swing foot
   * @param original Step targets that shall be executed, but without the clipping
   * @param poses Step targets that shall be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param kick Current kick
   * @param index Current keyframe index
   * @param ballPosition relative ball position to the swing foot
   * @param convertedOdometryRotation The odometryData matching the last ball position
   * @param lastExecutedSupportStep Last step of old support foot
   */
  void getNextStepPositions(const bool isLeftPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                            const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                            const Angle convertedOdometryRotation, const Pose2f& lastExecutedStep);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param original Same as poses (TODO copy should not be created here)
   * @param poses Step targets that shall be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param kick Current kick information
   * @param ballPosition relative ball position to the swing foot
   * @param lastStepRotation Last step rotation size
   * @param kickSole Pose of the kick foot at the start of the step (this is the actual current position of the sole!)
   */
  void getNextStepPositionsDiagonal(std::vector<Pose2f>& original, std::vector<Pose2f>& poses,
                                    std::vector<Vector2f>& stepSwingOffsets, const WalkKickVariant& kick,
                                    const Vector2f& ballPosition, const Angle lastStepRotation, const Vector2f& kickSole);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastStepInfo Information about the last motion phase
   * @param kick Current kick
   * @param index Current keyframe index
   * @param originalTargets Step targets without (some) clipping
   * @param stepTargets Step targets that will be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param lastExecutedStep Last executed step
   */
  void getNextStepPositionsWithClipping(const WalkKickStep::LastStepInfo& lastStepInfo, const WalkKickVariant& kick, const OdometryData& odometryData,
                                        const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                        std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastStepInfo Information about the last motion phase
   * @param kick Current kick
   * @param index Current keyframe index
   * @param originalTargets Step targets without (some) clipping
   * @param stepTargets Step targets that will be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param lastExecutedStep Last executed step
   */
  void getNextStepPositionsSpecialHandling(const WalkKickStep::LastStepInfo& lastStepInfo, const WalkKickVariant& kick, const OdometryData& odometryData,
                                           const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                           std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep);

  /**
   * Calculates the allowed deviations to start the kick
   * @param kick The InWalkKick
   * @param lastStep Last step target
   * @param precision If false, more deviation is allowed
   * @param isPreStep Is this a pre step?
   * @return The allowed deviations
   */
  DeviationValues getForwardTurnKickDeviation(const WalkKickVariant& kick, const Pose2f& lastStep,
                                              const KickPrecision precision, const bool isPreStep);

  /**
   * Calculates the relative position to the ball (RPB) based on the requested power
   * @param kickType The InWalkKick
   * @param index Current step keyframe index (which step)
   * @param keyframe Current keyframe index (which index of the step)
   * @param power Power of the kick [0 ... 1]
   *
   * @return Position relative to the ball
   */
  Vector2f getPowerScaledRPB(const WalkKicks::Type kickType, const int index, const int keyframe, const float power);

  /**
   * Calculates the offset scaled foot (OSF) position for the swing foot
   * @param kickType The InWalkKick
   * @param index Current step keyframe index (which step)
   * @param keyframe Current keyframe index (which index of the step)
   * @param power Power of the kick [0 ... 1]
   *
   * @return Swing foot offset position
   */
  Vector2f getPowerScaledOSF(const WalkKicks::Type kickType, const int index, const int keyframe, const float power);

  /**
   * Return the distance the kick foot must travel (for kicking the ball) or the distance it must keep to the ball (for aligning)
   */
  float getPowerScaleDiagonalKick(const float kickLength, const float ratio, const WalkKicks::Type type, const bool aligning);

  /**
   * Calculate the % for the kick keyframes, for how much time of a walk step they need to be executed
   */
  void calcStepTimingRatios(const std::vector<Pose2f>& stepTargets, const Pose2f& lastExecutedStep, std::vector<float>& stepRatio, const WalkKickVariant& kick);

  /**
   * Get the ball position relative to the zero swing foot position (the position, if the robot would stop the walk)
   * @param isLeftPhase Get the position for the left foot
   * @param scsCognition 2D transformation pose
   * @param ballTime Time for ball
   * @return The relative ball position
   */
  Vector2f getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition, const float ballTime);

  /**
   * Calculate a V-shaped step size
   * @param isLeftPhase Is the left foot the swing foot?
   * @param direction Kick direction
   * @param stealXShift Shift kick pose in x direction by this amount
   * @return The step size
   */
  Pose2f getVShapeWalkStep(const bool isLeftPhase, const Angle direction, const float stealXShift);

  /**
   * Special Check for the forwardSteal kick
   * @param kick The kick parameters
   * @param steps The to be executed step
   * @param index The current kick index
   * @return true if the forwardSteal kick should be aborted
   */
  bool forwardStealAbortCondition(const WalkKickVariant& kick, const std::vector<Pose2f>& steps, const int index);

  /**
   * Clip the planned kick direction based on the precision range;
   * @param direction The planned kick direction
   * @param walkKickVariant The kick parameters
   * @param precisionRange The precision for the kick direction
   */
  void clipKickDirectionWithPrecision(Angle& direction, const WalkKickVariant& walkKickVariant, const Rangea& precisionRange);

  /**
   * Draw the current planned end position of the swing foot in the robot coordinate system
   * When an InWalkKick is executed, all part positions are drawn too
   * @param pose The to be drawn pose
   */
  void draw(const Pose2f& pose);

  std::array<std::string, WalkKicks::numOfTypes> kickTypesSpeech; // Stores the enums converted properly so the TTS can synthesize it
};
