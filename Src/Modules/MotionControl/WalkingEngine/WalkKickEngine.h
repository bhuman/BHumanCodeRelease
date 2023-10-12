/**
 * @file WalkKickEngine.h
 *
 * This file declares a module that provides a walk kick generator.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
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
  (float)(1000.f) maxSideStep, /**< Max allowed side step target. Values is based on the left foot (so only positive values make sense). */
  (float)(1.f) directionRatio, /**< Reach direction with this ratio. */
  (float)(0.5f) reachPositionRatio, /**< Reach the relativePositionToBall after this % part of the step. (Between 0 and 1. One step takes 250 ms)*/
  (Rangef)(Rangef(1.f, 1.f)) speedUpSwingPositionScaling, /**< Swing foot shall reach the step target faster by this factor, scaled by the kick power. */
  (bool)(false) useDefaultReachPositionRatio, /**< If true, use reachPositionRatio. */
  (bool)(false) holdXTranslationWhenMovingBackward, /**< If true: If the step target would move the swing foot backwards again, clip it. */
  (bool)(false) holdYTranslationWhenFeetTogether, /**< If true: If the step target would move the swing foot y translation back to zero, clip it. */
  (bool)(false) holdXSwingTarget, /**< If true: The swing foot x translation will not change. */
  (bool)(false) holdXSupportTarget, /**< If true: The support foot x translation will not change. */
  (bool)(false) returnXToZero, /**< Set step target for x translation to 0. */
  (bool)(false) returnYToZero, /**< Set step target for y translation to 0. */
  (bool)(false) returnRotationToZero, /**< Set step target rotation to 0. */
  (bool)(false) hold, /**< Hold current position. Only used if returnToZero is false. */
  (bool)(false) clipMaxSpeed, /**< Clip forward step target based on last step target. */
});

STREAMABLE(KickKeyframe,
{,
  (float)(0.f) reduceSwingFootHeight, /**< Swing foot height is clipped at the end of the step by this amount. */
  (float)(1.f) increaseSwingHeightFactor, /**< Increase max height of the swing foot for this step. */
  (bool)(false) useSlowSupportFootHeightAfterKickInterpolation, /**< Interpolate the support foot height slower back to 0 in the next walking step. */
  (float)(0.f) useSlowSwingFootHeightInterpolation, /**< Interpolate the swing foot height as if it was a large side step. */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSwingFoot, /**<  Override the previous swing foot position variables based on the last measured/requested angles? */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSupportFoot, /**<  Override the previous support foot position variables based on the last measured/requested angles? */
  (int) numOfBalanceSteps, /**< Number of balance steps that need to follow after the inWalkKick. */
  (bool)(false) useLastKeyframeForSupportFootXTranslation, /**< Use only the last keyframe to interpolate the support foot. */
  (std::vector<KickKeyframePart>) keyframes, /**< The list of swing foot poses relative to the ball, and their extra informations. */
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
  (Rangef) sideDeviationScalingBasedOnLastStep, /**< Min -> negativ scaling, Max -> positiv scaling. */
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
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
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
    (Vector2f) restrictedYTranslationOfPreStep, /**< If the y translation was above this value, the kicks in walkKicksWithRestrictedStart can not start. */
    (float) sidewardsOuterForwardRestriction, /**< When executing the sidewardOuter, the previous forward step size is not allowed to be to high. */
    (Rangef) forwardLongFirstStepAnkleCompensationRange, /**< Step size of first step part for the kick foot for the InWalkKick forwardLong. */
    (Angle) forwardLongFirstStepAnkleCompensationAngle, /**< Max angle offset for the ankle pitch of the kick foot of first step part for the InWalkKick forwardLong. */
    (float) forwardLongMaxStepSize, /**< The forwardLong kick uses a smaller allowed forward step size. */
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
    (ENUM_INDEXED_ARRAY(WalkKickParameters, WalkKicks::Type)) walkKicksList, /**< List of in walk kicks. */
  }),
});

class WalkKickEngine : public WalkKickEngineBase
{
public:
  WalkKickEngine();

private:
  OdometryData odometryAtStart; /**< Odometry at the start of the kick. */
  WalkKickVariant currentKick; /**< Current executed kick. */
  int kickIndex; /**< Current index of the current executed kick. */
  bool enableDrawings = false; /**< If true, drawings are calculated. */
  void update(WalkKickGenerator& walkKickGenerator) override;

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param kick all information to the requested kick
   * @param isLeftPhase Is left foot the swing foot?
   * @params lastPhase the last walk phase
   */
  bool canKickStepSize(WalkKickVariant& kick, const bool isLeftPhase, const MotionPhase& lastPhase);

  /**
   * Creates the next walk phase, that executes the desired in walk kick
   * @params lastPhase the last walk phase
   * @retrun Next walk phase. Nullpointer, if no walk phase is needed for the in walk kick or the kick was aborted
   */
  std::unique_ptr<MotionPhase> createPhaseWithNextPhase(const MotionPhase& lastPhase);

  /**
   * Can the kick be executed or is one of the step targets too much clipped?
   * @param originalPoses The original planned step targets
   * @param clippedPoses The clipped step targets
   * @param maxClipBeforeAbort Parameters to decide, if the clippedPoses differentiate too much from the originalPoses
   * @param mirror Is the kick mirrored (instead of left leg is kicking, right leg is kicking)
   * @return True if executable, else false
   */
  bool canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforeAbortX,
                  const Rangef& maxClipBeforeAbortY, const Rangea& maxClipBeforeAbortRot, const bool mirror, const WalkKickVariant& kick);

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param precisionRange Precision of the kick direction
   * @param ballModel the ball relativ to the kicking foot
   * @param walkKickVariant all information to the requested kick
   * @param lastExecutedStep
   * @param precision Ball needs to be as close to the ref position as possible
   * @param turnKickAllowed Turn kicks are allowed?
   * @param isPreStep Is the current walk step a pre step?
   */
  bool findBestKickExecutionDiagonal(const Rangea& precisionRange,
                                     WalkKickVariant& walkKickVariant,
                                     const Pose2f& lastExecutedStep, const KickPrecision precision,
                                     const bool turnKickAllowed, const bool isPreStep);

  /**
   * Checks if the kick can be executed, given the current ball position. Simple threshold checks
   * @param canKick previous check, if kikc is possible
   * @param precisionRange Precision of the kick direction
   * @param ballModel the ball relativ to the kicking foot
   * @param walkKickVariant all information to the requested kick
   * @param lastExecutedStep
   * @param precision Ball needs to be as close to the ref position as possible
   * @param turnKickAllowed Turn kicks are allowed?
   * @param isPreStep Is the current walk step a pre step?
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
   * @param kick The kick
   * @param lastExecutedStep Last executed step
   */
  void clipStepTarget(Pose2f& original, Pose2f& newPose,
                      const bool isLeftPhase, const KickKeyframePart keyframePart,
                      const WalkKickVariant& kick, const Pose2f& lastExecutedStep);

  /**
   * Apply the following clipping: minimal step target, hold position
   * @param kick current kick info
   * @param isLeftPhase is the left foot the swing foot
   * @param index current kick index
   * @param stepTargets the calculated step targets for the kick
   * @param lastExecutedStep step target of the previous step
   */
  void applyAllClipping(const WalkKickVariant& kick, const bool isLeftPhase, const int index,
                        std::vector<Pose2f>& stepTargets, Pose2f& lastExecutedStep);

  /**
   * Clip rotation of step target. This method is used for the step target calculation, so the correct rotation is used, but the clipping still clips on the same value.
   * Other words: The calculation knows beforhand that the rotation will be clipped and adjusts the calculation accordingly.
   * @param rotation The to be clipped rotation
   * @param isLeftPhase Is left foot the swing foot?
   */
  void clipRotation(Angle& rotation, const bool isLeftPhase);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastPhase Last walk phase
   * @param original Step targets that shall be executed, but without the clipping
   * @param poses Step targets that shall be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param kick Current kick
   * @param index Current keyframe index
   * @param ballPosition relativ ball position to the swing foot
   * @param convertedOdometryRotation The odometryData matching the last ball position
   * @param lastExecutedSupportStep Last step of old support foot
   */
  void getNextStepPositions(const MotionPhase& lastPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                            const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                            const Angle convertedOdometryRotation, const Pose2f& lastExecutedStep);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastPhase Last walk phase
   * @param poses Step targets that shall be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param kick Current kick
   * @param index Current keyframe index
   * @param ballPosition relativ ball position to the swing foot
   * @param clipOffsets offsets for the swing foot
   * @param convertedOdometryRotation The odometryData rotation
   */
  void getNextStepPositionsDiagonal(const MotionPhase& lastPhase, std::vector<Pose2f>& original, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                                    const WalkKickVariant& kick, const int index, const Vector2f& ballPosition,
                                    const Angle lastStepRotation);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastPhase Last walk phase
   * @param kick Current kick
   * @param odometryData The odometryData matching the last ball position
   * @param index Current keyframe index
   * @param originalTargets Step targets without (some) clipping
   * @param stepTargets Step targets that will be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param lastExecutedStep Last executed step
   */
  void getNextStepPositionsWithClipping(const MotionPhase& lastPhase, const WalkKickVariant& kick, const OdometryData& odometryData,
                                        const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                        std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep);

  /**
   * Calculate the new step targets with the special handling for the forward-turn interpolation
   * @param lastPhase Last walk phase
   * @param kick Current kick
   * @param odometryData The odometryData matching the last ball position
   * @param index Current keyframe index
   * @param originalTargets Step targets without (some) clipping
   * @param stepTargets Step targets that will be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param lastExecutedStep Last executed step
   */
  void getNextStepPositionsSpecialHandling(const MotionPhase& lastPhase, const WalkKickVariant& kick, const OdometryData& odometryData,
                                           const int index, std::vector<Pose2f>& originalTargets, std::vector<Pose2f>& stepTargets,
                                           std::vector<Vector2f>& stepSwingOffsets, Pose2f& lastExecutedStep);

  /**
   * Calculates the allowed deviations to start the kick
   * @param kick The InWalkKick
   * @param lastStep Last step target
   * @param precision If false, more deviation is allowed
   * @param isPreStep Is this a pre step?
   *
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
   * @return Position relativ to the ball
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
   * Get the ball position relativ to the zero swing foot position (the position, if the robot would stop the walk)
   * @param isLeftPhase Get the position for the left foot
   * @param scsCognition 2D transformation pose
   * @param ballTime Time for ball
   * @return The relativ ball position
   */
  Vector2f getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition, const float ballTime);

  /**
   * Calculate a V-shaped step size
   * @param isLeftPhase Is the left foot the swing foot?
   * @param direction Kick direction
   * @return The step size
   */
  Pose2f getVShapeWalkStep(const bool isLeftPhase, const Angle direction);

  /**
   * Special Check for the forwardSteal kick
   * @param The requested kick
   * @param steps The to be executed step
   * @param kick The kick parameters
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
