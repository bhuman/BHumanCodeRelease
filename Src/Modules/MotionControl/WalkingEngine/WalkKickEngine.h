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
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Motion/WalkKickType.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Module/Module.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"

STREAMABLE(KickKeyframePart,
{,
  (Vector2f)(Vector2f(0.f, 0.f)) relativePositionToBallMax, /**< Max relative position to the ball for the origin of the swing foot sole. */
  (Vector2f)(Vector2f(0.f, 0.f)) relativePositionToBallMin, /**< Min relative position to the ball for the origin of the swing foot sole. */
  (Vector2f)(Vector2f(0.f, 0.f)) offsetSwingFootMax, /**< Max hip shift backward. */
  (Vector2f)(Vector2f(0.f, 0.f)) offsetSwingFootMin, /**< Min hip shift backward. */
  (Vector2f)(Vector2f(0.f, 0.f)) minStepTarget, /**< Min size of the step target. */
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
  (bool)(false) useSlowFootHeightInterpolation, /**< Interpolate the support foot height slower back to 0 in the next walking step. */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSwingFoot, /**<  Override the previous swing foot position variables based on the last measured/requested angles? */
  (WalkKickStep::OverrideFoot)(WalkKickStep::OverrideFoot::none) overrideOldSupportFoot, /**<  Override the previous support foot position variables based on the last measured/requested angles? */
  (int) numOfBalanceSteps, /**< Number of balance steps that need to follow after the inWalkKick. */
  (std::vector<KickKeyframePart>) keyframes, /**< The list of swing foot poses relative to the ball, and their extra informations. */
  (std::vector<WalkKickStep::LongKickParams>) jointOffsets, /**< Joint offsets. */
});

STREAMABLE(WalkKickParameters,
{,
  // All kicks are based on the left foot as kick foot
  (Rangef) maxXDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) maxYDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) additionalXDeviation, /**< Additional x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is false. */
  (Rangef) additionalYDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick, when alignPrecisely is false. */
  (Angle) maxAngleDeviation, /**< Max x deviation for ball relative to support foot at the start of the walk kick. */
  (Rangef) maxClipBeforAbortX, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
  (Rangef) maxClipBeforAbortY, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
  (Rangea) maxClipBeforAbortRot, /**< Step targets can be clipped for so much, before the in walk kick is aborted. */
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
  REQUIRES(InertialData),
  REQUIRES(KickInfo),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  USES(OdometryData),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkStepData),
  REQUIRES(TorsoMatrix),
  PROVIDES(WalkKickGenerator),
  LOADS_PARAMETERS(
  {,
    (bool) playKickSounds, /**< Say which kick is currently executed. */
    (Rangea) maxTurnLeftFoot, /**< Max allowed turn target. */
    (Rangef) maxForward, /** < Max allowed forward target. */
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
    (float) walkToBallForForwardStealMaxXTranslation, /**< Max allowed x-Translation step size when walking to the ball for the forward steal kick. */
    (Vector2f) forwardStealKickPose, /**< Feet kick pose for the forward steal. */
    (Vector2f) forwardStealWaitingKickPose, /**< Feet kick pose for the forward steal when no rotation is applied yet. */
    (float) forwardStealVFeetAngleFactor, /**< Factor for forwardStealVFeetAngle to start the the kick pose for the forward steal. */
    (Angle) forwardPreStepSkipMaxKickAngle, /**< Skip pre step for forward kicks, if the kick angle is small. */
    (Angle) forwardStealMinVAngleThreshold, /**< Start forwardSteal if the Angle between both legs are bigger than this value. */
    (float) forwardStealMinXAbortThreshold, /**< Abort forwardSteal if the ball is too far away. */
    (float) timeSinceBallLastSeenThreshold, /**< Abort forwardSteal if the ball was not seen for this time. */
    (Pose2f) maxClipBeforAbortForwardSteal, /**< For the forwardSteal, use different abort thresholds after the first keyframe. */
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
  float currentKickPoseShiftY;
  bool enableDrawings = false; /**< If true, drawings are calculated. */
  OdometryData updatedOdometryData;
  void update(WalkKickGenerator& walkKickGenerator) override;

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
   * @param maxClipBeforAbort Parameters to decide, if the clippedPoses differentiate too much from the originalPoses
   *
   * @return True if executable, else false
   */
  bool canExecute(const std::vector<Pose2f>& originalPoses, const std::vector<Pose2f>& clippedPoses, const Rangef& maxClipBeforAbortX,
                  const Rangef& maxClipBeforAbortY, const Rangea& maxClipBeforAbortRot, const float kickPoseShiftY, const bool mirror);

  /**
   * Clip step target
   * @param original Original step target
   * @param newPose New step target
   * @param isLeftPhase Is left foot the swing foot?
   * @param keyframePart Special parameters
   */
  void clipStepTarget(Pose2f& original, Pose2f& newPose, const Pose2f& lastStep, const bool isLeftPhase,
                      const KickKeyframePart keyframePart,
                      const WalkKicks::Type walkKickType);

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
   * @param poses Step targets that shall be executed
   * @param stepSwingOffsets Step targets for the swing foot
   * @param kick Current kick
   * @param odometryData The odometryData matching the last ball position
   * @param index Current keyframe index
   */
  void getNextStepPositions(const MotionPhase& lastPhase, std::vector<Pose2f>& poses, std::vector<Vector2f>& stepSwingOffsets,
                            const WalkKickVariant& kick, const int index, const Vector2f ballPosition,
                            std::vector<Pose2f> clipOffsets, const Angle convertedOdometryRotation);

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
   * @param alignPrecisely If false, more deviation is allowed
   *
   * @return The allowed deviations
   */
  DeviationValues getForwardTurnKickDeviation(const WalkKickVariant& kick, const Pose2f& lastStep, const bool alignPrecisely, const float kickPoseShiftY);

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
   * @return The relativ ball position
   */
  Vector2f getZeroBallPosition(const bool isLeftPhase, Pose2f& scsCognition);

  /**
   * Calculate a V-shaped step size
   * @param isLeftPhase Is the left foot the swing foot?
   * @param direction Kick direction
   * @return The step size
   */
  Pose2f getVShapeWalkStep(const bool isLeftPhase, const Angle direction);

  /**
   * Special Check for the forwardSteal kick
   * @param kick The kick parameters
   * @return true if the forwardSteal kick should be aborted
   */
  bool forwardStealAbortCondition(const WalkKickVariant& kick);

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
