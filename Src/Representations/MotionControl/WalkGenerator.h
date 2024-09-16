/**
 * @file WalkGenerator.h
 *
 * @author Arne Hasselbring
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Math/Pose2f.h"
#include "Math/Pose3f.h"
#include "Tools/Motion/MotionPhase.h"
#include "Math/Range.h"
#include "Tools/Motion/WalkKickStep.h"
#include <memory>

STREAMABLE(WalkGenerator,
{
  using CreateNextPhaseCallback = std::function<std::unique_ptr<MotionPhase>(const MotionPhase&, const WalkKickStep&)>;

  /**
   * @param lastPhase The previous MotionPhase
   * @param stepTarget The requested walk step
   *
   * @return Is the next walk phase will be a left or right phase.
   */
  FUNCTION(bool(const MotionPhase& lastPhase, const Pose2f& stepTarget)) isNextLeftPhase;
  /**
   * Get the allowed rotation to turn in place with a walk step (z-axis; yaw direction)
   *
   * @param isLeftPhase Plan with the next swing foot being the left one
   * @param walkSpeedRatio The walk speed in % (for rotation and translation)
   *
   * @return the min and max allowed rotation
   */
  FUNCTION(Rangea(const bool isLeftPhase, const Pose2f& walkSpeedRatio)) getRotationRange;
  /**
   * Normally, more rotation means less possible translation (because of stability reasons).
   * Therefore given a planned walk step, what is the maximal possible walk step rotation,
   * which would NOT further reduce the walk step translation?
   *
   * @param isLeftPhase Plan with the next swing foot being the left one
   * @param walkSpeedRatio The walk speed in % (for rotation and translation)
   * @param step The planned walk step
   * @param isFastWalk Is fast walk allowed? Meaning, more rotation with translation is allowed
   * @param lastPhase The previous MotionPhase
   * @param ignoreXTranslation Do not consider the x-Translation
   * @param clipTranslation Use the translation polygon to clip step, to use a preview of the next step size. Otherwise use step as it is
   *
   * @return The min and max allowed rotation, that would not result in a translation reduction
   */
  FUNCTION(Rangea(const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step, const bool isFastWalk,
                  const MotionPhase& lastPhase, const bool ignoreXTranslation, const bool isMaxPossibleStepSize)) getStepRotationRange;

  FUNCTION(Rangea(const bool isLeftPhase, const Pose2f& walkSpeedRatio, Vector2f step, const bool isFastWalk,
                  const std::vector<Vector2f>& translationPolygon, const bool ignoreXTranslation, const bool isMaxPossibleStepSize)) getStepRotationRangeOther;
  /**
   * Given the last walk step size and the requested walk step rotation, return the min and max allowed translation as a convex polygon.
   * With that, the step size can be clipped.
   *
   * @param isLeftPhase Plan with the next swing foot being the left one
   * @param rotation The walk step rotation (z-axis; yaw)
   * @param lastPhase The previous MotionPhase
   * @param walkSpeedRatio The walk speed in % (for rotation and translation)
   * @param translationPolygon The convex polygon, which must contain the point (0,0)
   * @param translationPolygonNoCenter The convec polygon, which does not need to contain the point (0,0). Can happen after InWalkKicks
   * @param fastWalk Is fast walk allowed? Meaning, more rotation with translation is allowed
   * @param useMaxPossibleStepSize Use the actual max allowed translation polygon. ONLY USE IN SPECIAL CASES!
   */
  FUNCTION(void(const bool isLeftPhase, const float rotation, const MotionPhase& lastPhase, const Pose2f& walkSpeedRatio,
                std::vector<Vector2f>& translationPolygon, std::vector<Vector2f>& translationPolygonNoCenter, const bool fastWalk,
                const bool useMaxPossibleStepSize)) getTranslationPolygon;
  /**
   * A function to get a translation polygon (like getTranslationPolygon),
   * but here all accelerations are ignored and the min and max translations are partially given.
   * This is useful to for example calculate more optimized walk directions.
   *
   * @param isLeftPhase Plan with the next swing foot being the left one
   * @param rotation The walk step rotation (z-axis; yaw)
   * @param walkSpeedRatio The walk speed in % (for rotation and translation)
   * @param translationPolygon The convex polygon, which must contain the point (0,0)
   * @param backRight Min walk translation
   * @param frontLeft Max walk translation
   * @param useFastWalk Is fast walk allowed? Meaning, more rotation with translation is allowed
   * @param useMaxPossibleStepSize generate polygon with theortical bigger step size?
   */
  FUNCTION(void(const bool isLeftPhase, const Angle rotation, const Pose2f& walkSpeedRatio, std::vector<Vector2f>& translationPolygon,
                Vector2f backRight, Vector2f frontLeft, const bool useFastWalk, const bool useMaxPossibleStepSize)) generateTranslationPolygon;
  /**
   * Create a WalkPhase
   *
   * @param step The walk step size
   * @param lastPhase Last MotionPhase
   * @param delay A delay before the WalkPhase starts
   *
   * @return The WalkPhase
   */
  FUNCTION(std::unique_ptr<MotionPhase>(const Pose2f& step, const MotionPhase& lastPhase, float delay)) createPhase;
  /**
   * Create a WalkPhase as InWalkKick
   *
   * @param walkKickStep All information about the InWalkKick
   * @param lastPhase Last MotionPhase
   * @param createNextPhaseCallback All information about the InWalkKick, after the WalkPhase is done (for InWalkKicks with multiple walk steps)
   * @param delay A delay before the WalkPhase starts
   *
   * @return The WalkPhase with an InWalkKick
   */
  FUNCTION(std::unique_ptr<MotionPhase>(const WalkKickStep& walkKickStep, const MotionPhase& lastPhase, const CreateNextPhaseCallback& createNextPhaseCallback, float delay)) createPhaseWithNextPhase;
  /**
   * Gets information about the start positions of the feet
   *
   * @param lastPhase The previous MotionPhase
   * @return The offsets of the left foot, right foot and the previous step size (<left,right,step>)
   */
  FUNCTION(std::tuple<Pose2f, Pose2f, Pose2f>(const MotionPhase& lastPhase)) getStartOffsetOfNextWalkPhase;
  /**
   * Gets information about the actual previous executed step size (similar to the requested step size, but the motors are not perfect :) )
   *
   * @param lastPhase The previous MotionPhase
   * @return The executed step size
   */
  FUNCTION(Pose2f(const MotionPhase& lastPhase)) getLastStepChange;
  /**
   * @return Was the last WalkPhase a left phase (left foot was swing foot)?
   */
  FUNCTION(bool(const MotionPhase& lastPhase)) wasLastPhaseLeftPhase;
  /**
   * @return Was the last WalkPhase an InWalkKick?
   */
  FUNCTION(bool(const MotionPhase& lastPhase)) wasLastPhaseInWalkKick;
  /**
   * Gets information about the step and walk kick in the previous MotionPhase.
   *
   * @param walkKickStep InWalkKick information
   * @param step Previous requested step size
   * @param lastPhase The previous MotionPhase
   */
  FUNCTION(void(WalkKickStep& walkKickStep, Pose2f& step, const MotionPhase& lastPhase)) getLastWalkPhase;
  /**
   * Is a WalkDelayPhase possible?
   *
   * @param lastPhase The last MotionPhase
   */
  FUNCTION(bool(const MotionPhase& lastPhase, const float delay, const Pose2f& stepTarget, const bool isKick)) isWalkDelayPossible,
});
