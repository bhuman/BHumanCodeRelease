/**
 * @file Defender.h
 *
 * This file declares the defender role.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Streaming/AutoStreamable.h"

class Defender : public PositionRole
{
  STREAMABLE(Parameters,
  {,
    (float)(500.f) minXDiffToBeClearForward, /**< The minimum radius offset to the back circle to be considered actually forward. */
    (float)(600.f) defenderRoyaleDistanceToGoalArea, /**< The minimum distance that a defender must have to the goal area. */
    (float)(850.f) defenderRoyaleForwardDistanceToBall, /**< The minimum distance that a forward defender must have to the ball. */
    (float)(1.3f) standOffsetMultiplierToAdjustGoalieDefenderLineDistance, /**< A factor. */
    (Rangef)(2500.f, 6500.f) ballDistanceInterpolationRange, /**< Interpolate thresholds based on the distance to the ball. */
    (Rangea)(8_deg, 15_deg) rotationThresholdRange, /**< Interpolate rotation threshold between these values. */
    (Rangef)(250.f, 350.f) translationXThresholdRange, /**< Interpolate translation threshold between these values. */
    (Rangef)(150.f, 250.f) translationYThresholdRange, /**< Interpolate translation threshold between these values. */
    (Angle)(3_deg) shouldStopRotation, /**< Stop walking if orientation to the target is less than this value. */
    (Vector2f)(100.f, 50.f) shouldStopTranslation, /**< Stop walking if translation to the target is less than this value. */
    (float)(3000.f) annotationTime, /**< Wait this long between annotations. */
  });

  void preProcess() override;

  Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agents& teammates) override;

  Pose2f tolerance() const override;

  bool shouldStop(const Pose2f& target) const override;

  /**
   * Calculates the best position of the royale defender.
   * @param isLeftDefender Whether the robot should position left (right otherwise).
   * @return The royale defender position.
   */
  Vector2f calcDefenderRoyalePosition(const bool isLeftDefender) const;

  /**
   * Returns the position it is given, but creates drawings for it.
   * @param position The position to return.
   * @return The position this function is given.
   */
  const Vector2f& returnPositionWithDraw(const Vector2f& position) const;

  /**
   * Calculates whether this robot should take a forward position.
   * @param radius The resulting distance from the own goal center.
   * @param isLeftDefender Whether this defender is left (right otherwise).
   * @param secondDefenderNumber The number of the other defender.
   * @param maxRadius The maximum radius that a defender can have.
   * @param minRadius The minimum radius that a defender must have.
   * @param ballPositionField The position of the ball.
   * @return Whether the defender should be forward.
   */
  bool isRoyalePositionForward(float& radius, const bool isLeftDefender, const Agent* otherDefender,
                               const float maxRadius, const float minRadius, const Vector2f& ballPositionField) const;

  /**
   * Calculates a BOb line.
   * @param isLeftDefender Whether the BOb line is calculated for a left defender.
   * @param radius The radius from the goal center at which the defender should be.
   * @param bobLine The resulting line is added to this list.
   */
  void calcDefenderBObLine(const bool isLeftDefender, const float radius, std::vector<Geometry::Line>& bobLine) const;

  /**
   * Calculates if the defender should be left.
   * @param wasLeft Whether the defender was left in the last frame.
   * @return true to position left (false to position right).
   */
  bool calcDefenderRoyaleIsLeft(const bool wasLeft) const;

  bool returnIsLeftWithDraw(const bool leftArrow) const;

  Parameters p;

  const Agent* otherDefender = nullptr;
  const Agent* goalkeeper = nullptr;
  unsigned int annotationTimestamp = 0;
};
