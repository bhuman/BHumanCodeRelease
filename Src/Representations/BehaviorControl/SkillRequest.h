/**
 * @file SkillRequest.h
 *
 * This file declares the representation of the request from the strategy layer to the skill layer.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(SkillRequest,
{
  /** When adding a type for playing the ball, add them to
   * - HandlePenaltyKick
   * - HandleStrikerLostBall
   * - SkillBehaviorControl
   * - PlayBall
   */
  ENUM(Type,
  {,
    none,
    stand,
    walk,
    shoot,
    pass,
    dribble,
    block,
    mark,
    observe,
    clear,
  });

  struct Builder
  {
    // All coordinates/angles/directions are in field coordinates.
    static SkillRequest empty();
    static SkillRequest stand();
    static SkillRequest walkTo(const Pose2f& target);
    static SkillRequest shoot();
    static SkillRequest passTo(int target);
    static SkillRequest dribbleTo(Angle target);
    static SkillRequest block(const Vector2f& player);
    static SkillRequest mark(const Vector2f& player);
    static SkillRequest observe(const Vector2f& point);
    static SkillRequest clear();
  },

  (Type)(none) skill, /**< The skill that shall run. */
  (Pose2f) target, /**< The target pose, object or direction (in field coordinates). */
  (int)(-1) passTarget, /**< The number of the passed-to player. */
});
