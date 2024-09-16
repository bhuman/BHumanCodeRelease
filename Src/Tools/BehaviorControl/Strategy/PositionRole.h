/**
 * @file PositionRole.h
 *
 * This file declares a base class for position roles.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Role.h"
#include "Tactic.h"
#include "Math/Pose2f.h"
#include "Streaming/Enum.h"

class PositionRole : public Role
{
public:
  ENUM(Type,
  {,
    goalkeeper,
    attackingGoalkeeper,
    defender,
    midfielder,
    forward,
    sacPasser,
  });

  ENUM(Side,
  {,
    unspecified,
    left,
    right,
    center,
  });

  static Role::Type toRole(Type type)
  {
    return static_cast<Role::Type>(positionRoleBegin + static_cast<unsigned>(type));
  }

  /**
   * Converts a position type to a position role.
   * @param type A position type.
   * @return The corresponding position role.
   */
  static Type fromPosition(Tactic::Position::Type type);

  /**
   * Converts a position type to a (y-)side.
   * @param type A position type.
   * @return The corresponding (y-)side.
   */
  static Side sideFromPosition(Tactic::Position::Type type);

  /**
   * Calculates the target position (actually pose) on field.
   * @param side The (y-)side on which to choose a position (only if that is unambiguous from the tactic).
   * @param basePose The base pose set by the tactic.
   * @param baseArea The region in the Voronoi diagram of the base poses in the tactic.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The target pose on field.
   */
  virtual Pose2f position(Side side, const Pose2f& basePose, const std::vector<Vector2f>& baseArea, const Agent& self, const Agents& teammates);

  /**
   * Defines tolerance thresholds relative to the target pose in which the agent should not walk.
   * @return Maximum deviations per component.
   */
  [[nodiscard]] virtual Pose2f tolerance() const;

  /**
   * Checks whether the agent should start moving if it was previously standing.
   * @param target The desired target.
   * @return Whether the agent should start moving.
   */
  [[nodiscard]] virtual bool shouldStart(const Pose2f& target) const;

  /**
   * Checks whether the agent should stop moving if it was previously moving.
   * @param target The desired target.
   * @return Whether the agent should stop moving.
   */
  [[nodiscard]] virtual bool shouldStop(const Pose2f& target) const;

  /**
   * Calculates the skill request for a position role.
   * @param self The agent which executes the role.
   * @param teammates The other agents in the team.
   * @return The skill request to walk to the desired position (or stay in place).
   */
  SkillRequest execute(const Agent& self, const Agents& teammates) final;
};
