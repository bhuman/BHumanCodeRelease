/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Declaration of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#pragma once

#include "Platform/BHAssert.h"
#include "RoleCheck.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"

/**
 * @struct Role
 * Representation of a robot's behavior role
 */
STREAMABLE(Role,
{
  /** The different roles */
  ENUM(RoleType,
  {,
    undefined,
    keeper,
    attackingKeeper,
    striker,
    defender,
    supporter,
    bishop,
    penaltyStriker,
    penaltyKeeper,
    none,
  });

  static Role::RoleType fromBHulksRole(const B_HULKs::Role role);
  static B_HULKs::Role toBHulksRole(const Role::RoleType type);

  bool isGoalkeeper() const;

  CHECK_OSCILLATION(role, RoleType, undefined, "Role", 5000)
  /** Draws the current role next to the robot on the field view (in local robot coordinates) */
  void draw() const,

  /** Instance of role */
  (RoleType)(undefined) role,
  (RoleType)(undefined) lastRole,
});

STREAMABLE(TeammateRoles, COMMA public BHumanMessageParticle<idTeammateRoles>
{
  /** BHumanMessageParticle functions */
  void operator >> (BHumanMessage& m) const override;
  void operator << (const BHumanMessage& m) override;

  Role::RoleType operator [](const size_t i) const;
  Role::RoleType& operator [](const size_t i);
  static const char* getName(Role::RoleType e),

  (std::vector<Role::RoleType>) roles,
});
