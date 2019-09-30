/**
 * @file TeammateRoles.h
 *
 * This file declares a representation of a team role assignment in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(TeammateRoles, COMMA public BHumanMessageParticle<undefined>
{
  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override;

  int operator[](const size_t i) const;
  int& operator[](const size_t i),

  (std::vector<int>) roles, /**< The role assignment for all robots in the team (which is its index in the player set, i.e. how many active robots are behind it). */
  (int)(-1) captain, /**< The number of the robot which calculated this role assignment. */
  (unsigned)(0) timestamp, /**< The timestamp when this role assignment has been calculated. */
});
