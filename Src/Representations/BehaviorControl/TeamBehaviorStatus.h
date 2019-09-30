/**
 * @file TeamBehaviorStatus.h
 *
 * This file declares a representation of the team behavior status in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/TeammateRoles.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/Communication/BHumanTeamMessageParts/BHumanMessageParticle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(TeamBehaviorStatus, COMMA public BHumanMessageParticle<idTeamBehaviorStatus>
{
  ENUM(TeamActivity,
  {,
    noTeam,
  });
  static constexpr TeamActivity numOfTeamActivities = numOfTeamActivitys;

  /** BHumanMessageParticle functions */
  void operator>>(BHumanMessage& m) const override;
  void operator<<(const BHumanMessage& m) override,

  (TeamActivity)(noTeam) teamActivity,
  (TimeToReachBall) timeToReachBall,
  (TeammateRoles) teammateRoles,
  (Role) role,
});

inline void TeamBehaviorStatus::operator>>(BHumanMessage& m) const
{
  m.theBHumanStandardMessage.teamActivity = static_cast<unsigned char>(teamActivity);
  timeToReachBall >> m;
  teammateRoles >> m;
  role >> m;
}

inline void TeamBehaviorStatus::operator<<(const BHumanMessage& m)
{
  if(!m.hasBHumanParts)
  {
    role.playBall = m.theMixedTeamHeader.wantsToPlayBall;
    return;
  }
  teamActivity = static_cast<TeamActivity>(m.theBHumanStandardMessage.teamActivity);
  timeToReachBall << m;
  teammateRoles << m;
  role << m;
}
