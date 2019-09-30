/**
 * @file TeamSkills.h
 *
 * This file declares all team behavior skills that are used by the current behavior.
 *
 * @author Probably many people who will not add themselves to this declaration.
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/TeammateRoles.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Tools/BehaviorControl/Framework/Skill/TeamSkill.h"

namespace TeamSkills
{
  /**
   * This skill sets the teamActivity member of the TeamBehaviorStatus.
   * @param teamActivity The team activity to set
   */
  TEAM_SKILL_INTERFACE(TeamActivity, (TeamBehaviorStatus::TeamActivity) teamActivity);

  /**
   * This skill sets the timeToReachBall member of the TeamBehaviorStatus.
   * @param timeToReachBall The time to reach ball to set
   */
  TEAM_SKILL_INTERFACE(TimeToReachBall, (const ::TimeToReachBall&) timeToReachBall);

  /**
   * This skill sets the teammateRoles member of the TeamBehaviorStatus.
   * @param teammateRoles The teammate roles to set
   */
  TEAM_SKILL_INTERFACE(TeammateRoles, (const ::TeammateRoles&) teammateRoles);

  /**
   * This skill sets the role member of the TeamBehaviorStatus.
   * @param role The role to set
   */
  TEAM_SKILL_INTERFACE(Role, (const ::Role&) role);
}
