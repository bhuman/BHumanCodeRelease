/**
 * @file TeamBehaviorControl.h
 *
 * This file declares the module that executes the team behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Module.h"
#include <string>

MODULE(TeamBehaviorControl,
{,
  REQUIRES(FrameInfo),
  REQUIRES(LibCheck),

  PROVIDES(TeamActivationGraph),
  REQUIRES(TeamActivationGraph),

  PROVIDES(TeamBehaviorStatus),

  LOADS_PARAMETERS(
  {,
    (std::string) rootCard, /**< The card that is executed directly by this module. */
  }),
});

// Include here so macros do not dismantle themself
#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"
#include "Tools/BehaviorControl/Framework/Skill/TeamSkill.h"

class TeamBehaviorControl : public TeamBehaviorControlBase
{
public:
  /** Constructor. */
  TeamBehaviorControl();

  /**
   * Creates extended module info (union of this module's info and requirements of all behavior parts (cards and skills)).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();

private:
  /**
   * Updates the team activation graph.
   * @param teamActivationGraph The provided team activation graph.
   */
  void update(TeamActivationGraph& teamActivationGraph) override;

  /**
   * Updates the team behavior status.
   * @param teamBehaviorStatus The provided team behavior status.
   */
  void update(TeamBehaviorStatus& teamBehaviorStatus) override { teamBehaviorStatus = theTeamBehaviorStatus; }

  TeamBehaviorStatus theTeamBehaviorStatus; /**< The team behavior status that is modified by the behavior. */

  TeamSkillRegistry theTeamSkillRegistry; /**< The manager of all skills. */
  TeamCardRegistry theTeamCardRegistry; /**< The manager of all cards. */
};
