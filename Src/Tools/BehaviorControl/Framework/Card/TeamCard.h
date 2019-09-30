/**
 * @file TeamCard.h
 *
 * This file instantiates the card system for team cards.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "CardBase.h"

struct TeamCard : public CardBase
{
public:
  using CardBase::CardBase;

protected:
  void setState(const char* name);

private:
  void call() override;
};

#include "CardDetails.h"

#define TEAM_CARD(name, header, ...) _CARD(TeamCard, TeamSkillRegistry, TeamSkills, name, (header), __VA_ARGS__)
#define MAKE_TEAM_CARD(...) _MAKE_CARD(TeamCard, __VA_ARGS__)

#include "Tools/BehaviorControl/Framework/Skill/TeamSkill.h"

#define _CARD_REGISTRY TeamCardRegistry
#define _CARD_SKILL_REGISTRY TeamSkillRegistry
#define _CARD_SKILLS_NAMESPACE TeamSkills

#include "CardRegistryBase.h"

class TeamCardRegistry : public CardRegistryBase
{
public:
  /**
   * Constructor.
   * @param activationGraph The activation graph that can be modified by cards.
   */
  TeamCardRegistry(ActivationGraph& activationGraph);

  /** Destructor. */
  ~TeamCardRegistry();

  static thread_local TeamCardRegistry* theInstance; /**< The one and only instance per (cognition) process. */
};
