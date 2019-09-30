/**
 * @file TeamSkill.h
 *
 * This file instantiates the skill system for team skills.
 *
 * @author Arne Hasselbring
 */

#pragma once

struct TeamSkill
{
};

#include "SkillDetails.h"

#define TEAM_SKILL_INTERFACE(...) _SKILL_INTERFACE(TeamSkillRegistry, __VA_ARGS__)
#define TEAM_SKILL_IMPLEMENTATION(name, header, ...) _SKILL_IMPLEMENTATION(TeamSkillRegistry, TeamSkills, name, (header), __VA_ARGS__)
#define MAKE_TEAM_SKILL_IMPLEMENTATION(...) _MAKE_SKILL_IMPLEMENTATION(TeamSkill, __VA_ARGS__)

#include "SkillRegistryBase.h"

struct TeamBehaviorStatus;

class TeamSkillRegistry : public SkillRegistryBase
{
public:
  /**
   * Constructor.
   * @param skillConfig The name of the configuration file.
   * @param activationGraph The activation graph that can be modified by skills.
   * @param teamBehaviorStatus The team behavior status that can be modified by skills.
   */
  TeamSkillRegistry(const char* skillConfig, ActivationGraph& activationGraph, TeamBehaviorStatus& teamBehaviorStatus);

  /** Destructor. */
  ~TeamSkillRegistry();

  static thread_local TeamSkillRegistry* theInstance; /**< The one and only instance per (cognition) process. */

  TeamBehaviorStatus& theTeamBehaviorStatus; /**< The behavior status that can be modified by skills. */
};

#define _SKILL_REGISTRY TeamSkillRegistry
#define _SKILLS_NAMESPACE TeamSkills
