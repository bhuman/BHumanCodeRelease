/**
 * @file Skill.h
 *
 * This file instantiates the skill system for individual robot skills.
 *
 * @author Arne Hasselbring
 */

#pragma once

struct Skill
{
};

#include "SkillDetails.h"

#define SKILL_INTERFACE(...) _SKILL_INTERFACE(SkillRegistry, __VA_ARGS__)
#define SKILL_IMPLEMENTATION(name, header, ...) _SKILL_IMPLEMENTATION(SkillRegistry, Skills, name, (header), __VA_ARGS__)
#define MAKE_SKILL_IMPLEMENTATION(...) _MAKE_SKILL_IMPLEMENTATION(Skill, __VA_ARGS__)

#include "SkillRegistryBase.h"

struct ArmMotionRequest;
struct BehaviorStatus;
struct HeadMotionRequest;
struct MotionRequest;
struct TeamTalk;

class SkillRegistry : public SkillRegistryBase
{
public:
  /**
   * Constructor.
   * @param skillConfig The name of the configuration file.
   * @param activationGraph The activation graph that can be modified by skills.
   * @param armMotionRequest The arm motion request that can be modified by skills.
   * @param behaviorStatus The behavior status that can be modified by skills.
   * @param headMotionRequest The head motion request that can be modified by skills.
   * @param motionRequest The motion request that can be modified by skills.
   */
  SkillRegistry(const char* skillConfig, ActivationGraph& activationGraph, ArmMotionRequest& armMotionRequest,
                BehaviorStatus& behaviorStatus, HeadMotionRequest& headMotionRequest, MotionRequest& motionRequest, TeamTalk& teamTalk);

  /** Destructor. */
  ~SkillRegistry();

  static thread_local SkillRegistry* theInstance; /**< The one and only instance per (cognition) process. */

  ArmMotionRequest& theArmMotionRequest; /**< The arm motion request that can be modified by skills. */
  BehaviorStatus& theBehaviorStatus; /**< The behavior status that can be modified by skills. */
  HeadMotionRequest& theHeadMotionRequest; /**< The head motion request that can be modified by skills. */
  MotionRequest& theMotionRequest; /**< The motion request that can be modified by skills. */
  TeamTalk& theTeamTalk; /**< The team talk that can be modified by skills. */
};

#define _SKILL_REGISTRY SkillRegistry
#define _SKILLS_NAMESPACE Skills
