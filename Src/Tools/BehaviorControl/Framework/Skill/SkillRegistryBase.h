/**
 * @file SkillRegistryBase.h
 *
 * This file declares a class that manages skill interfaces and implementations.
 * DO NOT INCLUDE THIS FILE DIRECTLY FROM OUTSIDE THIS DIRECTORY!
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <string>
#include <vector>

struct ActivationGraph;
class SkillInterfaceCreator;
class SkillInterface;
class SkillImplementationCreatorBase;
class SkillImplementationInterface;

class SkillRegistryBase
{
public:
  /**
   * Constructor.
   * @param activationGraph The activation graph that can be modified by skills.
   */
  SkillRegistryBase(ActivationGraph& activationGraph);
  /** Destructor (deliberately not virtual). */
  ~SkillRegistryBase();

  /**
   * Creates all skill implementations from a creator list and adds them to the registry.
   * @param firstCreator The anchor of the creator list.
   */
  void create(SkillImplementationCreatorBase* firstCreator);
  /** Destroys all skill implementations in the registry. */
  void destroy();

  /** Calls \c modifyParameters on all skill implementations. */
  void modifyAllParameters();

  /**
   * Calls \c preProcess on all skill implementations.
   * @param frameTime The current time from the FrameInfo.
   */
  void preProcess(unsigned frameTime);
  /** Calls \c postProcess on all skill implementations. */
  void postProcess();

  /**
   * Instantiates skill interfaces and the skill implementations that are needed and connects them.
   * @param skillConfig The name of the configuration file.
   */
  void resolveSkills(const char* skillConfig);

  /**
   * Checks whether all given skill interfaces have an implementation (and FAILs otherwise).
   * @param skillNames The names of the skills to check
   */
  void checkSkills(const std::vector<const char*>& skillNames);

  /**
   * Obtains a handle to a skill.
   * @tparam SkillType The type of the skill to obtain
   * @param skillName The name of the skill (without trailing "Skill")
   * @return A pointer to the skill (may be nullptr if the skill does not exist)
   */
  template<typename SkillType>
  SkillType* getSkill(const std::string& skillName)
  {
    return dynamic_cast<SkillType*>(getSkillInterface(skillName));
  }

  ActivationGraph& theActivationGraph; /**< The activation graph that can be modified by skills. */

  unsigned currentFrameTime = 0; /**< The frame time during the current skill executions. */
  unsigned lastFrameTime = 0; /**< The frame time during the previous skill executions. */

private:
  /**
   * Obtains a handle to a skill with a generic type.
   * @param skillName The name of the skill (without trailing "Skill")
   * @return A pointer to the skill (may be nullptr if the skill does not exist)
   */
  SkillInterface* getSkillInterface(const std::string& skillName);

  STREAMABLE(Configuration,
  {
    STREAMABLE(SkillImplementation,
    {,
      (std::string) skill, /**< The name of the skill (interface). */
      (std::string) implementation, /**< The name of the skill implementation. */
    }),
    (std::vector<SkillImplementation>) skillImplementations, /**< The list of skill-implementation pairs. */
  });

  struct SkillImplementationState
  {
    /**
     * Constructor.
     * @param skill The skill interface creator.
     */
    SkillImplementationState(SkillImplementationCreatorBase* skill);
    SkillImplementationCreatorBase* skill; /**< The skill creator. */
    SkillImplementationInterface* instance = nullptr; /**< The instance of this skill implementation (may not exist). */
  };

  struct SkillInterfaceState
  {
    /**
     * Constructor. Directly creates an instance of this interface.
     * @param skill The skill interface creator.
     */
    SkillInterfaceState(const SkillInterfaceCreator& skill);
    std::string name; /**< The name of this skill (interface). */
    SkillInterface* instance; /**< The instance of this skill interface. */
  };

  Configuration config; /**< The mapping from skill interfaces to implementations (only for skills for which there are multiple implementations). */
  std::vector<SkillImplementationState> skillImplementations; /**< The skill implementations that are known to the registry. */
  std::vector<SkillInterfaceState> skillInterfaces; /**< The skill interfaces that are known to the registry. */
};
