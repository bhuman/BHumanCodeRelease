/**
 * @file Card.h
 *
 * This file instantiates the card system for individual robot cards.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "CardBase.h"

struct Card : public CardBase
{
public:
  using CardBase::CardBase;

protected:
  void setState(const char* name);

private:
  void call() override;
};

#include "CardDetails.h"

#define CARD(name, header, ...) _CARD(Card, SkillRegistry, Skills, name, (header), __VA_ARGS__)
#define MAKE_CARD(...) _MAKE_CARD(Card, __VA_ARGS__)

#include "Tools/BehaviorControl/Framework/Skill/Skill.h"

#define _CARD_REGISTRY CardRegistry
#define _CARD_SKILL_REGISTRY SkillRegistry
#define _CARD_SKILLS_NAMESPACE Skills

#include "CardRegistryBase.h"

class CardRegistry : public CardRegistryBase
{
public:
  /**
   * Constructor.
   * @param activationGraph The activation graph that can be modified by cards.
   */
  CardRegistry(ActivationGraph& activationGraph);

  /** Destructor. */
  ~CardRegistry();

  static thread_local CardRegistry* theInstance; /**< The one and only instance per (cognition) process. */
};
