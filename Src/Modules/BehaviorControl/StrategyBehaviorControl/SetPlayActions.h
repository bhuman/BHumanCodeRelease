/**
 * @file SetPlayActions.h
 *
 * This file declares set play actions.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/SetPlay.h"

class ShotAction : public SetPlay::Action::Implementation
{
  void reset() override;

  bool isDone(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) const override;

  SkillRequest execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) override;

  unsigned timeWhenActionStarted = 0;
};

class PassAction : public SetPlay::Action::Implementation
{
  void reset() override;

  bool isDone(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) const override;

  SkillRequest execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) override;

  int lastSelectedPassTarget = -1;
};

class WaitAction : public SetPlay::Action::Implementation
{
  bool isDone(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) const override;

  SkillRequest execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) override;
};

class MarkAction : public SetPlay::Action::Implementation
{
  SkillRequest execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) override;
};

class PositionAction : public SetPlay::Action::Implementation
{
  SkillRequest execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) override;
};
