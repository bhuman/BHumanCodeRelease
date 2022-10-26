/**
 * @file ClosestToTeammatesBall.h
 *
 * This file declares a behavior for an agent that should play the ball but doesn't see it.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"

class ClosestToTeammatesBall : public ActiveRole
{
  SkillRequest execute(const Agent& self, const Agents& teammates) override;
};
