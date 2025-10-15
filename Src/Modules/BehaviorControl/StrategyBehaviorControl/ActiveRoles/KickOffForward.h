/**
 * @file KickOffForward.h
 *
 * This file declares a behavior to walk into the opposing half after the kick off.
 *
 * @author Thade Struckhoff
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"

class KickOffForward : public ActiveRole
{
  const float OuterForwardX = 1000.f;
  const float centralForwardX = 1500.f;
  SkillRequest execute(const Agent& self, const Agents&) override;
};
