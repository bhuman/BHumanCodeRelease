/**
 * @file FreeKickWall.h
 *
 * This file declares a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"

class FreeKickWall : public ActiveRole
{
  const float normalizeValue = 1300.f; // threshold for normalize the old Vector
  const float maxXValue = 500.f; // threshold for x Value on orthogonal line between penalty mark and ball
  const float maxYValue = 300.f; // threshold for y Value on orthogonal line between penalty mark and ball
  Vector2f currentTarget = Vector2f::Zero(); // current Target
  SkillRequest execute(const Agent& self, const Agents& teammates) override;
};
