/**
 * @file DribbleTarget.h
 *
 * This file defines a representation that calculates the ball position after the execution of the DribbleToGoal-Skill.
 *
 * @author Nico Holsten
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Streaming/Function.h"

STREAMABLE(DribbleTarget,
{
  FUNCTION(Angle(const Vector2f& ballPosition)) calculateDribbleAngle; /**< Calculates the dribbleAngle */
  FUNCTION(Vector2f(const Vector2f& ballPosition)) getTarget, /**< Returns the dribbleTarget */
});
