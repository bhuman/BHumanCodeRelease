/**
 * @file ClearTarget.h
 *
 * This file defines a representation that calculates the ball position after the execution of the ClearBall-Skill.
 *
 * @author Nico Holsten
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Eigen.h"
#include "Streaming/Function.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Representations/Configuration/KickInfo.h"

STREAMABLE(ClearTarget,
{
  FUNCTION(void()) update; /**< Update clear target */
  FUNCTION(void()) reset, /**< resets some variables */

  (float)(20000.f) kickLength, /**< Kick length of last kick. */
  (Angle)(0_deg) kickAngle, /**< Kick angle in global coordinates. */
  (Vector2f)(Vector2f::Zero()) kickTarget,
  (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType, /**< The kick type. */
  (float)(-1.f) rating,  /**< The xG-values times fieldFactor */
  (bool)(false) isValid, /**< False if not updated this frame. */
});
