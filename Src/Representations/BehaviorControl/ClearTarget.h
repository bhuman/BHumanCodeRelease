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
  FUNCTION(Vector2f()) getTarget; /**< Returns the dribbleTarget */
  FUNCTION(void()) reset; /**< resets some variables */
  FUNCTION(Angle()) getAngle; /**< Returns the dribbleAngle */
  FUNCTION(KickInfo::KickType()) getKickType; /**< Returns the best kickType */
  FUNCTION(float()) getRating, /**< Returns the xG-values times fieldFactor */
});
