/**
 * @file LibObstacle.h
 *
 * This file defines a representation that contains information about obstacles
 *
 * @author Daniel Krause
 */

#pragma once
#include "Streaming/Function.h"
#include "Math/Pose2f.h"
#include "Streaming/Enum.h"

STREAMABLE(LibObstacle,
{
  FUNCTION(bool(const bool left)) isObstacleOnSide;

  FUNCTION(bool(const Vector2f& target, const float pathWidth, const float maxDistance)) isObstacleInPath
  ,
});
