/**
 * @file LibWalk.h
 *
 * This file defines a representation that allows to recalculate walktargets to avoid obstacles, fieldborder and penaltyarea
 *
 * @author Daniel Krause
 */

#pragma once
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Function.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE(LibWalk,
{
  FUNCTION(MotionRequest::ObstacleAvoidance(const Pose2f& target, bool rough, bool disableObstacleAvoidance)) calcObstacleAvoidance,
});
