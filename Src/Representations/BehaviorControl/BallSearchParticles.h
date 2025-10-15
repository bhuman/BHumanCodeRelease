/**
 * @file BallSearchParticles.h
 *
 * This file declares a representation that describes places on the field in which the ball should be searched for.
 *
 * @author Moritz Oppermann
 */

#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

struct Agent;

STREAMABLE(BallSearchParticles,
{
  FUNCTION(bool(const Agent& agent)) anyParticlesInRegion;
  FUNCTION(Vector2f(const Agent& agent)) positionToSearch;
  std::vector<Vector2f> particles,
});
