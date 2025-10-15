/**
 * @file ClosestToTeamBall.cpp
 *
 * This file implements a behavior for an agent that should play the ball but doesn't see it.
 *
 * @author Arne Hasselbring
 */

#include "ClosestToTeamBall.h"

SkillRequest ClosestToTeamBall::execute(const Agent&, const Agents&)
{
  return SkillRequest::Builder::observe(theFieldBall.recentBallPositionOnField());
}
