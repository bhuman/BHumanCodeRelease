/**
 * @file ClosestToTeammatesBall.cpp
 *
 * This file implements a behavior for an agent that should play the ball but doesn't see it.
 *
 * @author Arne Hasselbring
 */

#include "ClosestToTeammatesBall.h"

SkillRequest ClosestToTeammatesBall::execute(const Agent&, const Agents&)
{
  return SkillRequest::Builder::observe(theTeammatesBallModel.position);
}
