/**
 * @file BallSearch.cpp
 *
 * This file implements a ball search behavior.
 *
 * @author Arne Hasselbring, Sina Schreiber
 */

#include "BallSearch.h"

BallSearch::BallSearch() :
  Cabsl(&activationGraph)
{
}

void BallSearch::preProcess()
{
  beginFrame(theFrameInfo.time);
}

void BallSearch::postProcess()
{
  endFrame();
  MODIFY("behavior:BallSearch", activationGraph);
}

SkillRequest BallSearch::execute(const Agent& agent, const Agents& otherAgents)
{
  this->agent = &agent;
  agents = otherAgents;
  // Execute behavior
  Root();
  this->agent = nullptr;

  return skillRequest;
}
