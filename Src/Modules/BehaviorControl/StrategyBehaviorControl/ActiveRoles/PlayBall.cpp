/**
 * @file PlayBall.cpp
 *
 * This file implements the default ball playing behavior that selects the action to be executed.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "PlayBall.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Debugging/Modify.h"
#include <array>
#include <cmath>

void PlayBall::reset()
{
  lastPassTarget = 0;
}

void PlayBall::preProcess()
{
  MODIFY("parameters:behavior:PlayBall", p);
  DECLARE_DEBUG_DRAWING("behavior:PlayBall:ratings", "drawingOnField");
}

SkillRequest PlayBall::execute(const Agent& self, const Agents& teammates)
{
  return p.ignoreObstacles ? executeLegacy(self, teammates) : smashOrPass(self, teammates);
}

SkillRequest PlayBall::smashOrPass(const Agent& self, const Agents& teammates)
{
  if(p.alwaysShoot)
    return SkillRequest::Builder::shoot();

  // Calculate a penalty for changing the decision of the shoot or pass target based on the distance to the ball and the penalty value range
  const float decisionPenalty = mapToRange(self.ballPosition.norm(), p.ballDistMaxPenalty, p.ballDistMinPenalty, p.maxPenalty, p.minPenalty);

  // Get the estimated probability of shooting a goal from the current ball position
  // TODO: Use ballEndPosition instead?
  const Vector2f ballPosition = self.pose * self.ballPosition;
  float maxValue = theExpectedGoals.getRating(ballPosition);
  float maxPassDistance = p.maxPassDistance;
  if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    // Minimize the own goal rating because the rules do not allow this robot to perform a direct kick.
    maxValue = p.minRating;
    maxPassDistance = theGameState.isGoalKick() ? p.maxGoalKickDistance : p.maxFreeKickDistance;
  }
  draw(ballPosition, goalPosition, goalPosition, maxValue, decisionPenalty, lastPassTarget > 0);
  // Apply the decision penalty when this action is different from the last frame
  if(lastPassTarget > 0)
    maxValue -= decisionPenalty;
  if(maxValue > p.shootThreshold)
  {
    lastPassTarget = -1;
    return SkillRequest::Builder::shoot();
  }

  const Agent* passTarget = nullptr;
  // Find a pass target i.e. the teammate with the highest rating for a successful pass (from the current ball position) and a following goal shot (from the teammate's current position)
  for(const Agent* agent : teammates)
  {
    if(agent->position == Tactic::Position::Type::goalkeeper)
      continue;
    Vector2f agentPosition = agent->currentPosition;
    const float passDistance = (agentPosition - ballPosition).norm();
    if(passDistance < p.minPassDistance ||
       passDistance > maxPassDistance)
      continue;

    Vector2f bestAgentPosition = agentPosition;
    float agentValue = 0.f;
    // Sample and evaluate a given number of positions in a given distance on the line from the agent's current position to the opponent's goal
    const Geometry::Line agentGoalLine(agentPosition, (goalPosition - agentPosition).normalized(p.sampleDistance));
    for(int t = p.sampleSteps.min; t <= p.sampleSteps.max; t++)
    {
      const Vector2f samplePosition = agentGoalLine.base + t * agentGoalLine.direction;
      // The teammate's rating is the estimated probability that the ball would reach the given target position when passed AND the teammate would score a direct goal from the pass target position
      const float sampleValue = thePassEvaluation.getRating(samplePosition) *
                                theExpectedGoals.getRating(samplePosition) *
                                theExpectedGoals.getOpponentRating(samplePosition);
      if(sampleValue > agentValue)
      {
        agentValue = sampleValue;
        bestAgentPosition = samplePosition;
      }
    }
    // TODO: if p.lookAhead we should also consider the agent's estimated future position given by:
    // const Vector2f futurePosition = Teammate::getEstimatedPosition(agent->lastKnownPose, agent->lastKnownTarget, agent->lastKnownSpeed, theFrameInfo.getTimeSince(agent->lastKnownTimestamp) + p.lookAheadTime);

    // Apply the decision penalty when this action is different from the last frame
    const bool applyDecisionPenalty = lastPassTarget && lastPassTarget != agent->number;
    draw(ballPosition, agentPosition, bestAgentPosition, agentValue, decisionPenalty, applyDecisionPenalty);
    if(applyDecisionPenalty)
      agentValue -= decisionPenalty;
    if(agentValue > maxValue)
    {
      maxValue = agentValue;
      passTarget = agent;
    }
  }
  if(passTarget)
  {
    lastPassTarget = passTarget->number;
    return SkillRequest::Builder::passTo(passTarget->number);
  }
  else if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    // Here, the pass skill request is set with an invalid pass target on purpose. This way, the waiting states of the pass skill are utilized during setplays. Outside of setplays, the shoot skill request is set and the robot will dribble immediately without waiting behind the ball.
    lastPassTarget = -1;
    return SkillRequest::Builder::passTo(0);
  }
  lastPassTarget = maxValue > p.minRating ? -1 : 0;
  return SkillRequest::Builder::shoot();
}

SkillRequest PlayBall::executeLegacy(const Agent& self, const Agents& teammates)
{
  if(!p.alwaysShoot)
  {
    const Vector2f ballPosition = self.pose * self.ballPosition;

    float xGOpt = theExpectedGoals.xG(ballPosition);
    draw(ballPosition, goalPosition, goalPosition, xGOpt, 0.f, false);
    if(xGOpt > p.shootThreshold)
      return SkillRequest::Builder::shoot();
    xGOpt += p.passImprovement;
    const Agent* ptOpt = nullptr;
    for(const Agent* agent : teammates)
    {
      const float passDistance = (agent->currentPosition - ballPosition).norm();
      if(passDistance < p.minPassDistance ||
         passDistance > p.maxPassDistance)
        continue;
      const float passTargetxG = theExpectedGoals.xG(agent->currentPosition);
      draw(ballPosition, agent->currentPosition, agent->currentPosition, passTargetxG, 0.f, false);
      if(passTargetxG > xGOpt)
      {
        xGOpt = passTargetxG;
        ptOpt = agent;
      }
    }
    if(ptOpt)
      return SkillRequest::Builder::passTo(ptOpt->number);
  }
  return SkillRequest::Builder::shoot();
}

void PlayBall::draw([[maybe_unused]] const Vector2f& ballPosition, [[maybe_unused]] const Vector2f& agentPosition, [[maybe_unused]] const Vector2f& bestPosition, [[maybe_unused]] const float value, [[maybe_unused]] const float penalty, const bool applyPenalty)
{
  COMPLEX_DRAWING("behavior:PlayBall:ratings")
  {
    float maxPassDistance = theGameState.isFreeKick() && theGameState.isForOwnTeam() ?
                            (theGameState.isGoalKick() ? p.maxGoalKickDistance : p.maxFreeKickDistance) :
                            p.maxPassDistance;
    CIRCLE("behavior:PlayBall:ratings", ballPosition.x(), ballPosition.y(), maxPassDistance, 20, Drawings::solidPen, ColorRGBA::violet, Drawings::noPen, ColorRGBA::violet);
    LINE("behavior:PlayBall:ratings", ballPosition.x(), ballPosition.y(), agentPosition.x(), agentPosition.y(), 20, Drawings::dashedPen, applyPenalty ? ColorRGBA::violet : ColorRGBA::blue);
    DRAW_TEXT("behavior:PlayBall:ratings", bestPosition.x(), bestPosition.y(), 250, ColorRGBA::blue, value);
    if(applyPenalty)
      DRAW_TEXT("behavior:PlayBall:ratings", bestPosition.x(), bestPosition.y() - 250, 250, ColorRGBA::violet, value - penalty);
    const Geometry::Line agentGoalLine(agentPosition, (goalPosition - agentPosition).normalized(p.sampleDistance));
    for(int t = p.sampleSteps.min; t <= p.sampleSteps.max; t++)
    {
      const Vector2f samplePosition = agentGoalLine.base + t * agentGoalLine.direction;
      LARGE_DOT("behavior:PlayBall:ratings", samplePosition.x(), samplePosition.y(), ColorRGBA::blue, applyPenalty ? ColorRGBA::violet : ColorRGBA::blue);
    }
  }
}
