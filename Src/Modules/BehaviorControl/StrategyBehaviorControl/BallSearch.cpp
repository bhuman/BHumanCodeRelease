/**
 * @file BallSearch.cpp
 *
 * This file implements a ball search behavior.
 *
 * @author Arne Hasselbring
 */

#include "BallSearch.h"


void BallSearch::preProcess()
{
  const float usedRefereeBallPlacementAccuracy = refereeBallPlacementAccuracy + (ballUnknown ? 0.f : 200.f);
  Vector2f leftBallPosition = theGameState.isCornerKick() ? leftOpponentCorner : leftOpponentGoalCorner;
  Vector2f rightBallPosition = theGameState.isCornerKick() ? rightOpponentCorner : rightOpponentGoalCorner;
  if (theTeammatesBallModel.isValid)
  {
    const bool teammatesBallModelIsInCorner = std::min((leftBallPosition - theTeammatesBallModel.position).squaredNorm(), (rightBallPosition - theTeammatesBallModel.position).squaredNorm()) < sqr(usedRefereeBallPlacementAccuracy);
    if (teammatesBallModelIsInCorner || theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > refereeBallPlacementDelay)
    {
      teammatesBallModelInCorner = true;
    }
    else
    {
      teammatesBallModelInCorner = false;
    }
  }
  ballModelIsInOneCorner = ((leftBallPosition - theFieldBall.positionOnField).squaredNorm() < sqr(usedRefereeBallPlacementAccuracy)) || ((rightBallPosition - theFieldBall.positionOnField).squaredNorm() < sqr(usedRefereeBallPlacementAccuracy));
}

SkillRequest BallSearch::execute(const Agent& agent, const Agents& otherAgents)
{
  if (agent.isGoalkeeper) {
    return SkillRequest::Builder::walkTo(agent.basePose);
  }
  Vector2f cornerRelative;
  Vector2f targetRelative;

  if (((theGameState.isCornerKick() && theGameState.isForOwnTeam()) ||
       (theGameState.isGoalKick() && theGameState.isForOpponentTeam())) &&
      !(agent.position == Tactic::Position::defender ||
        agent.position == Tactic::Position::defenderL ||
        agent.position == Tactic::Position::defenderR))
  {
    Vector2f leftBallPosition = theGameState.isCornerKick() ? leftOpponentCorner : leftOpponentGoalCorner;
    Vector2f rightBallPosition = theGameState.isCornerKick() ? rightOpponentCorner : rightOpponentGoalCorner;
    if (agent.position == Tactic::Position::forward)
    {
      float distanceToLeftCorner = (leftBallPosition - theRobotPose.translation).squaredNorm();
      return SkillRequest::Builder::observe(distanceToLeftCorner < (rightBallPosition - theRobotPose.translation).squaredNorm() ?
        leftBallPosition : rightBallPosition);
    }

    if (Geometry::isPointInsidePolygon(leftBallPosition, agent.baseArea) ||
      std::find(agent.baseArea.begin(), agent.baseArea.end(), leftBallPosition) != agent.baseArea.end())
    {
      return SkillRequest::Builder::observe(leftBallPosition);
    }
    else if (Geometry::isPointInsidePolygon(rightBallPosition, agent.baseArea) ||
      std::find(agent.baseArea.begin(), agent.baseArea.end(), rightBallPosition) != agent.baseArea.end())
    {
      return SkillRequest::Builder::observe(rightBallPosition);
    }
    bool oneForward = false;
    const Agent* forwardAgent;
    for (auto agent : otherAgents) {
      if (agent->position == Tactic::Position::forward) {
        oneForward = true;
        forwardAgent = agent;
      }
    }
    if (oneForward) {
      float distanceToLeftCorner = (leftBallPosition - forwardAgent->currentPosition).squaredNorm();
      return SkillRequest::Builder::observe(distanceToLeftCorner < (rightBallPosition - theRobotPose.translation).squaredNorm() ?
        rightBallPosition : leftBallPosition);
    }
    else {
      return SkillRequest::Builder::walkTo(agent.basePose);
    }
  }
  else if (((theGameState.isCornerKick() && theGameState.isForOpponentTeam()) ||
            (theGameState.isGoalKick() && theGameState.isForOwnTeam())) &&
           !(agent.position == Tactic::Position::forward ||
             agent.position == Tactic::Position::forwardL ||
             agent.position == Tactic::Position::forwardR))
  {
    Vector2f leftBallPosition = theGameState.isCornerKick() ? leftOwnCorner : leftOwnGoalCorner;
    Vector2f rightBallPosition = theGameState.isCornerKick() ? rightOwnCorner : rightOwnGoalCorner;
    if (agent.position == Tactic::Position::defender)
    {
      float distanceToLeftCorner = (leftBallPosition - theRobotPose.translation).squaredNorm();
      return SkillRequest::Builder::observe(distanceToLeftCorner < (rightBallPosition - theRobotPose.translation).squaredNorm() ?
        leftBallPosition : rightBallPosition);
    }
    if (Geometry::isPointInsidePolygon(leftBallPosition, agent.baseArea) ||
      std::find(agent.baseArea.begin(), agent.baseArea.end(), leftBallPosition) != agent.baseArea.end())
    {
      return SkillRequest::Builder::observe(leftBallPosition);
    }
    else if (Geometry::isPointInsidePolygon(rightBallPosition, agent.baseArea) ||
      std::find(agent.baseArea.begin(), agent.baseArea.end(), rightBallPosition) != agent.baseArea.end())
    {
      return SkillRequest::Builder::observe(rightBallPosition);
    }
    bool oneDefender = false;
    const Agent* defenderAgent;
    for (auto agent : otherAgents) {
      if (agent->position == Tactic::Position::defender) {
        oneDefender = true;
        defenderAgent = agent;
      }
    }
    if (oneDefender) {
      float distanceToLeftCorner = (leftBallPosition - defenderAgent->currentPosition).squaredNorm();
      return SkillRequest::Builder::observe(distanceToLeftCorner < (rightBallPosition - theRobotPose.translation).squaredNorm() ?
        rightBallPosition : leftBallPosition);
    }
    else {
      return SkillRequest::Builder::walkTo(agent.basePose);
    }
  }
  else
  {
    return SkillRequest::Builder::walkTo(agent.basePose);
  }
}
