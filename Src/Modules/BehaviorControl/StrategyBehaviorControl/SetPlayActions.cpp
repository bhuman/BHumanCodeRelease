/**
 * @file SetPlayActions.cpp
 *
 * This file implements set play actions.
 *
 * @author Arne Hasselbring
 */

#include "SetPlayActions.h"
#include "Math/Random.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"

void ShotAction::reset()
{
  timeWhenActionStarted = theFrameInfo.time;
}

bool ShotAction::isDone(const SetPlay::Action&, const Agent&, const Agents&) const
{
  return theMotionInfo.lastKickTimestamp > timeWhenActionStarted;
}

SkillRequest ShotAction::execute(const SetPlay::Action&, const Agent&, const Agents&)
{
  return SkillRequest::Builder::shoot();
}

void PassAction::reset()
{
  lastSelectedPassTarget = -1;
}

bool PassAction::isDone(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents) const
{
  for(Tactic::Position::Type passTargetPosition : action.passTarget)
    if(otherAgents.byPosition(Tactic::Position::mirrorIf(passTargetPosition, agent.acceptedMirror)))
      return false;
  // Pass action is "done" if no pass target agent is present.
  // TODO: This does not necessarily need to be true if a lot of time is left in the set play.
  return true;
}

SkillRequest PassAction::execute(const SetPlay::Action& action, const Agent& agent, const Agents& otherAgents)
{
  std::vector<int> passTargets;
  for(Tactic::Position::Type passTargetPosition : action.passTarget)
    if(const auto* passTarget = otherAgents.byPosition(Tactic::Position::mirrorIf(passTargetPosition, agent.acceptedMirror)); passTarget)
    {
      if(passTarget->number == lastSelectedPassTarget)
        return SkillRequest::Builder::passTo(passTarget->number);
      passTargets.push_back(passTarget->number);
    }

  // This should not happen due to isDone, but I am not confident enough to put a FAIL/ASSERT here.
  // The empty skill request should still make us notice.
  if(passTargets.empty())
  {
    lastSelectedPassTarget = -1;
    return SkillRequest::Builder::empty();
  }

  lastSelectedPassTarget = passTargets[Random::uniformInt(static_cast<int>(passTargets.size()) - 1)];
  return SkillRequest::Builder::passTo(lastSelectedPassTarget);
}

bool WaitAction::isDone(const SetPlay::Action&, const Agent&, const Agents&) const
{
  const auto& theGameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);
  return theGameState.state == GameState::playing;
}

SkillRequest WaitAction::execute(const SetPlay::Action&, const Agent&, const Agents&)
{
  return SkillRequest::Builder::stand();
}

SkillRequest MarkAction::execute(const SetPlay::Action& action, const Agent& agent, const Agents&)
{
  const auto& theGlobalOpponentsModel = static_cast<const GlobalOpponentsModel&>(Blackboard::getInstance()["GlobalOpponentsModel"]);
  for(const auto& opponent : theGlobalOpponentsModel.opponents)
  {
    const Vector2f obstacleOnField = opponent.position;
    // TODO: hysteresis and tracking of the previously chosen one
    if(!action.markZone.isInside(agent.acceptedMirror ? Vector2f(obstacleOnField.x(), -obstacleOnField.y()) : obstacleOnField))
      continue;
    return SkillRequest::Builder::mark(obstacleOnField);
  }

  if((agent.basePose.translation - agent.currentPosition).norm() > 500.f)
    return SkillRequest::Builder::walkTo(agent.basePose);
  else
    return SkillRequest::Builder::stand();
}

SkillRequest PositionAction::execute(const SetPlay::Action& action, const Agent& agent, const Agents&)
{
  // TODO: walkTo(basePose) or roles[PositionRole::toRole(PositionRole::fromPosition(agent.position))]->execute(agent, otherAgents) ?
  return SkillRequest::Builder::walkTo(agent.acceptedMirror ? Pose2f(-action.pose.rotation, action.pose.translation.x(), -action.pose.translation.y()) : action.pose);
}
