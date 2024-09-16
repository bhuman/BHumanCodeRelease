/**
 * @file StrategyBehaviorControl.cpp
 *
 * This file implements a module that determines the strategy of the team.
 *
 * @author Arne Hasselbring
 */

#include "StrategyBehaviorControl.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/BehaviorControl/Strategy/BehaviorBase.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(StrategyBehaviorControl, StrategyBehaviorControl::getExtModuleInfo);

StrategyBehaviorControl::StrategyBehaviorControl() :
  theBehavior(theBallDropInModel, theBallSpecification, theExtendedGameState,
              theFieldBall, theFieldDimensions, theFieldInterceptBall,
              theFrameInfo, theGameState, theIndirectKick, theOpposingKickoff, theTeammatesBallModel)
{}

std::vector<ModuleBase::Info> StrategyBehaviorControl::getExtModuleInfo()
{
  auto result = StrategyBehaviorControl::getModuleInfo();
  BehaviorBase::addToModuleInfo(result);
  return result;
}

void StrategyBehaviorControl::update(SkillRequest& skillRequest)
{
  auto* self = updateAgents();

  theBehavior.preProcess();

  if(theGameState.playerState != GameState::active ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isFinished())
  {
    // Reset provided representations.
    theStrategyStatus.proposedTactic = Tactic::none;
    theStrategyStatus.acceptedTactic = Tactic::none;
    theStrategyStatus.proposedMirror = false;
    theStrategyStatus.acceptedMirror = false;
    theStrategyStatus.proposedSetPlay = SetPlay::none;
    theStrategyStatus.acceptedSetPlay = SetPlay::none;
    theStrategyStatus.setPlayStep = -1;
    theStrategyStatus.position = Tactic::Position::none;
    theStrategyStatus.role = Role::none;
    skillRequest = SkillRequest::Builder::empty();
  }
  else
  {
    ASSERT(self);

    skillRequest = theBehavior.update(strategy, *self, agents);

    theStrategyStatus.proposedTactic = self->proposedTactic;
    theStrategyStatus.acceptedTactic = self->acceptedTactic;
    theStrategyStatus.proposedMirror = self->proposedMirror;
    theStrategyStatus.acceptedMirror = self->acceptedMirror;
    theStrategyStatus.proposedSetPlay = self->proposedSetPlay;
    theStrategyStatus.acceptedSetPlay = self->acceptedSetPlay;
    theStrategyStatus.setPlayStep = self->setPlayStep;
    theStrategyStatus.position = self->position;
    theStrategyStatus.role = self->role;
  }

  theBehavior.postProcess();
}

Agent* StrategyBehaviorControl::updateAgents()
{
  // Add agents that are active now but weren't before.
  for(unsigned int i = 0; i < theGameState.ownTeam.playerStates.size(); ++i)
  {
    const int number = Settings::lowestValidPlayerNumber + i;
    if((number == theGameState.playerNumber ? theGameState.playerState : theGameState.ownTeam.playerStates[i]) != GameState::active)
      continue;
    if(std::any_of(agents.begin(), agents.end(), [&](const Agent& agent){return agent.number == number;}))
      continue;
    agents.emplace_back();
    Agent& agent = agents.back();
    agent.number = number;
    agent.lastKnownTimestamp = theFrameInfo.time; // This is to avoid that "self" will write things into lastKnown* that were already sent a long time ago.
    agent.lastKnownPose = Vector2f(theFieldDimensions.xPosReturnFromPenalty, number % 2 ? theFieldDimensions.yPosLeftReturnFromPenalty : theFieldDimensions.yPosRightReturnFromPenalty);
  }

  // Remove agents that are not active anymore.
  for(auto it = agents.begin(); it != agents.end();)
  {
    Agent& agent = *it;
    if((agent.number == theGameState.playerNumber ? theGameState.playerState : theGameState.ownTeam.playerStates[agent.number - Settings::lowestValidPlayerNumber]) != GameState::active)
      it = agents.erase(it);
    else
    {
      agent.isGoalkeeper = theGameState.ownTeam.isGoalkeeper(agent.number);
      ++it;
    }
  }

  // The list of agents is now final for this frame, so the self pointer can be set.
  Agent* self = nullptr;
  for(Agent& agent : agents)
    if(agent.number == theGameState.playerNumber)
    {
      self = &agent;
      break;
    }

  if(self)
    updateAgentBySelf(*self);

  if(theGameState.kickOffSetupFromTouchlines)
  {
    for(Agent& agent : agents)
    {
      agent.lastKnownPose = theSetupPoses.getPoseOfRobot(agent.number).position;
    }
  }
  else if(theGameState.isSet() && theExtendedGameState.wasReady())
  {
    for(Agent& agent : agents)
    {
      agent.lastKnownPose = agent.basePose.translation;
      // Pretend that all other agents are where they are supposed to be and see the ball where they are supposed to see it - until the first message in playing arrives.
      if(&agent != self)
      {
        agent.pose = agent.basePose;
        agent.ballPosition = agent.pose.inverse() * (theGameState.isPenaltyKick() ? Vector2f(theGameState.isForOwnTeam() ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark, 0.f) :  Vector2f::Zero());
        agent.ballVelocity = Vector2f::Zero();
        agent.timeWhenBallLastSeen = agent.timestamp;
        agent.timeWhenBallDisappeared = agent.timestamp;
        agent.disagreeOnBall = false;
        agent.isUpright = true;
        agent.timeWhenLastUpright = agent.timestamp;
        if(self)
        {
          agent.proposedTactic = self->proposedTactic;
          agent.proposedSetPlay = self->proposedSetPlay;
          agent.proposedMirror = self->proposedMirror;
        }
      }
    }
  }
  else
  {
    for(const ReceivedTeamMessage& teamMessage : theReceivedTeamMessages.messages)
    {
      auto it = std::find_if(agents.begin(), agents.end(), [&](const Agent& agent){return agent.number == teamMessage.number;});
      if(it != agents.end())
        updateAgentByTeamMessage(*it, teamMessage);
    }
  }

  for(Agent& agent : agents)
    updateCurrentPosition(agent);

  return self;
}

void StrategyBehaviorControl::updateAgentBySelf(Agent& agent)
{
  if(theSentTeamMessage.theFrameInfo.time > agent.lastKnownTimestamp)
  {
    agent.lastKnownTimestamp = theSentTeamMessage.theFrameInfo.time;
    agent.lastKnownPose = theSentTeamMessage.theRobotPose;
    agent.lastKnownTarget = theSentTeamMessage.theBehaviorStatus.walkingTo;
    agent.lastKnownSpeed = theSentTeamMessage.theBehaviorStatus.speed;
  }
  agent.timestamp = theFrameInfo.time;
  agent.pose = theRobotPose;
  agent.ballPosition = theBallModel.estimate.position;
  agent.ballVelocity = theBallModel.estimate.velocity;
  agent.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;
  agent.timeWhenBallDisappeared = theBallModel.timeWhenDisappeared;
  agent.disagreeOnBall = false;
  agent.isUpright = (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting) &&
                    (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionInfo.executedPhase != MotionPhase::fall);
  if(agent.isUpright)
    agent.timeWhenLastUpright = theFrameInfo.time;
  agent.proposedTactic = theStrategyStatus.proposedTactic;
  agent.acceptedTactic = theStrategyStatus.acceptedTactic;
  agent.proposedMirror = theStrategyStatus.proposedMirror;
  agent.acceptedMirror = theStrategyStatus.acceptedMirror;
  agent.proposedSetPlay = theStrategyStatus.proposedSetPlay;
  agent.acceptedSetPlay = theStrategyStatus.acceptedSetPlay;
  agent.setPlayStep = theStrategyStatus.setPlayStep;
  agent.position = theStrategyStatus.position;
  agent.role = theStrategyStatus.role;
}

void StrategyBehaviorControl::updateAgentByTeamMessage(Agent& agent, const ReceivedTeamMessage& teamMessage)
{
  agent.lastKnownTimestamp = teamMessage.theFrameInfo.time;
  agent.lastKnownPose = teamMessage.theRobotPose;
  agent.lastKnownTarget = teamMessage.theBehaviorStatus.walkingTo;
  agent.lastKnownSpeed = teamMessage.theBehaviorStatus.speed;
  agent.timestamp = teamMessage.theFrameInfo.time;
  agent.pose = teamMessage.theRobotPose;
  agent.ballPosition = teamMessage.theBallModel.estimate.position;
  agent.ballVelocity = teamMessage.theBallModel.estimate.velocity;
  agent.timeWhenBallLastSeen = teamMessage.theBallModel.timeWhenLastSeen;
  agent.timeWhenBallDisappeared = teamMessage.theBallModel.timeWhenDisappeared;
  // Calculate disagreeOnBall.
  {
    // In theory, all those calculations should be made at the time teamMessage.theFrameInfo.time and therefore, there should be a buffer of the last few own RobotPoses and BallModels to compare with.
    // The following is okay as long as the network delay is sufficiently short.
    // Disagreement can come from either delocalized robots (where both would walk to the same physical ball, but at least one of them would play in the wrong direction) or from wrong balls (in which case both must go to their respective ball).
    // Agents who disagree on the ball with me will not be considered in my decision to play the ball.
    // Therefore, it is worse if both players wrongly assume no disagreement (while in fact they would go to two different balls).

    // Also, it could be nice if this decision was made somewhere in the modeling stage because similar calculations could happen in the TeammatesBallModel.
    if(teamMessage.theFrameInfo.getTimeSince(teamMessage.theBallModel.timeWhenLastSeen) > 1000 ||
       theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 1000)
    {
      // If one of us hasn't seen the ball recently, this is not disagreement.
      // One could instead check abs(teamMessage.theBallModel.timeWhenLastSeen - theBallModel.timeWhenLastSeen).
      // This is upper-bounded by 1000 here, but if both balls are old, it doesn't really matter anyway since the ball can have moved long ago and neither robot would become striker.
      agent.disagreeOnBall = false;
    }
    else
    {
      // Compare "now", i.e. propagate the teammate's ball to the current time.
      const Vector2f itsBallOnField = teamMessage.theRobotPose * BallPhysics::propagateBallPosition(teamMessage.theBallModel.estimate.position, teamMessage.theBallModel.estimate.velocity, static_cast<float>(theFrameInfo.getTimeSince(teamMessage.theFrameInfo.time)) / 1000.f, theBallSpecification.friction);
      const Vector2f myBallOnField = theRobotPose * theBallModel.estimate.position;
      agent.disagreeOnBall = (itsBallOnField - myBallOnField).squaredNorm() > sqr(777.f + (agent.disagreeOnBall ? 0.f : 222.f));
    }
  }
  agent.isUpright = teamMessage.theRobotStatus.isUpright;
  agent.timeWhenLastUpright = teamMessage.theRobotStatus.timeWhenLastUpright;
  agent.proposedTactic = teamMessage.theStrategyStatus.proposedTactic;
  agent.acceptedTactic = teamMessage.theStrategyStatus.acceptedTactic;
  agent.proposedMirror = teamMessage.theStrategyStatus.proposedMirror;
  agent.acceptedMirror = teamMessage.theStrategyStatus.acceptedMirror;
  agent.proposedSetPlay = teamMessage.theStrategyStatus.proposedSetPlay;
  agent.acceptedSetPlay = teamMessage.theStrategyStatus.acceptedSetPlay;
  agent.setPlayStep = -1;
  agent.position = teamMessage.theStrategyStatus.position;
  agent.role = teamMessage.theStrategyStatus.role;
}

void StrategyBehaviorControl::updateCurrentPosition(Agent& agent)
{
  agent.currentPosition = Teammate::getEstimatedPosition(agent.lastKnownPose, agent.lastKnownTarget, agent.lastKnownSpeed, theFrameInfo.getTimeSince(agent.lastKnownTimestamp));
}
