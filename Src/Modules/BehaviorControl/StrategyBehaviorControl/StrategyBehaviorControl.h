/**
 * @file StrategyBehaviorControl.h
 *
 * This file declares a module that determines the strategy of the team.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Behavior.h"
#include "Representations/BehaviorControl/AgentStates.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/OpposingKickoff.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Framework/Module.h"
#include <vector>

MODULE(StrategyBehaviorControl,
{,
  REQUIRES(BallDropInModel),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  REQUIRES(ExtendedGameState),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(IndirectKick),
  REQUIRES(GroundContactState),
  REQUIRES(MotionInfo),
  REQUIRES(OpposingKickoff),
  REQUIRES(ReceivedTeamMessages),
  REQUIRES(RobotPose),
  REQUIRES(SentTeamMessage),
  REQUIRES(SetupPoses),
  REQUIRES(TeammatesBallModel),
  PROVIDES(SkillRequest),
  REQUIRES(SkillRequest),
  PROVIDES(StrategyStatus),
  PROVIDES(AgentStates),
  LOADS_PARAMETERS(
  {,
    (Strategy::Type) strategy, /**< The strategy to play. */
  }),
});

class StrategyBehaviorControl : public StrategyBehaviorControlBase
{
public:
  /** Constructor. */
  StrategyBehaviorControl();

  /**
   * Creates extended module info (union of this module's info and requirements of other behavior parts).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();

private:
  /**
   * Updates the skill request.
   * @param skillRequest The provided skill request.
   */
  void update(SkillRequest& skillRequest) override;

  /**
   * Updates the strategy status.
   * @param strategyStatus The provided strategy status.
   */
  void update(StrategyStatus& strategyStatus) override { strategyStatus = theStrategyStatus; }

  /**
   * Updates the agent states.
   * @param agentStates The provided agent states.
   */
  void update(AgentStates& agentStates) override { agentStates.agents = agents; }

  /**
   * Updates the list of agents to represent the most recent data.
   * @return A pointer to the agent that represents this player.
   */
  Agent* updateAgents();

  /**
   * Updates an agent using local representations.
   * @param agent The agent to update.
   */
  void updateAgentBySelf(Agent& agent);

  /**
   * Updates an agent using a team message.
   * @param agent The agent to update.
   * @param teamMessage The team message to incorporate.
   */
  void updateAgentByTeamMessage(Agent& agent, const ReceivedTeamMessage& teamMessage);

  /**
   * Updates the estimated position of the agent.
   * @param agent The agent to update.
   */
  void updateCurrentPosition(Agent& agent);

  Behavior theBehavior; /**< The instance of the behavior. */
  std::vector<Agent> agents; /**< The list of active agents. */
  StrategyStatus theStrategyStatus; /**< The strategy status which is provided later. */
};
