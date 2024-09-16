/**
 * @file Behavior.h
 *
 * This file declares the behavior framework.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/ActiveRole.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "Tools/BehaviorControl/Strategy/FreeKick.h"
#include "Tools/BehaviorControl/Strategy/KickOff.h"
#include "Tools/BehaviorControl/Strategy/PenaltyKick.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/BehaviorControl/Strategy/SetPlay.h"
#include "Tools/BehaviorControl/Strategy/Strategy.h"
#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Math/Eigen.h"
#include "Framework/Settings.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/OpposingKickoff.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include <array>
#include <vector>

class BallSearch;

class Behavior final
{
public:
  /** Constructor. */
  Behavior(const BallDropInModel& theBallDropInModel, const BallSpecification& theBallSpecification,
           const ExtendedGameState& theExtendedGameState, const FieldBall& theFieldBall,
           const FieldDimensions& theFieldDimensions, const FieldInterceptBall& theFieldInterceptBall,
           const FrameInfo& theFrameInfo, const GameState& theGameState,
           const IndirectKick& theIndirectKick, const OpposingKickoff& theOpposingKickoff,
           const TeammatesBallModel& theTeammatesBallModel);

  /** Destructor. */
  ~Behavior();

  /** Call \c preProcess on all C++ behaviors. */
  void preProcess();

  /** Call \c postProcess on all C++ behaviors. */
  void postProcess();

  /**
   * Executes the whole thing.
   * @param strategy The strategy that is being played.
   * @param self The agent that executes this function.
   * @param agents All agents in the team (including \c self).
   * @return The skill request for this agent.
   */
  SkillRequest update(Strategy::Type strategy, Agent& self, std::vector<Agent>& agents);

private:
  /**
   * Assign positions to all agents.
   * @param tactic The current tactic.
   * @param setPlay The current set play.
   * @param agents The agents that should get a position (every agent will have \c position and \c basePose set).
   * @param dontChangePositions Agents which already have a position should keep their position as long as possible.
   * @param proposedMirror The proposed mirror flag.
   * @param acceptedMirror The actually used mirror flag.
   */
  void assignPositions(Tactic::Type tactic, SetPlay::Type setPlay, std::vector<Agent>& agents, bool dontChangePositions, bool& proposedMirror, bool& acceptedMirror) const;

  /**
   * Assign roles to all agents.
   * @param agents The agents that should get a role (every agent will have \c role set).
   * @param self The agent that executeds this function.
   * @param otherAgents All other agents (excluding \c agent).
   */
  void assignRoles(std::vector<Agent>& agents, Agent& self, const std::vector<const Agent*>& otherAgents) const;

  /**
   * Executes the selected role for an agent.
   * @param agent The agent for which to do the calculations.
   * @param otherAgents All other agents (excluding \c agent).
   * @return The resulting skill request.
   */
  SkillRequest execute(const Agent& agent, const Agents& otherAgents);

  /**
   * Selects a set play.
   * @tparam SetPlayType The type of set play that must be chosen from.
   * @param agents The list of agents.
   * @param setPlays The set of set plays of the given type.
   * @param random Whether selection is done randomly or pseudo-randomly based on GameController features.
   * @return A set play of the given type.
   */
  template<typename SetPlayType>
  SetPlay::Type selectNewSetPlay(const std::vector<Agent>& agents, const std::vector<Strategy::WeightedSetPlay<SetPlayType>>& setPlays, bool random) const;

  /**
   * Checks whether a set play can be started.
   * @param setPlay The set play to evaluate.
   * @param agents The list of agents.
   * @param wasSelected Whether the set play was already selected before.
   * @return Whether the set play can be started.
   */
  [[nodiscard]] bool checkSetPlayStartConditions(SetPlay::Type setPlay, const std::vector<Agent>& agents, bool wasSelected = false) const;

  /** Updates \c ballXTimestamps. */
  void updateBallXTimestamps();

  /**
   * Calculates the cost vector of assigning agent i to position positionIndices[i] with a given cost matrix.
   * @param costMatrix A matrix where the rows are agents and the columns are positions.
   * @param positionIndices The position indices per agent which represent the assignment to test.
   * @return The cost vector for this assignment.
   */
  static std::vector<float> getAssignmentCost(const Eigen::MatrixXf& costMatrix, const std::vector<std::size_t>& positionIndices);

  /**
   * Determines the agent which should have an active role.
   * @param self The agent that executes this function.
   * @param agents All other agents (excluding \c self).
   * @param assign Whether \c nextRole should be set.
   * @return The selected agent (can be empty).
   */
  const Agent* determineActiveAgent(Agent& self, const std::vector<const Agent*>& otherAgents, bool assign = false) const;

  /**
   * Performs a majority vote among all agents on a specific topic.
   * @tparam Type The type of this topic which represents the available options.
   * @tparam proposal A pointer to a member of \c Agent which contains the vote of an agent on this topic.
   * @param agents The list of agents.
   * @param isVoteValid Checks whether the vote of an agent can be used.
   * @return The elected option.
   */
  template<typename Type, Type Agent::* proposal>
  static Type evaluateVotes(const std::vector<Agent>& agents, const std::function<bool(Type)>& isVoteValid);

  struct Vote
  {
    unsigned int count = 0; /**< How often an option is voted for. */
    int playerNumber = Settings::highestValidPlayerNumber + 1; /**< The lowest number of any player voting for this option.  */
  };

  struct BallXTimestamps
  {
    unsigned lastTimeWhenNotAhead = 0; /**< The last time when the ball was not ahead of a certain x threshold. */
    unsigned lastTimeWhenNotBehind = 0; /**< The last time when the ball was not behind a certain x threshold. */
  };

  const Pose2f ttrbWalkSpeed = Pose2f(100_deg, 260.f, 220.f); /**< Assumed walk speed for time to reach ball (TTRB) [rad/s, mm/s, mm/s]. */
  static constexpr float ttrbGetUpDuration = 5000.f; /**< Assumed get up duration for TTRB [ms]. */
  static constexpr float ttrbStabilityOffset = 1000.f; /**< A bonus that the current active agent gets in its TTRB to prevent unnecessary role switches [ms]. */
  static constexpr float ttrbGoalkeeperGoalKickPenalty = 20000.f; /**< A penalty for the goalkeeper's TTRB when a goal kick (for the own team) is going on [ms]. */
  static constexpr float ttrbWithoutBallPenalty = 2000.f; /**< An penalty for the TTRB when assigning the closestToTeammatesBall role [ms]. */
  static constexpr float kickPoseBallOffsetX = 170.f; /**< The offset of the (fake) kick pose behind the ball for calculating the TTRB [mm]. */
  static constexpr int ballDisappearedThreshold = 64; /**< If the ball has disappeared longer than this [ms], an agent can not become active and its TTRB is increased. */
  static constexpr int ballSeenThreshold = 1000; /**< If the ball is older than this [ms], an agent can not become active (it can stay active, though, if it already was). */
  static constexpr int ballLostThreshold = 5000; /**< If the ball is older than this [ms], an agent can not be active (even if it was). */
  static constexpr int yetAnotherBallThreshold = 8000; /**< No comment. */

  std::array<Strategy, Strategy::numOfTypes> strategies;
  std::array<Tactic, Tactic::numOfTypes> tactics;

  std::array<OwnKickOff, OwnKickOff::numOfTypes> ownKickOffs;
  std::array<OpponentKickOff, OpponentKickOff::numOfTypes> opponentKickOffs;
  std::array<OwnPenaltyKick, OwnPenaltyKick::numOfTypes> ownPenaltyKicks;
  std::array<OpponentPenaltyKick, OpponentPenaltyKick::numOfTypes> opponentPenaltyKicks;
  std::array<OwnFreeKick, OwnFreeKick::numOfTypes> ownFreeKicks;
  std::array<OpponentFreeKick, OpponentFreeKick::numOfTypes> opponentFreeKicks;
  std::vector<SetPlay*> setPlays;

  std::array<ActiveRole*, ActiveRole::numOfTypes> activeRoles{nullptr};
  std::array<PositionRole*, PositionRole::numOfTypes> positionRoles{nullptr};
  std::vector<Role*> roles;
  std::array<SetPlay::Action::Implementation*, SetPlay::Action::numOfTypes> setPlayActions{nullptr};
  BallSearch* ballSearch = nullptr;

  std::unordered_map<float, BallXTimestamps> ballXTimestamps;

  const BallDropInModel& theBallDropInModel;
  const BallSpecification& theBallSpecification;
  const ExtendedGameState& theExtendedGameState;
  const FieldBall& theFieldBall;
  const FieldDimensions& theFieldDimensions;
  const FieldInterceptBall& theFieldInterceptBall;
  const FrameInfo& theFrameInfo;
  const GameState& theGameState;
  const IndirectKick& theIndirectKick;
  const OpposingKickoff& theOpposingKickoff;
  const TeammatesBallModel& theTeammatesBallModel;
};
