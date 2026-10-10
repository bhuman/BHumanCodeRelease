/**
 * @file Behavior.cpp
 *
 * This file implements the behavior framework.
 *
 * @author Arne Hasselbring
 */

#include "Behavior.h"
#include "BallSearch.h"
#include "ActiveRoles/ClosestToTeamBall.h"
#include "ActiveRoles/FreeKickWall.h"
#include "ActiveRoles/KickOffForward.h"
#include "ActiveRoles/PlayBall.h"
#include "PositionRoles/Defender.h"
#include "PositionRoles/Forward.h"
#include "PositionRoles/Goalkeeper.h"
#include "PositionRoles/Midfielder.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Settings.h"
#include "Math/Random.h"
#include "Platform/SystemCall.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Streaming/InExpr.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/Strategy/LBAP.h"
#include "Tools/Modeling/BallPhysics.h"
#include <algorithm>
#include <functional>
#include <random>

Behavior::Behavior(const BallSearchParticles& theBallSearchParticles, const BallSpecification& theBallSpecification,
                   const BehaviorParameters& theBehaviorParameters, const ExtendedGameState& theExtendedGameState,
                   const FieldBall& theFieldBall, const FieldDimensions& theFieldDimensions,
                   const FieldInterceptBall& theFieldInterceptBall, const FrameInfo& theFrameInfo,
                   const GameState& theGameState, const IndirectKick& theIndirectKick,
                   const TeamBallModel& theTeamBallModel, const WalkingEngineOutput& theWalkingEngineOutput) :
  theBallSearchParticles(theBallSearchParticles),
  theBallSpecification(theBallSpecification),
  theBehaviorParameters(theBehaviorParameters),
  theExtendedGameState(theExtendedGameState),
  theFieldBall(theFieldBall),
  theFieldDimensions(theFieldDimensions),
  theFieldInterceptBall(theFieldInterceptBall),
  theFrameInfo(theFrameInfo),
  theGameState(theGameState),
  theIndirectKick(theIndirectKick),
  theTeamBallModel(theTeamBallModel),
  theWalkingEngineOutput(theWalkingEngineOutput)
{
  activeRoles[ActiveRole::playBall] = new PlayBall;
  activeRoles[ActiveRole::kickOffForward] = new KickOffForward;
  activeRoles[ActiveRole::freeKickWall] = new FreeKickWall;
  activeRoles[ActiveRole::closestToTeamBall] = new ClosestToTeamBall;

  positionRoles[PositionRole::goalkeeper] = new Goalkeeper;
  positionRoles[PositionRole::defender] = new Defender;
  positionRoles[PositionRole::midfielder] = new Midfielder;
  positionRoles[PositionRole::forward] = new Forward;

  roles.resize(Role::Type_Info::numOfElements, nullptr);
  FOREACH_ENUM(ActiveRole::Type, type)
    roles[ActiveRole::toRole(type)] = activeRoles[type];
  FOREACH_ENUM(PositionRole::Type, type)
    roles[PositionRole::toRole(type)] = positionRoles[type];

  ballSearch = new BallSearch;

  // Create the symbol map for loading behaviors from configuration files.
  std::unordered_map<std::string, float> symbols;
#define SET_SYMBOL(representation, member) symbols[#member] = (representation).member
  SET_SYMBOL(theFieldDimensions, xPosOwnFieldBorder);
  SET_SYMBOL(theFieldDimensions, xPosOwnGoal);
  SET_SYMBOL(theFieldDimensions, xPosOwnGoalPost);
  SET_SYMBOL(theFieldDimensions, xPosOwnGoalLine);
  SET_SYMBOL(theFieldDimensions, xPosOwnGoalArea);
  SET_SYMBOL(theFieldDimensions, xPosOwnPenaltyMark);
  SET_SYMBOL(theFieldDimensions, xPosOwnPenaltyArea);
  SET_SYMBOL(theFieldDimensions, xPosHalfwayLine);
  SET_SYMBOL(theFieldDimensions, xPosPenaltyStrikerStartPosition);
  SET_SYMBOL(theFieldDimensions, xPosOpponentPenaltyArea);
  SET_SYMBOL(theFieldDimensions, xPosOpponentPenaltyMark);
  SET_SYMBOL(theFieldDimensions, xPosOpponentGoalArea);
  SET_SYMBOL(theFieldDimensions, xPosOpponentGoalLine);
  SET_SYMBOL(theFieldDimensions, xPosOpponentGoalPost);
  SET_SYMBOL(theFieldDimensions, xPosOpponentGoal);
  SET_SYMBOL(theFieldDimensions, xPosOpponentFieldBorder);
  SET_SYMBOL(theFieldDimensions, xPosReturnFromPenalty);

  SET_SYMBOL(theFieldDimensions, yPosLeftFieldBorder);
  SET_SYMBOL(theFieldDimensions, yPosLeftReturnFromPenalty);
  SET_SYMBOL(theFieldDimensions, yPosLeftTouchline);
  SET_SYMBOL(theFieldDimensions, yPosLeftPenaltyArea);
  SET_SYMBOL(theFieldDimensions, yPosLeftGoal);
  SET_SYMBOL(theFieldDimensions, yPosLeftGoalArea);
  SET_SYMBOL(theFieldDimensions, yPosRightGoalArea);
  SET_SYMBOL(theFieldDimensions, yPosRightGoal);
  SET_SYMBOL(theFieldDimensions, yPosRightPenaltyArea);
  SET_SYMBOL(theFieldDimensions, yPosRightTouchline);
  SET_SYMBOL(theFieldDimensions, yPosRightReturnFromPenalty);
  SET_SYMBOL(theFieldDimensions, yPosRightFieldBorder);

  SET_SYMBOL(theFieldDimensions, centerCircleRadius);
  SET_SYMBOL(theFieldDimensions, fieldLinesWidth);
  SET_SYMBOL(theFieldDimensions, goalPostRadius);
  SET_SYMBOL(theFieldDimensions, markRadius);
#undef SET_SYMBOL

  // Load strategies.
  FOREACH_ENUM(Strategy::Type, strategy)
  {
    if(strategy == Strategy::none)
      continue;
    InExprMapFile stream(std::string("Behavior/Strategies/") + TypeRegistry::getEnumName(strategy) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> strategies[strategy];

    for(const auto& state : strategies[strategy].tactics)
      for(const auto& transition : state.transitions)
        for(const auto& condition : transition.conditions)
          ballXTimestamps[condition.ballXThreshold] = BallXTimestamps();
  }

  // Load tactics.
  FOREACH_ENUM(Tactic::Type, tactic)
  {
    if(tactic == Tactic::none)
      continue;
    InExprMapFile stream(std::string("Behavior/Tactics/") + TypeRegistry::getEnumName(tactic) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> tactics[tactic];
#ifndef NDEBUG
    tactics[tactic].verify(tactic);
#endif
  }

  // Load set plays (and put them into the total set play array).
  setPlays.resize(SetPlay::Type_Info::numOfElements, nullptr);
  FOREACH_ENUM(OwnKickOff::Type, ownKickOff)
  {
    InExprMapFile stream(std::string("Behavior/KickOffs/") + TypeRegistry::getEnumName(ownKickOff) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownKickOffs[ownKickOff];
#ifndef NDEBUG
    ownKickOffs[ownKickOff].verify(OwnKickOff::toSetPlay(ownKickOff), tactics);
#endif
    setPlays[OwnKickOff::toSetPlay(ownKickOff)] = &ownKickOffs[ownKickOff];
  }
  FOREACH_ENUM(OpponentKickOff::Type, opponentKickOff)
  {
    InExprMapFile stream(std::string("Behavior/KickOffs/") + TypeRegistry::getEnumName(opponentKickOff) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentKickOffs[opponentKickOff];
#ifndef NDEBUG
    opponentKickOffs[opponentKickOff].verify(OpponentKickOff::toSetPlay(opponentKickOff), tactics);
#endif
    setPlays[OpponentKickOff::toSetPlay(opponentKickOff)] = &opponentKickOffs[opponentKickOff];
  }
  FOREACH_ENUM(OwnPenaltyKick::Type, ownPenaltyKick)
  {
    InExprMapFile stream(std::string("Behavior/PenaltyKicks/") + TypeRegistry::getEnumName(ownPenaltyKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownPenaltyKicks[ownPenaltyKick];
#ifndef NDEBUG
    ownPenaltyKicks[ownPenaltyKick].verify(OwnPenaltyKick::toSetPlay(ownPenaltyKick), tactics);
#endif
    setPlays[OwnPenaltyKick::toSetPlay(ownPenaltyKick)] = &ownPenaltyKicks[ownPenaltyKick];
  }
  FOREACH_ENUM(OpponentPenaltyKick::Type, opponentPenaltyKick)
  {
    InExprMapFile stream(std::string("Behavior/PenaltyKicks/") + TypeRegistry::getEnumName(opponentPenaltyKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentPenaltyKicks[opponentPenaltyKick];
#ifndef NDEBUG
    opponentPenaltyKicks[opponentPenaltyKick].verify(OpponentPenaltyKick::toSetPlay(opponentPenaltyKick), tactics);
#endif
    setPlays[OpponentPenaltyKick::toSetPlay(opponentPenaltyKick)] = &opponentPenaltyKicks[opponentPenaltyKick];
  }
  FOREACH_ENUM(OwnFreeKick::Type, ownFreeKick)
  {
    InExprMapFile stream(std::string("Behavior/FreeKicks/") + TypeRegistry::getEnumName(ownFreeKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownFreeKicks[ownFreeKick];
#ifndef NDEBUG
    ownFreeKicks[ownFreeKick].verify(OwnFreeKick::toSetPlay(ownFreeKick), tactics);
#endif
    setPlays[OwnFreeKick::toSetPlay(ownFreeKick)] = &ownFreeKicks[ownFreeKick];
  }
  FOREACH_ENUM(OpponentFreeKick::Type, opponentFreeKick)
  {
    InExprMapFile stream(std::string("Behavior/FreeKicks/") + TypeRegistry::getEnumName(opponentFreeKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentFreeKicks[opponentFreeKick];
#ifndef NDEBUG
    opponentFreeKicks[opponentFreeKick].verify(OpponentFreeKick::toSetPlay(opponentFreeKick), tactics);
#endif
    setPlays[OpponentFreeKick::toSetPlay(opponentFreeKick)] = &opponentFreeKicks[opponentFreeKick];
  }
  for(SetPlay* setPlay : setPlays)
  {
    if(!setPlay)
      continue;
    setPlay->compileVoronoiRegions(tactics);
  }
}

Behavior::~Behavior()
{
  for(const Role* role : roles)
    delete role;
  delete ballSearch;
}

void Behavior::preProcess()
{
  DECLARE_DEBUG_DRAWING("behavior:Tactic:voronoiDiagram", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:activeRole", "drawingOnField");
  updateBallXTimestamps();

  for(Role* role : roles)
    if(role)
      role->preProcess();
  if(ballSearch)
    ballSearch->preProcess();
}

void Behavior::postProcess()
{
  for(Role* role : roles)
    if(role)
      role->postProcess();
  if(ballSearch)
    ballSearch->postProcess();
}

SkillRequest Behavior::update(Strategy::Type strategy, Agent& self, std::vector<Agent>& agents)
{
  Agents otherAgents;
  otherAgents.reserve(agents.size() - 1);
  for(const Agent& otherAgent : agents)
    if(otherAgent.number != self.number)
      otherAgents.push_back(&otherAgent);

  // Handle set plays.
  const bool isKickingTeam = theGameState.isForOwnTeam();
  if(theGameState.isReady())
  {
    // Always force selection of a new set play when entering the ready state.
    // Otherwise, when scoring/conceding a goal, the same kick-off will be used again without deliberate selection.
    if(!theExtendedGameState.wasReady())
      self.proposedSetPlay = SetPlay::none;

    // During the ready state, all agents can freely propose a set play.
    const SetPlay::GameState setPlayType = theGameState.isPenaltyKick() ?
                                           (isKickingTeam ? SetPlay::ownPenaltyKick : SetPlay::opponentPenaltyKick) :
                                           (isKickingTeam ? SetPlay::ownKickOff : SetPlay::opponentKickOff);
    if(!SetPlay::isCompatible(setPlayType, self.proposedSetPlay) || !checkSetPlayStartConditions(self.proposedSetPlay, agents, true))
    {
      switch(setPlayType)
      {
        case SetPlay::ownKickOff:
          self.proposedSetPlay = selectNewSetPlay<OwnKickOff>(agents, strategies[strategy].ownKickOffs, false);
          break;
        case SetPlay::opponentKickOff:
          self.proposedSetPlay = selectNewSetPlay<OpponentKickOff>(agents, strategies[strategy].opponentKickOffs, false);
          break;
        case SetPlay::ownPenaltyKick:
          self.proposedSetPlay = selectNewSetPlay<OwnPenaltyKick>(agents, strategies[strategy].ownPenaltyKicks, false);
          break;
        case SetPlay::opponentPenaltyKick:
          self.proposedSetPlay = selectNewSetPlay<OpponentPenaltyKick>(agents, strategies[strategy].opponentPenaltyKicks, false);
          break;
        default:
          self.proposedSetPlay = SetPlay::none;
          FAIL("Unknown set play type.");
      }
    }

    // Since there is no communication in ready but set play selection of kick-offs / penalty kicks is deterministic, the proposed set play is also accepted.
    // Note that the accepted set play can still be none (e.g. if there is no available set play for the current game state in the strategy).
    self.acceptedSetPlay = self.proposedSetPlay;
    self.setPlayStep = -1;
  }
  else if(theGameState.isSet())
  {
    // During the set state, the selected set play cannot change anymore (even if its start conditions do not hold anymore).
    // However, agents which aren't proposing a valid set play (e.g. because they are being unpenalized now) continue to do so because
    // they will not usefully participate in the set play anyway (because they are far from the position where they should be).
    const SetPlay::GameState setPlayType = theGameState.isPenaltyKick() ?
                                           (isKickingTeam ? SetPlay::ownPenaltyKick : SetPlay::opponentPenaltyKick) :
                                           (isKickingTeam ? SetPlay::ownKickOff : SetPlay::opponentKickOff);
    if(!SetPlay::isCompatible(setPlayType, self.proposedSetPlay))
      self.proposedSetPlay = SetPlay::none;
    self.acceptedSetPlay = self.proposedSetPlay;
    self.setPlayStep = self.proposedSetPlay == SetPlay::none ? -1 : 0;
  }
  else
  {
    ASSERT(theGameState.isPlaying());

    bool proceedSetPlay = true;

    // Handle the preparation phase of a free kick.
    if(theGameState.isFreeKick())
    {
      // Reset the proposal when the set play type changes to force selection of a new free kick.
      // When the kicking team changes, the old proposal will not be compatible anymore anyway, so it doesn't need to be reset.
      if(theGameState.state != theExtendedGameState.stateLastFrame)
        self.proposedSetPlay = SetPlay::none;

      // If the current proposal is incompatible or the proposal has not been committed yet and its start conditions
      // do not hold anymore, a new free kick is selected.
      const SetPlay::GameState setPlayType = isKickingTeam ? SetPlay::ownFreeKick : SetPlay::opponentFreeKick;
      if(!SetPlay::isCompatible(setPlayType, self.proposedSetPlay) ||
         (!checkSetPlayStartConditions(self.proposedSetPlay, agents, !theGameState.stopped)))
      {
        self.proposedSetPlay = evaluateVotes<SetPlay::Type, &Agent::proposedSetPlay>(agents, [this, &agents, &setPlayType](auto setPlay)
        {
          return SetPlay::isCompatible(setPlayType, setPlay) && checkSetPlayStartConditions(setPlay, agents, !theGameState.stopped);
        });
        if(self.proposedSetPlay == SetPlay::none)
        {
          switch(setPlayType)
          {
            case SetPlay::ownFreeKick:
              self.proposedSetPlay = selectNewSetPlay<OwnFreeKick>(agents, strategies[strategy].ownFreeKicks, true);
              break;
            case SetPlay::opponentFreeKick:
              self.proposedSetPlay = selectNewSetPlay<OpponentFreeKick>(agents, strategies[strategy].opponentFreeKicks, true);
              break;
            default:
              self.proposedSetPlay = SetPlay::none;
              FAIL("Unknown set play type.");
          }
        }
        self.setPlayStep = -1;
      }

      // Under certain conditions, commit the free kick to allow it to start.
      // TODO: This does not work under very special circumstances.
      if(std::all_of(agents.begin(), agents.end(), [&](const Agent& agent) {return SetPlay::isCompatible(setPlayType, agent.proposedSetPlay);}))
      {
        self.proposedSetPlay = evaluateVotes<SetPlay::Type, &Agent::proposedSetPlay>(agents, [&setPlayType](auto setPlay) {return SetPlay::isCompatible(setPlayType, setPlay);});
        self.setPlayStep = 0;
      }
      else
        proceedSetPlay = false;

      self.acceptedSetPlay = evaluateVotes<SetPlay::Type, &Agent::proposedSetPlay>(agents, [&setPlayType](auto setPlay) {return SetPlay::isCompatible(setPlayType, setPlay);});
    }

    if(proceedSetPlay)
    {
      auto isSetPlayDone = [&]
      {
        // TODO: if(isOwnSetPlay && ballPossession == opponent || isOpponentSetPlay && ballPossession != opponent) return true;
        // TODO: if(set play game state is over (by timeout) and the ball hasn't been touched yet and won't be touched within the next 3 seconds) return true;

        // Kick-offs must be canceled if the whistle has not been heard (as in that case, the ball is probably already somewhere else).
        if(SetPlay::isKickOff(self.proposedSetPlay) && theExtendedGameState.wasSet() && !theGameState.isKickOff())
          return true;
        // If a set play hasn't been committed until now, it can't be played.
        if(self.setPlayStep < 0)
          return true;
        // TODO: This second call to assignPositions could break things.
        assignPositions(setPlays[self.proposedSetPlay]->tactic, self.proposedSetPlay, agents, true, self.proposedMirror, self.acceptedMirror);
        const Agent* activeAgent = determineActiveAgent(self, self.proposedSetPlay, otherAgents);
        // The set play cannot be continued if there is no active agent.
        if(!activeAgent)
          return true;
        // The set play cannot be continued if the active agent is not part of the set play (anymore).
        if(activeAgent->proposedSetPlay != self.proposedSetPlay &&
           // HACK: If the some agent does its last action, it can be that it send its message faster than this agent realized that another agent becomes active.
           // Therefore, we leave up to 1s to switch to the new active agent.
           !(activeAgent->proposedSetPlay == SetPlay::none && activeAgent != &self && theFrameInfo.getTimeSince(activeAgent->timestamp) < 1000))
          return true;
        if(theGameState.timeWhenStateEnds < theFrameInfo.time)
          return true;

        // Let it continue.
        return false;
      };

      // Let set plays end.
      if(self.proposedSetPlay != SetPlay::none && isSetPlayDone())
        self.proposedSetPlay = SetPlay::none;

      self.acceptedSetPlay = evaluateVotes<SetPlay::Type, &Agent::proposedSetPlay>(agents, [](auto setPlay) {return setPlay != SetPlay::none;});
    }
  }

  // Select a tactic.
  if(self.acceptedSetPlay != SetPlay::none)
    self.proposedTactic = self.acceptedTactic = setPlays[self.acceptedSetPlay]->tactic;
  else
  {
    // Reset the set play step.
    self.setPlayStep = -1;

    // Check if there was previously no tactic.
    if(self.proposedTactic == Tactic::none)
    {
      // Initialize the proposed tactic with the current majority (for stability).
      self.proposedTactic = evaluateVotes<Tactic::Type, &Agent::proposedTactic>(agents, [](auto tactic) {return tactic != Tactic::none;});
      if(self.proposedTactic == Tactic::none)
      {
        // If there is still no tactic (e.g. because there are no other agents), select the first tactic from the tactic state machine.
        ASSERT(!strategies[strategy].tactics.empty());
        self.proposedTactic = strategies[strategy].tactics[0].tactic;
      }
    }

    // Update the tactic with the tactic state machine.
    auto checkTransitionConditions = [&](const Strategy::TacticState::Transition::Condition& condition)
    {
      const std::size_t numOfFieldPlayers = std::count_if(agents.begin(), agents.end(), [](auto& agent) {return !agent.isGoalkeeper; });
      const int scoreDifference = theGameState.ownTeam.score - theGameState.opponentTeam.score;
      const auto ballXTimestamp = ballXTimestamps.find(condition.ballXThreshold);
      return condition.numOfFieldPlayersLE >= numOfFieldPlayers &&
             condition.numOfFieldPlayersGE <= numOfFieldPlayers &&
             condition.ownScoreLE >= theGameState.ownTeam.score &&
             condition.ownScoreGE <= theGameState.ownTeam.score &&
             condition.opponentScoreLE >= theGameState.opponentTeam.score &&
             condition.opponentScoreGE <= theGameState.opponentTeam.score &&
             condition.scoreDifferenceLE >= scoreDifference &&
             condition.scoreDifferenceGE <= scoreDifference &&
             condition.timeSinceBallAheadOfThresholdGE <= theFrameInfo.getTimeSince(ballXTimestamp->second.lastTimeWhenNotAhead) &&
             condition.timeSinceBallBehindThresholdGE <= theFrameInfo.getTimeSince(ballXTimestamp->second.lastTimeWhenNotBehind);
    };
    for(const Strategy::TacticState& state : strategies[strategy].tactics)
    {
      if(state.tactic == self.proposedTactic)
      {
        for(const Strategy::TacticState::Transition& transition : state.transitions)
          if(std::any_of(transition.conditions.begin(), transition.conditions.end(), std::bind(checkTransitionConditions, std::placeholders::_1)))
          {
            self.proposedTactic = transition.to;
            break;
          }
        goto foundTactic;
      }
    }
    ASSERT(!strategies[strategy].tactics.empty());
    self.proposedTactic = strategies[strategy].tactics[0].tactic;

  foundTactic:
    // Use a majority vote to determine the actually used tactic.
    self.acceptedTactic = evaluateVotes<Tactic::Type, &Agent::proposedTactic>(agents, [](auto tactic) {return tactic != Tactic::none;});
  }
  ASSERT(self.acceptedTactic != Tactic::none);

  // Assign positions using the tactic and potential set play modifications.
  assignPositions(self.acceptedTactic, self.acceptedSetPlay, agents, false, self.proposedMirror, self.acceptedMirror);
  ASSERT(self.position != Tactic::Position::none);

  // Assign roles.
  assignRoles(agents, self, otherAgents);

  // Determine the skill request.
  return execute(self, otherAgents);
}

void Behavior::assignPositions(Tactic::Type tactic, SetPlay::Type setPlay, std::vector<Agent>& agents, bool dontChangePositions, bool& proposedMirror, bool& acceptedMirror)
{
  const auto* positionSubsets = &tactics[tactic].positionSubsetsPerNumOfAgents;
  const auto* voronoiRegionSubsets = &tactics[tactic].voronoiRegionSubsetsPerNumOfAgents;
  ASSERT(positionSubsets->size() == voronoiRegionSubsets->size());
  auto positions = tactics[tactic].positions;
  std::array<bool, Tactic::Position::numOfTypes> relativeToTeamBallMap{ false };
  std::array<bool, Tactic::Position::numOfTypes> intendedStrikerMap{ false };

  // Integrate positions from set plays.
  if(setPlay != SetPlay::none)
  {
    const SetPlay& sP = *setPlays[setPlay];
    positionSubsets = &sP.positionSubsetsPerNumOfAgents;
    voronoiRegionSubsets = &sP.voronoiRegionSubsetsPerNumOfAgents;
    for(Tactic::Position& position : positions)
      for(const SetPlay::Position& positionOverride : sP.positions)
        if(positionOverride.position == position.type)
        {
          position.translation = positionOverride.translation;
          if(positionOverride.relativeToTeamBall)
          {
            position.translation += theTeamBallModel.position;
            relativeToTeamBallMap[position.type] = true;
          }
          if(positionOverride.intendedStriker)
          {
            position.translation = theTeamBallModel.position;
            intendedStrikerMap[position.type] = true;
          }
          break;
        }
  }

  // Initialize a set of pointers to the agents that haven't been assigned a position yet.
  std::vector<Agent*> remainingAgents;
  remainingAgents.reserve(agents.size());
  for(Agent& agent : agents)
    remainingAgents.push_back(&agent);

  // If the tactic contains a goalkeeper position, assign that to the goalkeeper agent (and nobody else).
  for(std::size_t i = 0; i < positions.size(); i++)
  {
    const Tactic::Position& position = positions[i];
    if(Tactic::Position::isGoalkeeper(position.type))
    {
      for(auto agentIt = remainingAgents.begin(); agentIt != remainingAgents.end(); ++agentIt)
      {
        Agent& agent = **agentIt;
        if(agent.isGoalkeeper)
        {
          agent.position = position.type;
          agent.baseTranslation = position.translation;
          if(agent.baseTranslation.x() == theFieldDimensions.xPosOwnPenaltyMark)
          {
            agent.baseArea
            = {Vector2f(theFieldDimensions.xPosOwnPenaltyMark + 350, theFieldDimensions.yPosLeftPenaltyArea),
               Vector2f(theFieldDimensions.xPosOwnPenaltyMark + 350, theFieldDimensions.yPosRightPenaltyArea),
               Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightPenaltyArea),
               Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftPenaltyArea)
              };

            // Draw Voronoi diagram:
            COMPLEX_DRAWING("behavior:Tactic:voronoiDiagram")
            {
              POLYGON("behavior:Tactic:voronoiDiagram", agent.baseArea.size(), agent.baseArea.data(), 40, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
              CIRCLE("behavior:Tactic:voronoiDiagram", agent.baseTranslation.x() + 100, agent.baseTranslation.y() - 50, 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
              DRAW_TEXT("behavior:Tactic:voronoiDiagram", agent.baseTranslation.x() + 100, agent.baseTranslation.y() - 50, 200, ColorRGBA::black, TypeRegistry::getEnumName(agent.position));
              DRAW_TEXT("behavior:Tactic:voronoiDiagram", theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline + 300, 200, ColorRGBA::black, "Current tactic: " << TypeRegistry::getEnumName(tactic));
            }
          }
          else
          {
            agent.baseArea = {Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea),
                              Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoalArea),
                              Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoalArea),
                              Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea)
                             };
            COMPLEX_DRAWING("behavior:Tactic:voronoiDiagram")
            {
              POLYGON("behavior:Tactic:voronoiDiagram", agent.baseArea.size(), agent.baseArea.data(), 40, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
              CIRCLE("behavior:Tactic:voronoiDiagram", agent.baseTranslation.x(), agent.baseTranslation.y(), 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
              DRAW_TEXT("behavior:Tactic:voronoiDiagram", agent.baseTranslation.x() + 100, agent.baseTranslation.y() - 50, 200, ColorRGBA::black, TypeRegistry::getEnumName(agent.position));
              DRAW_TEXT("behavior:Tactic:voronoiDiagram", theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline + 300, 200, ColorRGBA::black, "Current tactic: " << TypeRegistry::getEnumName(tactic));
            }
          }

          remainingAgents.erase(agentIt);
          break;
        }
      }
      break;
    }
  }

  if(!remainingAgents.empty())
  {
    if(remainingAgents.size() > positionSubsets->size() && theFrameInfo.getTimeSince(timeWhenLastPlayedSound) > 10000)
    {
      SystemCall::playSound("siren.wav");
      SystemCall::say("More robots than in strategy expected", true);
      timeWhenLastPlayedSound = theFrameInfo.time;

      for(size_t i = 0; i < remainingAgents.size(); i++)
      {
        remainingAgents[i]->position = Tactic::Position::none;
      }
      return;
    }
    const auto& suitableSubsets = (*positionSubsets)[remainingAgents.size() - 1];
    const auto& suitableVoronoiRegionSubsets = (*voronoiRegionSubsets)[remainingAgents.size() - 1];

    // Sort remaining agents because there is at least one point where the results depend on the order of the rows of the cost matrix (minCoeff in startPositionSpecialHandling).
    // Okay, it's not minCoeff anymore, but I am hesitant to remove this now. (2023-06-24)
    std::sort(remainingAgents.begin(), remainingAgents.end(), [](const Agent* a1, const Agent* a2) { return a1->number < a2->number; });

    // Initialize a set of pointers to the positions that haven't been assigned to an agent yet.
    std::array<const Tactic::Position*, Tactic::Position::numOfTypes> positionMap {nullptr};
    for(const Tactic::Position& position : positions)
      positionMap[position.type] = &position;

    std::array<const Tactic::Position*, Tactic::Position::numOfTypes> remainingPositions {nullptr};
    ASSERT(suitableSubsets.size() == suitableVoronoiRegionSubsets.size());
    for(size_t i = 0; i < suitableSubsets.size(); i++)
    {
      ASSERT(suitableSubsets[i].size() == suitableVoronoiRegionSubsets[i].size());
      for(Tactic::Position::Type position : suitableSubsets[i])
        remainingPositions[position] = positionMap[position];
    }

    // Construct a cost matrix where the rows correspond to agents and the columns correspond to positions.
    // The columns of the matrix alternate between the original position and the mirrored positions.
    // However, columns representing a mirrored position are only calculated for positions which can actually be mirrored.
    Eigen::MatrixXf costMatrix(remainingAgents.size(), 2 * remainingPositions.size());
    // Note here than Eigen matrices are stored column-major.
    for(Eigen::MatrixXf::Index positionIndex = 0; positionIndex < costMatrix.cols(); ++positionIndex)
    {
      const Tactic::Position* p = remainingPositions[positionIndex / 2];
      if(!p)
        continue;
      const bool mirror = positionIndex % 2;
      // If multiple agents claim to have had the same position, only the one with the lowest number counts.
      int theOneAndOnlyAgent = Settings::highestValidPlayerNumber + 1;
      if(!theGameState.isReady() && !theGameState.isSet())
        for(const Agent* agent : remainingAgents)
          if(agent->position == Tactic::Position::mirrorIf(p->type, mirror))
          {
            // remainingAgents is sorted ascending by player number, so we take the lowest numbered player with this role.
            theOneAndOnlyAgent = agent->number;
            break;
          }
      Vector2f target = Vector2f(p->translation.x(), mirror ? -p->translation.y() : p->translation.y());
      if(relativeToTeamBallMap[p->type])
      {
        target += theTeamBallModel.position;
      }
      if(intendedStrikerMap[p->type])
      {
        target = theTeamBallModel.position;
      }
      for(Eigen::MatrixXf::Index agentIndex = 0; agentIndex < costMatrix.rows(); ++agentIndex)
      {
        costMatrix(agentIndex, positionIndex) = (target - remainingAgents[agentIndex]->lastKnownPose.translation).norm();
        if(remainingAgents[agentIndex]->number == theOneAndOnlyAgent)
        {
          if(dontChangePositions)
            costMatrix(agentIndex, positionIndex) = std::numeric_limits<float>::lowest();
          else
            costMatrix(agentIndex, positionIndex) -= p->stabilityOffset;
        }
      }
    }

    const bool startPositionSpecialHandling = setPlay != SetPlay::none &&
                                              theGameState.isReady() &&
                                              setPlays[setPlay]->startPosition != Tactic::Position::none &&
                                              !Tactic::Position::isGoalkeeper(setPlays[setPlay]->startPosition);

    std::array<std::vector<std::size_t>, 2> bestAssignment;
    std::array<std::vector<float>, 2> bestAssignmentCost;
    // The index of the subset that the best assignment was taken from, because the regions are only valid within their own subset.
    std::array<std::size_t, 2> bestSubsetIndex{0, 0};
    // In some cases, we could already know that one of the mirror variants is not going to be used.
    // But you know what they say about premature optimization...
    for(bool mirrored : {false, true})
    {
      std::size_t startPositionIndex = 0; // The index (column) of the start position in the cost matrix.
      float startPositionCost; // The optimal cost for the start position.
      if(startPositionSpecialHandling)
      {
        const auto startPosition = setPlays[setPlay]->startPosition;
        ASSERT(remainingPositions[startPosition]);
        startPositionIndex = startPosition * 2 + mirrored;
        startPositionCost = costMatrix.col(startPositionIndex).minCoeff();
      }
      for(std::size_t subsetIndex = 0; subsetIndex < suitableSubsets.size(); ++subsetIndex)
      {
        const std::vector<Tactic::Position::Type>& subset = suitableSubsets[subsetIndex];
        std::vector<std::size_t> candidateColumns(subset.size());
        for(std::size_t k = 0; k < subset.size(); ++k)
          candidateColumns[k] = subset[k] * 2 + mirrored;

        const auto isAllowed = [&](std::size_t row, std::size_t col)
        {
          return !startPositionSpecialHandling || col != startPositionIndex || costMatrix(row, col) <= startPositionCost;
        };

        const std::vector<std::size_t> assignment = LBAP::solve(costMatrix, candidateColumns, isAllowed);
        if(assignment.empty())
          continue;

        const std::vector<float> assignmentCost = getAssignmentCost(costMatrix, assignment);
        if(bestAssignmentCost[mirrored].empty() ||
           std::lexicographical_compare(assignmentCost.begin(), assignmentCost.end(), bestAssignmentCost[mirrored].begin(), bestAssignmentCost[mirrored].end()))
        {
          bestAssignmentCost[mirrored] = assignmentCost;
          bestAssignment[mirrored] = assignment;
          bestSubsetIndex[mirrored] = subsetIndex;
        }
      }
      // Make sure that there was a valid assignment.
      ASSERT(!bestAssignmentCost[mirrored].empty());
    }
    ASSERT(!bestAssignment[false].empty() || !bestAssignment[true].empty());

    // TODO: move this to updateSetPlay
    // TODO: setPlay != SET_PLAY_NONE -> setPlayStep < 0?
    // TODO: don't check team ball model continuously, but only before the free kick starts
    if(SetPlay::isFreeKick(setPlay) && theGameState.isFreeKick() && static_cast<const FreeKick*>(setPlays[setPlay])->ballSide != FreeKick::irrelevant)
    {
      const FreeKick& freeKick = *static_cast<const FreeKick*>(setPlays[setPlay]);
      if((freeKick.ballSide == FreeKick::left && theFieldBall.recentBallPositionOnField().y() < 0.f) ||
         (freeKick.ballSide == FreeKick::right && theFieldBall.recentBallPositionOnField().y() > 0.f))
        proposedMirror = true;
      else
        proposedMirror = false;
    }
    else if(!dontChangePositions)
      proposedMirror = std::lexicographical_compare(bestAssignmentCost[true].begin(), bestAssignmentCost[true].end(), bestAssignmentCost[false].begin(), bestAssignmentCost[false].end());

    if(theGameState.isReady() || theGameState.isSet())
      acceptedMirror = proposedMirror;
    else
      acceptedMirror = evaluateVotes<bool, &Agent::proposedMirror>(agents, [](bool) {return true;});

    const auto& acceptedSubset = suitableSubsets[bestSubsetIndex[acceptedMirror]];
    const auto& acceptedVoronoiRegions = suitableVoronoiRegionSubsets[bestSubsetIndex[acceptedMirror]];

    for(std::size_t i = 0; i < bestAssignment[acceptedMirror].size(); ++i)
    {
      const size_t positionIndex = bestAssignment[acceptedMirror][i] / 2;
      const Tactic::Position& position = *remainingPositions[positionIndex];
      remainingAgents[i]->position = Tactic::Position::mirrorIf(position.type, acceptedMirror);
      remainingAgents[i]->baseTranslation = acceptedMirror ? Vector2f(position.translation.x(), -position.translation.y())
                                            : position.translation;
      if(relativeToTeamBallMap[position.type])
      {
        remainingAgents[i]->baseTranslation += theTeamBallModel.position;
      }
      if(intendedStrikerMap[position.type])
      {
        remainingAgents[i]->baseTranslation = theTeamBallModel.position;
      }

      const std::size_t indexInSubset = std::find(acceptedSubset.begin(), acceptedSubset.end(), static_cast<Tactic::Position::Type>(positionIndex)) - acceptedSubset.begin();
      ASSERT(indexInSubset < acceptedSubset.size());
      const std::vector<Vector2f>& voronoiRegion = acceptedVoronoiRegions[indexInSubset];

      if(!acceptedMirror)
        remainingAgents[i]->baseArea = voronoiRegion;
      else
      {
        remainingAgents[i]->baseArea.resize(voronoiRegion.size());
        std::reverse_copy(voronoiRegion.begin(), voronoiRegion.end(), remainingAgents[i]->baseArea.begin());
        for(Vector2f& point : remainingAgents[i]->baseArea)
          point = Vector2f(point.x(), -point.y());
      }
    }

    COMPLEX_DRAWING("behavior:Tactic:voronoiDiagram")
    {
      for(std::size_t i = 0; i < bestAssignment[acceptedMirror].size(); ++i)
      {
        POLYGON("behavior:Tactic:voronoiDiagram", remainingAgents[i]->baseArea.size(), remainingAgents[i]->baseArea.data(), 40, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
        CIRCLE("behavior:Tactic:voronoiDiagram", remainingAgents[i]->baseTranslation.x(), remainingAgents[i]->baseTranslation.y(), 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
        DRAW_TEXT("behavior:Tactic:voronoiDiagram", remainingAgents[i]->baseTranslation.x() + 100, remainingAgents[i]->baseTranslation.y() - 50, 200, ColorRGBA::black, TypeRegistry::getEnumName(remainingAgents[i]->position));
        DRAW_TEXT("behavior:Tactic:voronoiDiagram", theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline + 300, 200, ColorRGBA::black, "Current tactic: " << TypeRegistry::getEnumName(tactic));
      }
    }
  }
  else
    proposedMirror = acceptedMirror = false;
}

void Behavior::assignRoles(std::vector<Agent>& agents, Agent& self, const std::vector<const Agent*>& otherAgents) const
{
  if(theGameState.isPlaying())
  {
    // By default, assign everyone to their position role.
    for(Agent& agent : agents)
    {
      agent.nextRole = PositionRole::toRole(PositionRole::fromPosition(agent.position));
    }

    determineActiveAgent(self, self.acceptedSetPlay, otherAgents, true);
  }
  else
  {
    const Tactic::Position::Type setPlayStartPosition = self.acceptedSetPlay != SetPlay::none &&
                                                        setPlays[self.acceptedSetPlay] ?
                                                        Tactic::Position::mirrorIf(setPlays[self.acceptedSetPlay]->startPosition, self.acceptedMirror) :
                                                        Tactic::Position::none;
    for(Agent& agent : agents)
      agent.nextRole = agent.position == setPlayStartPosition ? ActiveRole::toRole(ActiveRole::startSetPlay) : Role::none;
  }

  // Commit the roles.
  for(Agent& agent : agents)
  {
    if(agent.nextRole != agent.role)
    {
      agent.role = agent.nextRole;
      if(agent.number == self.number && roles[agent.role])
        roles[agent.role]->reset();
    }
  }
}

SkillRequest Behavior::execute(const Agent& agent, const Agents& otherAgents)
{
  const auto getTargetPoseRotatedToBall = [&]
  {
    Vector2f translation = agent.baseTranslation;
    Vector2f ballPosition;
    if(theGameState.isReady())
    {
      if(theGameState.isKickOff())
        ballPosition = Vector2f::Zero();
      else if(theGameState.isForOwnTeam())
        ballPosition = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
      else
        ballPosition = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
    }
    else
    {
      ASSERT(theGameState.isPlaying());
      ballPosition = theFieldBall.recentBallPositionOnField();
    }

    Angle rotation = (ballPosition - translation).angle();
    return Pose2f(rotation, translation);
  };
  if(theGameState.isReady())
    return SkillRequest::Builder::walkTo(getTargetPoseRotatedToBall());
  else if(theGameState.isSet())
    return SkillRequest::Builder::stand();

  hasTeam = !otherAgents.empty();

  const int ballSearchThreshold = !hasTeam ? theBehaviorParameters.ballSearchThresholdNoTeam : theBehaviorParameters.ballSearchThresholdTeam;
  if(!theTeamBallModel.isValid && theFrameInfo.getTimeSince(agent.timeWhenBallLastSeen) > ballSearchThreshold && theBallSearchParticles.anyParticlesInRegion(agent))
    return ballSearch->execute(agent);

  return roles[agent.role] ? roles[agent.role]->execute(agent, otherAgents) : SkillRequest::Builder::empty();
}

template<typename SetPlayType>
SetPlay::Type Behavior::selectNewSetPlay(const std::vector<Agent>& agents, const std::vector<Strategy::WeightedSetPlay<SetPlayType>>& setPlays, bool random) const
{
  const std::uint32_t seed = theGameState.ownTeam.score | (theGameState.opponentTeam.score << 4) | (theGameState.opponentTeam.number << 8) | (Global::getSettings().magicNumber << 16);
  std::mt19937 generator(seed);
  std::vector<const Strategy::WeightedSetPlay<SetPlayType>*> applicableSetPlays;
  float weightSum = 0.f;
  for(const Strategy::WeightedSetPlay<SetPlayType>& weightedSetPlay : setPlays)
  {
    if(checkSetPlayStartConditions(SetPlayType::toSetPlay(weightedSetPlay.type), agents))
    {
      applicableSetPlays.push_back(&weightedSetPlay);
      weightSum += weightedSetPlay.weight;
    }
  }
  if(applicableSetPlays.empty())
    return SetPlay::none;
  if(weightSum == 0.f)
  {
    std::size_t index = random ? Random::uniformInt(applicableSetPlays.size() - 1) : std::uniform_int_distribution<std::size_t>(0, applicableSetPlays.size() - 1)(generator);
    return SetPlayType::toSetPlay(applicableSetPlays[index]->type);
  }
  const float index = random ? Random::uniform(0.f, weightSum) : std::uniform_real_distribution<float>(0.f, weightSum)(generator);
  float currentSum = 0.f;
  for(const Strategy::WeightedSetPlay<SetPlayType>* weightedSetPlay : applicableSetPlays)
  {
    currentSum += weightedSetPlay->weight;
    if(index < currentSum)
      return SetPlayType::toSetPlay(weightedSetPlay->type);
  }
  return SetPlayType::toSetPlay(applicableSetPlays.back()->type);
}

bool Behavior::checkSetPlayStartConditions(SetPlay::Type setPlay, const std::vector<Agent>& agents, bool wasSelected) const
{
  if(SetPlay::isKickOff(setPlay))
  {
    // A kick-off is selectable if there are enough players available.
    const auto& kickOff = *static_cast<const KickOff*>(setPlays[setPlay]);
    std::size_t numOfRequiredNonGoalkeeperAgents = 0;
    for(const auto& priorityGroup : kickOff.priorityGroups)
      for(auto priority : priorityGroup.priorities)
        if(priority <= kickOff.lowestRequiredPriority)
          ++numOfRequiredNonGoalkeeperAgents;
    const std::size_t numOfNonGoalkeeperAgents = std::count_if(agents.begin(), agents.end(), [](auto& agent) {return !agent.isGoalkeeper; });
    return numOfNonGoalkeeperAgents >= numOfRequiredNonGoalkeeperAgents;
  }
  else if(SetPlay::isPenaltyKick(setPlay))
  {
    // No need to implement something here as long as there is only one penalty kick.
    return true;
  }
  else if(SetPlay::isFreeKick(setPlay))
  {
    const int ballSearchThreshold = !hasTeam ? theBehaviorParameters.ballSearchThresholdNoTeam : theBehaviorParameters.ballSearchThresholdTeam;
    const auto& freeKick = *static_cast<const FreeKick*>(setPlays[setPlay]);
    const FreeKick::Condition& condition = wasSelected ? freeKick.invariants : freeKick.preconditions;
    const Vector2f ballPosition = theFieldBall.recentBallPositionOnField(ballSearchThreshold);
    const Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoalLine, 0.f);
    const float ballToOpponentGoalDistance = (opponentGoal - ballPosition).norm();
    const Angle ballToOpponentGoalAbsAngle = std::abs((opponentGoal - ballPosition).angle());
    const float ballX = ballPosition.x();
    const float ballAbsY = std::abs(ballPosition.y());
    return condition.ballToOpponentGoalDistanceLE >= ballToOpponentGoalDistance &&
           condition.ballToOpponentGoalDistanceGE <= ballToOpponentGoalDistance &&
           condition.ballToOpponentGoalAbsAngleLE >= ballToOpponentGoalAbsAngle &&
           condition.ballToOpponentGoalAbsAngleGE <= ballToOpponentGoalAbsAngle &&
           condition.ballXLE >= ballX &&
           condition.ballXGE <= ballX &&
           condition.ballYAbsLE >= ballAbsY &&
           condition.ballYAbsGE <= ballAbsY;
  }
  else
    FAIL("Unknown set play type.");
  return true;
}

void Behavior::updateBallXTimestamps()
{
  const int ballSearchThreshold = !hasTeam ? theBehaviorParameters.ballSearchThresholdNoTeam : theBehaviorParameters.ballSearchThresholdTeam;
  if(theTeamBallModel.isValid || theFieldBall.ballWasSeen(ballSearchThreshold))
  {
    const float ballX = theFieldBall.recentBallPositionOnField(ballSearchThreshold).x();
    for(auto& ballXTimestamp : ballXTimestamps)
    {
      if(ballX < ballXTimestamp.first)
        ballXTimestamp.second.lastTimeWhenNotAhead = theFrameInfo.time;
      if(ballX > ballXTimestamp.first)
        ballXTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
    }
  }
  else
    for(auto& ballXTimestamp : ballXTimestamps)
      ballXTimestamp.second.lastTimeWhenNotAhead = ballXTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
}

std::vector<float> Behavior::getAssignmentCost(const Eigen::MatrixXf& costMatrix, const std::vector<std::size_t>& positionIndices)
{
  std::vector<float> result(positionIndices.size());
  ASSERT(positionIndices.size() == static_cast<std::size_t>(costMatrix.rows()));
  for(std::size_t i = 0; i < result.size(); ++i)
    result[i] = costMatrix(i, positionIndices[i]);
  std::sort(result.begin(), result.end(), std::greater<>());
  return result;
}

const Agent* Behavior::determineActiveAgent(Agent& self, SetPlay::Type& setPlay, const std::vector<const Agent*>& otherAgents, bool assign) const
{
  // In contrast to most other methods in this class which try to compute the same thing on every agent,
  // this method knowingly prefers the agent on which the software is running.

  const bool isOpponentFreeKick = theGameState.isFreeKick() && theGameState.isForOpponentTeam();

  std::array<bool, Tactic::Position::numOfTypes> intendedStrikerMap{ false };
  bool wasIntendedStrikerDefined = false;
  if(setPlay != SetPlay::none)
  {
    const SetPlay& sP = *setPlays[setPlay];
    for(const SetPlay::Position& position : sP.positions)
      if(position.intendedStriker)
      {
        intendedStrikerMap[position.position] = true;
        wasIntendedStrikerDefined = true;
      }
  }

  // This function checks if the ball is in an area where the goalkeeper would want to play it.
  // The area is enlarged if the goalkeeper was already wanting to play the ball.
  const auto ballInGoalkeeperArea = [this](const Vector2f& ballOnField, bool wasAtBall) -> bool
  {
    // This should probably replaced with something elliptic.
    const float offset = wasAtBall ? 0.f : -300.f;
    return (
      ballOnField.x() < theFieldDimensions.xPosOwnPenaltyArea + offset &&
      ballOnField.y() < (theFieldDimensions.yPosLeftPenaltyArea + theFieldDimensions.yPosLeftGoalArea) * 0.5f + 150.f + offset &&
      ballOnField.y() > (theFieldDimensions.yPosRightPenaltyArea + theFieldDimensions.yPosRightGoalArea) * 0.5f - 150.f - offset
    );
  };

  // This function calculates a pose which is assumed to be the target of agents who want an active role.
  // It is usually a pose behind the ball facing the center of the opponent's goal, unless we are in a
  // free kick for the opponent. In that case, the reference pose is between the ball and the own penalty mark
  // with sufficient distance to the ball to stay legal.
  // This function deliberately does not do any complex calculations about where the agent really wants to play
  // because that often depends on information that not all agents share.
  const auto getReferencePoseOnField = [&](const Vector2f& ballOnField) -> Pose2f
  {
    // Things get worse here when the ball gets close to the target (penalty mark or goal center).
    if(isOpponentFreeKick)
    {
      const Vector2f ownPenaltyMark(theFieldDimensions.xPosOwnPenaltyMark, 0.f);
      return Pose2f((ballOnField - ownPenaltyMark).angle(), ballOnField).translate(-1100.f, 0.f);
    }
    else
    {
      const Vector2f goalCenter(theFieldDimensions.xPosOpponentGoalLine + 200.f, 0.f);
      return Pose2f((goalCenter - ballOnField).angle(), ballOnField).translate(-kickPoseBallOffsetX, 0.f);
    }
  };

  // This function calculates the "time to reach the ball" for a given agent. It is the main rating function
  // to determine which agent should be active. Some calculations differ depending on whether the agent was
  // active in the previous decision.
  const auto calcTTRB = [&](const Agent& agent, const Vector2f& ballOnField, bool wasActive, bool useTSBD) -> float
  {
    // We don't use the configured walk speed here because all agents should play with the same values.
    // The basic time to reach the ball is the time to reach a reference pose.
    float ttrb = KickSelection::calcTTRP(agent.pose.inverse() * getReferencePoseOnField(ballOnField), theWalkingEngineOutput.maxSpeed);
    ttrb += agent.number == theIndirectKick.lastStriker ? 30000 : 0;

    const bool ballWasSeen = agent.timestamp <= agent.timeWhenBallLastSeen + theBehaviorParameters.ballSeenThreshold;
    const bool ballDisappeared = std::abs(static_cast<int>(agent.timeWhenBallDisappeared - agent.timestamp)) > ballDisappearedThreshold;
    if(useTSBD && (!ballWasSeen || ballDisappeared || agent.disagreeOnBall))
      ttrb += 3000.f + (agent.disagreeOnBall ? theFrameInfo.getTimeSince(agent.timeSinceDisagreeOnBall) : 0.f); // Teammates that do not see the ball are assumed to see it after 3 seconds.

    if(wasActive)
    {
      // The time to reach the ball is decreased for an agent that was already going for the ball.
      ttrb -= ttrbStabilityOffset;

      // TODO: It would be somehow justified to subtract the time that has already passed since the agent's information was taken.
      // However, it turned out that this underestimates times from other agents and it was better without this line.
      // ttrb -= static_cast<float>(theFrameInfo.getTimeSince(agent.timestamp));

      // In most circumstances, the active agent gets a penalty for having lost the ball. This is only added for agents
      // that were already active because others should be excluded entirely from the assignment.
      if(useTSBD && std::abs(static_cast<int>(agent.timeWhenBallDisappeared - agent.timestamp)) > ballDisappearedThreshold)
        ttrb += static_cast<float>(std::max(0, theFrameInfo.getTimeSince(agent.timeWhenBallDisappeared) - theBehaviorParameters.ballSeenThreshold) * 2);
    }

    // Account for the time needed to get up.
    if(!agent.isUpright)
      ttrb += std::max(ttrbGetUpDuration / 10.f, ttrbGetUpDuration - static_cast<float>(theFrameInfo.getTimeSince(agent.timeWhenLastUpright)));

    if(Tactic::Position::isGoalkeeper(agent.position) && !ballInGoalkeeperArea(agent.pose * agent.ballPosition, wasActive))
      ttrb *= 2;

    return ttrb;
  };

  // This function determines whether a given agent should be a candidate for being active. The calculation
  // depends on whether the agent was active in the previous decision, so that there are different conditions
  // to *become* active than to *stay* active. The caller may specify if it is necessary that the agent has seen
  // the ball.
  const auto canBeActive = [&](const Agent& agent, bool wasActive, bool ballNeeded) -> bool
  {
    // This checks for the ball age when the message was sent.
    const bool ballWasSeen = agent.timestamp <= agent.timeWhenBallLastSeen + theBehaviorParameters.ballSeenThreshold;
    // Agents who believe in a different ball don't compete.
    if(agent.disagreeOnBall && !Tactic::Position::isGoalkeeper(self.position))
      return false;

    if(wasIntendedStrikerDefined && !intendedStrikerMap[self.position] && !ballWasSeen)
      return false;
    // For the goalkeeper to go to the ball, the ball must be in a specific area.
    if((agent.position == Tactic::Position::goalkeeper && !wasActive) || (Tactic::Position::isGoalkeeper(agent.position) && theGameState.isFreeKick()))
    {
      if(!otherAgents.empty()
         && ((theGameState.isGoalKick() && agent.acceptedSetPlay == OwnFreeKick::toSetPlay(OwnFreeKick::ownKickInOwnHalf))
             || (theGameState.isFreeKick() && !theGameState.isGoalKick())))
        return false;
      if(theGameState.isForOpponentTeam())
        return false;
      if(!ballWasSeen && ((ballNeeded && !wasActive) || !theTeamBallModel.isValid))
        return false;
    }
    if(!wasActive)
    {
      if(!agent.isUpright)
        return false;
    }
    if(ballNeeded && agent.timestamp > agent.timeWhenBallLastSeen + theBehaviorParameters.ballLostThreshold) // Don't be striker if the ball hasn't been seen for some time.
      return false;
    return true;
  };

  // closestToTeamBall does not count as active in some cases.
  auto wasActive = [](const Agent& agent) -> bool
  {
    return Role::isActiveRole(agent.role) && agent.role != ActiveRole::toRole(ActiveRole::closestToTeamBall);
  };

  // Only the previously active agent with the lowest player number gets special treatment.
  const int theOneAndOnlyPreviouslyActiveAgent = [&]
  {
    int number = wasActive(self) ? self.number : (Settings::highestValidPlayerNumber + 1);
    for(const Agent* agent : otherAgents)
      if(wasActive(*agent) && agent->number < number)
        number = agent->number;
    return number;
  }();

  // Case 1: I can be active and see the ball. This is more or less
  //         agents
  //             .iter()
  //             .filter(|agent| canBeActive(agent, agent.number == theOneAndOnlyPreviouslyActiveAgent, true))
  //             .map(|agent| calcTTRB(agent, myBallOnField, agent.number == theOneAndOnlyPreviouslyActiveAgent, true))
  //             .min()
  if(canBeActive(self, self.number == theOneAndOnlyPreviouslyActiveAgent, true))
  {
    // All "TTRB" calculations are done for the own (local) ball model, because updates of the ball position from other agents
    // can take a long time. It also has disadvantages, though (different agents calculate different things -> none or several
    // agents can be active).
    const Vector2f& myBallOnField = self.position == Tactic::Position::goalkeeper ? theFieldBall.positionOnField : theFieldInterceptBall.interceptedEndPositionOnField;

    // Find the agent among agents which can be active that has the lowest "TTRB".
    float minTTRB = calcTTRB(self, myBallOnField, self.number == theOneAndOnlyPreviouslyActiveAgent, true);
    if(assign)
      DRAW_TEXT("behavior:activeRole", self.pose.translation.x(), self.pose.translation.y(), 100, ColorRGBA::red, std::to_string(static_cast<int>(minTTRB)));
    const Agent* minAgent = &self;
    for(const Agent* agent : otherAgents)
    {
      const bool itWasActive = agent->number == theOneAndOnlyPreviouslyActiveAgent;
      if(!canBeActive(*agent, itWasActive, true))
        continue;
      const float itsTTRB = calcTTRB(*agent, myBallOnField, itWasActive, true);
      if(assign)
        DRAW_TEXT("behavior:activeRole", agent->pose.translation.x(), agent->pose.translation.y(), 100, ColorRGBA::red, std::to_string(static_cast<int>(itsTTRB)));
      if(itsTTRB < minTTRB)
      {
        minTTRB = itsTTRB;
        minAgent = agent;
      }
    }
    ASSERT(minAgent);
    if(assign)
      const_cast<Agent*>(minAgent)->nextRole = ActiveRole::toRole(isOpponentFreeKick ? ActiveRole::freeKickWall : ActiveRole::playBall);
    return minAgent;
  }
  // Case 2: I can be active but do not see the ball, but teammates have seen a ball. This can mean that I should turn to the ball
  //         to have the chance to see the ball and become active then.
  else if(assign && canBeActive(self, self.role == ActiveRole::toRole(ActiveRole::closestToTeamBall), false) && theTeamBallModel.isValid)
  {
    const Vector2f teamBallOnField = BallPhysics::getEndPosition(theTeamBallModel.position, theTeamBallModel.velocity, theBallSpecification);
    float minTTRBWithoutBall = calcTTRB(self, teamBallOnField, self.role == ActiveRole::toRole(ActiveRole::closestToTeamBall), false);
    if(assign)
      DRAW_TEXT("behavior:activeRole", self.pose.translation.x(), self.pose.translation.y(), 100, ColorRGBA::blue, std::to_string(static_cast<int>(minTTRBWithoutBall)));
    const Agent* minAgentWithoutBall = &self;

    // We also need to keep track of the agent which is actually active and sees the ball, because we only want to become active
    // if we are significantly better than it.
    float minTTRBWithBall = std::numeric_limits<float>::max();
    const Agent* minAgentWithBall = nullptr;
    for(const Agent* agent : otherAgents)
    {
      const bool itWasActive = agent->number == theOneAndOnlyPreviouslyActiveAgent;
      if(!canBeActive(*agent, itWasActive, true))
      {
        if(!canBeActive(*agent, agent->role == ActiveRole::toRole(ActiveRole::closestToTeamBall), false))
          continue;
        const float itsTTRB = calcTTRB(*agent, teamBallOnField, agent->role == ActiveRole::toRole(ActiveRole::closestToTeamBall), false);
        if(assign)
          DRAW_TEXT("behavior:activeRole", agent->pose.translation.x(), agent->pose.translation.y(), 100, ColorRGBA::blue, std::to_string(static_cast<int>(itsTTRB)));
        if(itsTTRB < minTTRBWithoutBall)
        {
          minTTRBWithoutBall = itsTTRB;
          minAgentWithoutBall = agent;
        }
        continue;
      }
      const float itsTTRB = calcTTRB(*agent, agent->pose * BallPhysics::getEndPosition(agent->ballPosition, agent->ballVelocity, theBallSpecification), itWasActive, true);
      if(assign)
        DRAW_TEXT("behavior:activeRole", agent->pose.translation.x(), agent->pose.translation.y(), 100, ColorRGBA::red, std::to_string(static_cast<int>(itsTTRB)));
      if(itsTTRB < minTTRBWithBall)
      {
        minTTRBWithBall = itsTTRB;
        minAgentWithBall = agent;
      }
    }

    // ttrbWithoutBallPenalty somehow accounts for the fact that finding the ball takes some time, so it isn't sufficient to be *just a bit* closer to the ball
    // than a agent who can already see it, especially since there is uncertainty in the transformation of the ball into my local coordinate system.
    if(minTTRBWithoutBall + ttrbWithoutBallPenalty < minTTRBWithBall)
      const_cast<Agent*>(minAgentWithoutBall)->nextRole = ActiveRole::toRole(ActiveRole::closestToTeamBall);
    if(minAgentWithBall)
      const_cast<Agent*>(minAgentWithBall)->nextRole = ActiveRole::toRole(isOpponentFreeKick ? ActiveRole::freeKickWall : ActiveRole::playBall);
    return minAgentWithBall;
  }
  else
  {
    // Just maintain some belief which other agent should be play the ball.
    // We don't assign closestToTeamBall to other agents here because it does not matter.
    float minTTRB = std::numeric_limits<float>::max();
    const Agent* minAgent = nullptr;
    for(const Agent* agent : otherAgents)
    {
      const bool itWasActive = agent->number == theOneAndOnlyPreviouslyActiveAgent;
      if(!canBeActive(*agent, itWasActive, true))
        continue;
      const float itsTTRB = calcTTRB(*agent, agent->pose * BallPhysics::getEndPosition(agent->ballPosition, agent->ballVelocity, theBallSpecification), itWasActive, true);
      if(assign)
        DRAW_TEXT("behavior:activeRole", agent->pose.translation.x(), agent->pose.translation.y(), 100, ColorRGBA::red, std::to_string(static_cast<int>(itsTTRB)));
      if(itsTTRB < minTTRB)
      {
        minTTRB = itsTTRB;
        minAgent = agent;
      }
    }
    if(assign && minAgent)
      const_cast<Agent*>(minAgent)->nextRole = ActiveRole::toRole(isOpponentFreeKick ? ActiveRole::freeKickWall : ActiveRole::playBall);
    return minAgent;
  }
}

template<typename Type, Type Agent::* proposal>
Type Behavior::evaluateVotes(const std::vector<Agent>& agents, const std::function<bool(Type)>& isVoteValid)
{
  std::vector<Vote> votes(2);
  for(const Agent& agent : agents)
  {
    if(isVoteValid(agent.*proposal))
    {
      votes.resize(std::max(votes.size(), static_cast<std::size_t>(agent.*proposal) + 1));
      ++votes[agent.*proposal].count;
      votes[agent.*proposal].playerNumber = std::min(votes[agent.*proposal].playerNumber, agent.number);
    }
  }
  return static_cast<Type>(std::distance(votes.begin(), std::max_element(votes.begin(), votes.end(), [](const auto& largest, const auto& x)
  {
    return x.count > largest.count || (x.count == largest.count && x.playerNumber < largest.playerNumber);
  })));
}
