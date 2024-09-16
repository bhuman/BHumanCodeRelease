/**
 * @file SetPlay.cpp
 *
 * This file implements a base class for set plays.
 *
 * @author Arne Hasselbring
 */

#include "SetPlay.h"
#include "FreeKick.h"
#include "KickOff.h"
#include "PenaltyKick.h"
#include "Platform/BHAssert.h"
#include "Streaming/Enum.h"

unsigned SetPlay::ownKickOffBegin = 0;
unsigned SetPlay::opponentKickOffBegin = 0;
unsigned SetPlay::ownPenaltyKickBegin = 0;
unsigned SetPlay::opponentPenaltyKickBegin = 0;
unsigned SetPlay::ownFreeKickBegin = 0;
unsigned SetPlay::opponentFreeKickBegin = 0;

SetPlay::Type SetPlay::Type_Info::numOfElements = SetPlay::none;

void SetPlay::Type_Info::reg()
{
  PUBLISH(reg);
  const char* _type = typeid(Type).name();
  TypeRegistry::addEnum(_type);
  TypeRegistry::addEnumConstant(_type, "none");
  unsigned counter = 1;

  ownKickOffBegin = counter;
  FOREACH_ENUM(OwnKickOff::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  opponentKickOffBegin = counter;
  FOREACH_ENUM(OpponentKickOff::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  ownPenaltyKickBegin = counter;
  FOREACH_ENUM(OwnPenaltyKick::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  opponentPenaltyKickBegin = counter;
  FOREACH_ENUM(OpponentPenaltyKick::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  ownFreeKickBegin = counter;
  FOREACH_ENUM(OwnFreeKick::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }
  opponentFreeKickBegin = counter;
  FOREACH_ENUM(OpponentFreeKick::Type, constant)
  {
    TypeRegistry::addEnumConstant(_type, TypeRegistry::getEnumName(constant));
    ++counter;
  }

  numOfElements = static_cast<Type>(counter);
}

bool SetPlay::isCompatible(GameState state, Type type)
{
  switch(state)
  {
    case noSetPlay:
      return true;
    case ownKickOff:
      return isOwnKickOff(type);
    case opponentKickOff:
      return isOpponentKickOff(type);
    case ownPenaltyKick:
      return isOwnPenaltyKick(type);
    case opponentPenaltyKick:
      return isOpponentPenaltyKick(type);
    case ownFreeKick:
      return isOwnFreeKick(type);
    case opponentFreeKick:
      return isOpponentFreeKick(type);
  }
  return false;
}

void SetPlay::onRead()
{
  positionSubsetsPerNumOfAgents = Tactic::compilePriorityGroups(priorityGroups);
}

void SetPlay::compileVoronoiRegions(const std::array<Tactic, Tactic::numOfTypes>& tactics)
{
  std::vector<Tactic::Position> tacticPositions = tactics[tactic].positions;
  for(Tactic::Position& position : tacticPositions)
    for(const SetPlay::Position& positionOverride : positions)
      if(positionOverride.position == position.type)
      {
        position.pose = positionOverride.pose;
        break;
      }
  voronoiRegionSubsetsPerNumOfAgents = Tactic::generateVoronoiRegionSubsets(tacticPositions, positionSubsetsPerNumOfAgents);
}

void SetPlay::verify([[maybe_unused]] Type setPlay, const std::array<Tactic, Tactic::numOfTypes>& tactics) const
{
  if(tactic == Tactic::none)
    FAIL("The set play " << TypeRegistry::getEnumName(setPlay) << " references the tactic none.");

  std::array<int, Tactic::Position::numOfTypes> existingPositions{0};
  for(const Tactic::Position& position : tactics[tactic].positions)
    existingPositions[position.type] = 1;

  if(existingPositions[Tactic::Position::none])
    FAIL("The set play " << TypeRegistry::getEnumName(setPlay) << " tries to define the position none.");

  for(const Tactic::PriorityGroup& priorityGroup : priorityGroups)
    for(const Tactic::Position::Type position : priorityGroup.positions)
      if(Tactic::Position::isGoalkeeper(position))
        FAIL("The goalkeeper position appears in a priority group within set play " << TypeRegistry::getEnumName(setPlay) << ".");
      else if(!existingPositions[position])
        FAIL("The position " << TypeRegistry::getEnumName(position) << " appears in a priority group in set play " << TypeRegistry::getEnumName(setPlay) << " but is not defined within tactic " << TypeRegistry::getEnumName(tactic) << ".");
      else
        ++existingPositions[position];

  FOREACH_ENUM(Tactic::Position::Type, position)
    if(!Tactic::Position::isGoalkeeper(position))
    {
      if(existingPositions[position] == 1)
        FAIL("The position " << TypeRegistry::getEnumName(position) << " does not appear in a priority group within set play " << TypeRegistry::getEnumName(setPlay) << ".");
      else if(existingPositions[position] > 2)
        FAIL("The position " << TypeRegistry::getEnumName(position) << " appears multiple times in a priority group within set play " << TypeRegistry::getEnumName(setPlay) << ".");
    }

  if(existingPositions[Tactic::Position::goalkeeper])
    existingPositions[Tactic::Position::goalkeeper] = 2;

  int numOfWaitActions = 0;
  for(const Position& position : positions)
  {
    if(!existingPositions[position.position])
      FAIL("The position " << TypeRegistry::getEnumName(position.position) << " is overridden within set play " << TypeRegistry::getEnumName(setPlay) << " but does not exist within tactic " << TypeRegistry::getEnumName(tactic) << ".");
    else if(existingPositions[position.position] != 2)
      FAIL("The position " << TypeRegistry::getEnumName(position.position) << " is overridden multiple times within set play " << TypeRegistry::getEnumName(setPlay) << ".");
    else
      ++existingPositions[position.position];
    for(const Action& action : position.actions)
    {
      if(action.type == Action::Type::numOfTypes)
        FAIL("The position " << TypeRegistry::getEnumName(position.position) << " within set play " << TypeRegistry::getEnumName(setPlay) << " has an unspecified action.");
      if(action.type == Action::Type::pass && action.passTarget.empty())
        FAIL("The position " << TypeRegistry::getEnumName(position.position) << " within set play " << TypeRegistry::getEnumName(setPlay) << " has a pass action without targets.");
      if(action.type == Action::Type::wait)
        numOfWaitActions++;
    }
  }
  if(numOfWaitActions > 1)
    FAIL("The action " << TypeRegistry::getEnumName(Action::Type::wait) << " is used by multiple positions within set play " << TypeRegistry::getEnumName(setPlay) << ", although this active action is bound to the active role of playing the ball.");
  const bool isWaitActionAllowed = isOpponentKickOff(setPlay) || isOpponentPenaltyKick(setPlay);
  if(numOfWaitActions > 0 && !isWaitActionAllowed)
    FAIL("The action " << TypeRegistry::getEnumName(Action::Type::wait) << " is used in an unsupported game state within set play " << TypeRegistry::getEnumName(setPlay) << ". It should only be used in set plays for the opponent team with ready and set states.");
}
