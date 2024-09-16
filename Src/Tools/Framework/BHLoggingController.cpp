/**
 * @file BHLoggingController.cpp
 *
 * This file implements the class \c BHLoggingController.
 *
 * @author Arne Hasselbring
 */

#include "BHLoggingController.h"
#include "Representations/Infrastructure/GameState.h"
#include "Framework/Blackboard.h"
#include "Platform/Time.h"
#include "Streaming/InStreams.h"

BHLoggingController::BHLoggingController() :
  gameState(Blackboard::getInstance().alloc<GameState>("GameState"))
{
  InMapFile stream("teamList.cfg");
  if(stream.exists())
  {
    TeamList teamList;
    stream >> teamList;
    for(const Team& team : teamList.teams)
      teams[team.number] = team.name;
  }
}

BHLoggingController::~BHLoggingController()
{
  Blackboard::getInstance().free("GameState");
}

bool BHLoggingController::shouldLog(bool wasLogging) const
{
  // We want to log in most states, but there are exceptions:
  // - The INITIAL state is excluded.
  // - The FINISHED state is excluded, except for the first 10s if we have been logging before.
  return !(gameState.isInitial(/* orStandby: */ false)
           || (gameState.isFinished() && !(wasLogging && Time::getTimeSince(gameState.timeWhenStateStarted) < 10000)));
}

std::string BHLoggingController::getDescription() const
{
  if(gameState.gameControllerActive)
  {
    auto team = teams.find(static_cast<std::uint8_t>(gameState.opponentTeam.number));
    if(team != teams.end())
      return team->second + "_"
             + (gameState.phase == GameState::penaltyShootout ? "ShootOut"
                : gameState.phase == GameState::firstHalf ? "1stHalf" : "2ndHalf");
  }
  return "Testing";
}
