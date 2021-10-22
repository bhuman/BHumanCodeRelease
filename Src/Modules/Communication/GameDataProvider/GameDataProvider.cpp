/**
 * @file GameDataProvider.cpp
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#include "GameDataProvider.h"
#include "Tools/Settings.h"
#ifdef TARGET_ROBOT
#include <arpa/inet.h>
#include <netinet/in.h>
#endif
#include <cstring>

MAKE_MODULE(GameDataProvider, communication);

GameDataProvider::GameDataProvider()
{
  // Initialize GameController packet
  resetGameCtrlData();

  // Initialize socket
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", GAMECONTROLLER_DATA_PORT));
}

void GameDataProvider::update(RobotInfo& theRobotInfo)
{
  if(!(gameCtrlData.gamePhase != GAME_PHASE_PENALTYSHOOT && gameCtrlData.state == STATE_FINISHED))
    whenStateNotFinished = theFrameInfo.time;

  ignoreChestButton = false;
  switch(mode)
  {
    case RobotInfo::unstiff:
      if(theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      {
        mode = RobotInfo::active;
        ignoreChestButton = true;
      }
      break;
    case RobotInfo::active:
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theFrameInfo.getTimeSince(whenStateNotFinished) > unstiffFinishedDuration)
      {
        resetGameCtrlData();
        mode = RobotInfo::unstiff;
      }
      else if(gameCtrlData.state == STATE_INITIAL &&
              gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty == PENALTY_NONE &&
              theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
              theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
      {
        resetGameCtrlData();
        gameCtrlData.state = STATE_PLAYING;
        mode = RobotInfo::calibration;
      }
      break;
    case RobotInfo::calibration:
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theBehaviorStatus.activity == BehaviorStatus::calibrationFinished)
      {
        resetGameCtrlData();
        mode = RobotInfo::unstiff;
      }
      break;
  }

  receive(mode == RobotInfo::active);


  if(theFrameInfo.getTimeSince(whenPacketWasReceived) >= gameControllerTimeout)
    handleButtons();
  else if(theFrameInfo.getTimeSince(whenPacketWasSent) >= aliveDelay && sendAliveMessage())
    whenPacketWasSent = theFrameInfo.time;

  const RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  static_cast<RoboCup::RobotInfo&>(theRobotInfo) = team.players[Global::getSettings().playerNumber - 1];
  theRobotInfo.number = Global::getSettings().playerNumber;
  theRobotInfo.mode = mode;

  DEBUG_RESPONSE_ONCE("module:GameDataProvider:robotInfo")
    OUTPUT(idRobotInfo, bin, theRobotInfo);
}

void GameDataProvider::update(OwnTeamInfo& theOwnTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(theOwnTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
}

void GameDataProvider::update(OpponentTeamInfo& theOpponentTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(theOpponentTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0];
}

void GameDataProvider::update(RawGameInfo& theRawGameInfo)
{
  std::memcpy(static_cast<RoboCup::RoboCupGameControlData*>(&theRawGameInfo), &gameCtrlData, sizeof(gameCtrlData));
  theRawGameInfo.timeLastPacketReceived = whenGameCtrlDataWasSet;
}

void GameDataProvider::receive(bool setGameCtrlData)
{
  int size;
  RoboCup::RoboCupGameControlData buffer;
  unsigned from;
  while((size = socket.read(reinterpret_cast<char*>(&buffer), sizeof(buffer), from)) > 0)
  {
    if(size == sizeof(buffer) &&
       !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
       buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
       (buffer.teams[0].teamNumber == Global::getSettings().teamNumber ||
        buffer.teams[1].teamNumber == Global::getSettings().teamNumber))
    {
      if(setGameCtrlData)
      {
        gameCtrlData = buffer;
        whenGameCtrlDataWasSet = theFrameInfo.time;
      }
#ifdef TARGET_ROBOT
      unsigned ip = htonl(from);
      socket.setTarget(inet_ntoa(reinterpret_cast<in_addr&>(ip)), GAMECONTROLLER_RETURN_PORT);
#endif
      whenPacketWasReceived = theFrameInfo.time;
    }
  }
}

bool GameDataProvider::sendAliveMessage()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.team = static_cast<std::uint8_t>(Global::getSettings().teamNumber);
  returnPacket.player = static_cast<std::uint8_t>(Global::getSettings().playerNumber);
  returnPacket.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
  return socket.write(reinterpret_cast<char*>(&returnPacket), sizeof(returnPacket));
}

void GameDataProvider::handleButtons()
{
  RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  RoboCup::RobotInfo& player = team.players[Global::getSettings().playerNumber - 1];

  if(mode == RobotInfo::active && !ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
  {
    if(player.penalty == PENALTY_NONE)
      player.penalty = PENALTY_MANUAL;
    else
    {
      player.penalty = PENALTY_NONE;
      gameCtrlData.state = STATE_PLAYING;
    }
  }

  if(gameCtrlData.state == STATE_INITIAL && player.penalty == PENALTY_NONE)
  {
    if(theEnhancedKeyStates.hitStreak[KeyStates::lFootLeft] == 1)
      team.teamColor = (team.teamColor + 1) % (TEAM_GRAY + 1); // cycle between TEAM_BLUE .. TEAM_GRAY

    if(theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
    {
      if(gameCtrlData.gamePhase == GAME_PHASE_NORMAL)
      {
        gameCtrlData.gamePhase = GAME_PHASE_PENALTYSHOOT;
        gameCtrlData.kickingTeam = team.teamNumber;
        SystemCall::say("Penalty striker");
      }
      else if(gameCtrlData.kickingTeam == team.teamNumber)
      {
        gameCtrlData.kickingTeam = 0;
        SystemCall::say("Penalty keeper");
      }
      else
        gameCtrlData.gamePhase = GAME_PHASE_NORMAL;
    }
  }
}

void GameDataProvider::resetGameCtrlData()
{
  std::memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  gameCtrlData.teams[0].teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  gameCtrlData.teams[0].teamColor = static_cast<uint8_t>(Global::getSettings().teamColor);
  gameCtrlData.teams[1].teamColor = gameCtrlData.teams[0].teamColor ^ 1; // we don't know better
  gameCtrlData.playersPerTeam = static_cast<uint8_t>(Global::getSettings().playerNumber); // we don't know better
  gameCtrlData.firstHalf = 1;
  whenGameCtrlDataWasSet = 0;
}
