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

MAKE_MODULE(GameDataProvider, communication)

GameDataProvider::GameDataProvider()
{
  // Initialize GameController packet
  memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  gameCtrlData.teams[0].teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  gameCtrlData.teams[0].teamColor = static_cast<uint8_t>(Global::getSettings().teamColor);
  gameCtrlData.teams[1].teamColor = gameCtrlData.teams[0].teamColor ^ 1; // we don't know better
  gameCtrlData.playersPerTeam = static_cast<uint8_t>(Global::getSettings().playerNumber); // we don't know better

  // Initialize socket
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", GAMECONTROLLER_DATA_PORT));
}

void GameDataProvider::update(RobotInfo& theRobotInfo)
{
  if(receive())
    whenPacketWasReceived = theFrameInfo.time;

  if(theFrameInfo.getTimeSince(whenPacketWasReceived) < gameControllerTimeout
     && theFrameInfo.getTimeSince(whenPacketWasSent) >= aliveDelay
     && sendAliveMessage())
    whenPacketWasSent = theFrameInfo.time;

  handleButtons();

  const RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  static_cast<RoboCup::RobotInfo&>(theRobotInfo) = team.players[Global::getSettings().playerNumber - 1];
  theRobotInfo.number = Global::getSettings().playerNumber;

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
  theRawGameInfo.timeLastPacketReceived = whenPacketWasReceived;
  if(Global::getSettings().teamNumber >= 90)
    theRawGameInfo.competitionType = COMPETITION_TYPE_MIXEDTEAM;
}

bool GameDataProvider::receive()
{
  bool received = false;
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
      gameCtrlData = buffer;
#ifdef TARGET_ROBOT
      unsigned ip = htonl(from);
      socket.setTarget(inet_ntoa(reinterpret_cast<in_addr&>(ip)), GAMECONTROLLER_RETURN_PORT);
#endif
      received = true;
    }
  }
  return received;
}

bool GameDataProvider::sendAliveMessage()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.team = (uint8_t) Global::getSettings().teamNumber;
  returnPacket.player = (uint8_t) Global::getSettings().playerNumber;
  returnPacket.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
  return socket.write(reinterpret_cast<char*>(&returnPacket), sizeof(returnPacket));
}

void GameDataProvider::handleButtons()
{
  RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];

  bool chestButtonPressed = theKeyStates.pressed[KeyStates::chest];
  bool chestButtonReleased = previousChestButtonPressed && !chestButtonPressed;

  if(!previousChestButtonPressed && chestButtonPressed)
    whenChestButtonPressed = theFrameInfo.time;

  if(chestButtonReleased && theFrameInfo.getTimeSince(whenChestButtonStateChanged) >= buttonDelay)
  {
    whenChestButtonReleased = theFrameInfo.time;
    chestButtonPressCounter = (chestButtonPressCounter + 1) % 3; // ignore triple press of a button, e.g. for sitting down
    if(chestButtonPressCounter == 0)
      whenChestButtonStateChanged = 0; // reset last chest button state change to ignore the next press
  }

  if(chestButtonPressCounter > 0
     && theFrameInfo.getTimeSince(whenChestButtonStateChanged) >= buttonDelay
     && theFrameInfo.getTimeSince(whenChestButtonPressed) < chestButtonPressDuration)
  {
    if(theFrameInfo.getTimeSince(whenChestButtonReleased) >= chestButtonTimeout)
    {
      if(whenChestButtonStateChanged && theFrameInfo.getTimeSince(whenPacketWasReceived) >= gameControllerTimeout)  // ignore first press, e.g. for getting up
      {
        RoboCup::RobotInfo& player = team.players[Global::getSettings().playerNumber - 1];
        if(player.penalty == PENALTY_NONE)
          player.penalty = PENALTY_MANUAL;
        else
        {
          player.penalty = PENALTY_NONE;
          gameCtrlData.state = STATE_PLAYING;
        }
      }
      whenChestButtonStateChanged = theFrameInfo.time;
      chestButtonPressCounter = 0;
    }
  }

  previousChestButtonPressed = chestButtonPressed;

  if(gameCtrlData.state == STATE_INITIAL && team.players[Global::getSettings().playerNumber - 1].penalty == PENALTY_NONE)
  {
    bool leftFootButtonPressed = theKeyStates.pressed[KeyStates::lFootLeft] != 0.f || theKeyStates.pressed[KeyStates::lFootRight] != 0.f;
    if(leftFootButtonPressed != previousLeftFootButtonPressed
       && theFrameInfo.getTimeSince(whenLeftFootButtonStateChanged) >= buttonDelay)
    {
      if(leftFootButtonPressed)
        team.teamColor = (team.teamColor + 1) % (TEAM_GRAY + 1); // cycle between TEAM_BLUE .. TEAM_GRAY
      previousLeftFootButtonPressed = leftFootButtonPressed;
      whenLeftFootButtonStateChanged = theFrameInfo.time;
    }

    bool rightFootButtonPressed = theKeyStates.pressed[KeyStates::rFootLeft] != 0.f || theKeyStates.pressed[KeyStates::rFootRight] != 0.f;
    if(rightFootButtonPressed != previousRightFootButtonPressed
       && theFrameInfo.getTimeSince(whenRightFootButtonStateChanged) >= buttonDelay)
    {
      if(rightFootButtonPressed)
      {
        if(gameCtrlData.gamePhase == GAME_PHASE_NORMAL)
        {
          gameCtrlData.gamePhase = GAME_PHASE_PENALTYSHOOT;
          gameCtrlData.kickingTeam = team.teamNumber;
        }
        else if(gameCtrlData.kickingTeam == team.teamNumber)
          gameCtrlData.kickingTeam = 0;
        else
          gameCtrlData.gamePhase = GAME_PHASE_NORMAL;
      }
      previousRightFootButtonPressed = rightFootButtonPressed;
      whenRightFootButtonStateChanged = theFrameInfo.time;
    }
  }
}
