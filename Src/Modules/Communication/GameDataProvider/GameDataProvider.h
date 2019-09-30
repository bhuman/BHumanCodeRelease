/**
 * @file GameDataProvider.h
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/Communication/UdpComm.h"
#include "Tools/Module/Module.h"

MODULE(GameDataProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(KeyStates),
  REQUIRES(RobotInfo),
  PROVIDES(RobotInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(RawGameInfo),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) chestButtonPressDuration, /**< Chest button state changes are ignored when happening in more than this period (in ms). */
    (int)(300) chestButtonTimeout, /**< Accept chest button press when chest button was not pressed again within this period (in ms). */
    (int)(30) buttonDelay, /**< Button state changes are ignored when happening in less than this period (in ms). */
    (int)(2000) gameControllerTimeout, /**< Connected to GameController when packet was received within this period of time (in ms). */
    (int)(1000) aliveDelay, /**< Send an alive signal in this interval of ms. */
  }),
});

class GameDataProvider : public GameDataProviderBase
{
  UdpComm socket; /**< The socket to communicate with the GameController. */
  RoboCup::RoboCupGameControlData gameCtrlData; /**< The local copy of the GameController packet. */
  bool previousChestButtonPressed = false; /**< Whether the chest button was pressed during the previous cycle. */
  bool previousLeftFootButtonPressed = false; /**< Whether the left foot bumper was pressed during the previous cycle. */
  bool previousRightFootButtonPressed = false; /**< Whether the right foot bumper was pressed during the previous cycle. */
  unsigned whenChestButtonStateChanged = 0; /**< When last state change of the chest button occured. */
  unsigned whenChestButtonPressed = 0; /**< When the chest button was pressed. */
  unsigned whenChestButtonReleased = 0;/**< When the chest button was released. */
  unsigned whenLeftFootButtonStateChanged = 0; /**< When last state change of the left foot bumper occured. */
  unsigned whenRightFootButtonStateChanged = 0; /**< When last state change of the right foot bumper occured. */
  unsigned whenPacketWasReceived = 0; /**< When the last GameController packet was received. */
  unsigned whenPacketWasSent = 0; /**< When the last return packet was sent to the GameController. */
  int chestButtonPressCounter = 0; /**< Counter for pressing the chest button*/

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRobotInfo The representation updated.
   */
  void update(RobotInfo& theRobotInfo);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOwnTeamInfo The representation updated.
   */
  void update(OwnTeamInfo& theOwnTeamInfo);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOpponentTeamInfo The representation updated.
   */
  void update(OpponentTeamInfo& theOpponentTeamInfo);

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRawGameInfo The representation updated.
   */
  void update(RawGameInfo& theRawGameInfo);

  /**
   * Receives a packet from the GameController.
   * Packets are only accepted when the team number is know (nonzero) and
   * they are addressed to this team.
   */
  bool receive();

  /** Sends the alive message to the GameController. */
  bool sendAliveMessage();

  /** Handle the official button interface. */
  void handleButtons();

public:
  /** Initialize data and open socket. */
  GameDataProvider();
};
