/**
 * @file GameDataProvider.h
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Platform/SystemCall.h"
#include "Tools/Communication/UdpComm.h"
#include "Tools/Module/Module.h"

MODULE(GameDataProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  PROVIDES(RobotInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(RawGameInfo),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(1000) unstiffHeadButtonPressDuration, /**< How long the head buttons need to be pressed until the robot transitions to unstiff (in ms). */
    (unsigned)(200) calibrationHeadButtonPressDuration, /**< How long the front head button needs to be pressed until the robot transitions to calibration (in ms). */
    (int)(7777) unstiffFinishedDuration, /**< How long the game state needs to be finished until the robot transitions to unstiff (in ms). */
    (int)(2000) gameControllerTimeout, /**< Connected to GameController when packet was received within this period of time (in ms). */
    (int)(1000) aliveDelay, /**< Send an alive signal in this interval of ms. */
  }),
});

class GameDataProvider : public GameDataProviderBase
{
  UdpComm socket; /**< The socket to communicate with the GameController. */
  RoboCup::RoboCupGameControlData gameCtrlData; /**< The local copy of the GameController packet. */
  unsigned whenGameCtrlDataWasSet = 0; /**< When the local copy of the GameController packet was set. */
  unsigned whenPacketWasReceived = 0; /**< When the last GameController packet was received. */
  unsigned whenPacketWasSent = 0; /**< When the last return packet was sent to the GameController. */
  unsigned whenStateNotFinished = 0; /**< Last time when the state was something else than finished. */
  RobotInfo::Mode mode = SystemCall::getMode() == SystemCall::physicalRobot ? RobotInfo::unstiff : RobotInfo::active; /**< The current robot mode. */
  bool ignoreChestButton = false; /**< Whether the chest button should be ignored in \c handleButtons. */

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
   * @param setGameCtrlData Whether the received data shall overwrite the local state.
   */
  void receive(bool setGameCtrlData);

  /** Sends the alive message to the GameController. */
  bool sendAliveMessage();

  /** Handle the official button interface. */
  void handleButtons();

  /** Resets the internal game state. */
  void resetGameCtrlData();

public:
  /** Initialize data and open socket. */
  GameDataProvider();
};
