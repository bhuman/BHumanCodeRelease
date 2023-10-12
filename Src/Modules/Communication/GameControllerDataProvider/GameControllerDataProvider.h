/**
 * @file GameControllerDataProvider.h
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/GameControllerData.h"
#include "Representations/Communication/RefereeSignal.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include "Network/UdpComm.h"
#include "Framework/Module.h"

MODULE(GameControllerDataProvider,
{,
  USES(BallModel),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(MotionInfo),
  USES(RefereeSignal),
  USES(RobotPose),
  PROVIDES(GameControllerData),
  DEFINES_PARAMETERS(
  {,
    (int)(2000) gameControllerTimeout, /**< Connected to GameController when packet was received within this period of time (in ms). */
    (int)(1000) aliveDelay, /**< Send an alive signal in this interval of ms. */
    (int)(300) refereeSendDuration, /**< Do not send referee signals later than this after their detection. */
    (int)(100) refereeSendInterval, /**< Interval between sending referee signals. */
  }),
});

class GameControllerDataProvider : public GameControllerDataProviderBase
{
  UdpComm socket; /**< The socket to communicate with the GameController. */
  unsigned whenPacketWasSent = 0; /**< When the last return packet was sent to the GameController. */
  unsigned whenRefereePacketWasSent = 0; /**< When the last referee signal was sent to the GameController. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theGameControllerData The representation updated.
   */
  void update(GameControllerData& theGameControllerData);

  /** Sends the return packet to the GameController. */
  bool sendReturnPacket();

  /** Sends the detected referee signal to the GameController. */
  bool sendRefereePacket();

public:
  /** Initialize data and open socket. */
  GameControllerDataProvider();
};
