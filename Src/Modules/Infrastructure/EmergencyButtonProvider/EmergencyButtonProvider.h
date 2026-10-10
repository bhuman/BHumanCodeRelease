/**
 * @file EmergencyButtonProvider.h
 *
 * This file declares a module that handles the emergency button UDP socket
 *
 * @author Tom Beuke
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/EmergencyButtonState.h"
#include "Representations/Infrastructure/FrameInfo.h"

#ifdef TARGET_ROBOT
#include "Network/UdpComm.h"
#endif

MODULE(EmergencyButtonProvider,
{,
  REQUIRES(FrameInfo),
  PROVIDES(EmergencyButtonState),

  LOADS_PARAMETERS(
  {,
    (int) udpPort, /**< UDP port the button is broadcasting on */
    (unsigned) maxWaitTimeMillis, /**< The amount of ms that can pass without receiving a ping from the button */
  }),
});

class EmergencyButtonProvider : public EmergencyButtonProviderBase
{
private:
  #ifdef TARGET_ROBOT
    UdpComm socket; /**< The socket to communicate with the emergency button server. */
    char socketDataBuffer[16];
    unsigned int lastPingTime = 0;
  #endif


  void update(EmergencyButtonState& buttonstate) override;

public:
  EmergencyButtonProvider();
};
