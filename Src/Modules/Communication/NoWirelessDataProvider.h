/*
 * Receiving data from the CommTester for the No WiFi Challange.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Communication/NoWirelessReceivedData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Communication/TcpComm.h"
#include "Tools/Module/Module.h"

MODULE(NoWirelessDataProvider,
{,
  REQUIRES(FrameInfo),
  PROVIDES(NoWirelessReceivedData),
});

class NoWirelessDataProvider : public NoWirelessDataProviderBase
{
  TcpComm tcpComm;

  void update(NoWirelessReceivedData& noWirelessReceivedData);

public:
  NoWirelessDataProvider() : tcpComm(nullptr, SPL_NO_WIFI_DEFAULT_TRANSMITTER_ROBOT_PORT) {}
};
