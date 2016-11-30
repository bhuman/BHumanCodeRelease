/*
 * Sending data to the CommTester for the No WiFi Challange.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "SPLNoWifiChallenge.h"
#include "Representations/Communication/NoWirelessDataSenderStatus.h"
#include "Representations/Communication/NoWirelessReturnData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Tools/Communication/TcpComm.h"
#include "Tools/Module/Module.h"

MODULE(NoWirelessDataSender,
{,
  REQUIRES(BehaviorStatus),
  REQUIRES(NoWirelessReturnData),
  REQUIRES(RobotInfo),
  PROVIDES(NoWirelessDataSenderStatus),
  LOADS_PARAMETERS(
  {,
    (std::string) ip,
  }),
});

class NoWirelessDataSender : public NoWirelessDataSenderBase
{
  TcpComm* tcpComm = nullptr;
  BehaviorStatus::Activity lastActivity = BehaviorStatus::unknown;
  size_t bytesSent = 0;
  unsigned timeOfLastLocationData = 0;

  void update(NoWirelessDataSenderStatus& noWirelessDataSenderStatus);
  void checkConnection();
};
