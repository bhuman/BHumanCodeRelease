#include "NoWirelessDataSender.h"

MAKE_MODULE(NoWirelessDataSender, communication)

void NoWirelessDataSender::update(NoWirelessDataSenderStatus& noWirelessDataSenderStatus)
{
  if(theBehaviorStatus.activity != lastActivity)
  {
    bytesSent = 0;
    lastActivity = theBehaviorStatus.activity;
  }

  if(theBehaviorStatus.activity == BehaviorStatus::noWifi)
  {
    if(tcpComm)
    {
      delete tcpComm;
      tcpComm = nullptr;
    }
  }
  else if(theRobotInfo.number == 1)
  {
    checkConnection();

    if(!tcpComm)
    {
      tcpComm = new TcpComm(ip.c_str(), SPL_NO_WIFI_COMMS_TESTER_PORT);
      checkConnection();
    }
    
    if(tcpComm && ((theBehaviorStatus.activity == BehaviorStatus::noWifiLocation
                    && theNoWirelessReturnData.timeOfLastLocationData != timeOfLastLocationData)
                   || (theBehaviorStatus.activity == BehaviorStatus::noWifiData
                       && theNoWirelessReturnData.data.size() > bytesSent)))
    {
      SPLNoWifiPacket packet;
      if(theBehaviorStatus.activity == BehaviorStatus::noWifiLocation)
      {
        packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION;
        packet.payload.location.x = static_cast<short>(theNoWirelessReturnData.positionToPointAt.x());
        packet.payload.location.y = static_cast<short>(theNoWirelessReturnData.positionToPointAt.y());
        timeOfLastLocationData = theNoWirelessReturnData.timeOfLastLocationData;
        tcpComm->send(reinterpret_cast<unsigned char*>(&packet), sizeof(packet.header) + sizeof(packet.payload.location));
      }
      else
      {
        packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_DATA;
        packet.payload.data.header.fragmentOffset = static_cast<unsigned short>(bytesSent);
        packet.payload.data.header.fragmentLength = static_cast<unsigned short>(theNoWirelessReturnData.data.size() - bytesSent);
        std::memcpy(packet.payload.data.data, &theNoWirelessReturnData.data[bytesSent], theNoWirelessReturnData.data.size() - bytesSent);
        bytesSent = theNoWirelessReturnData.data.size();
        tcpComm->send(reinterpret_cast<unsigned char*>(&packet), offsetof(SPLNoWifiPacket, payload.data.data) + packet.payload.data.header.fragmentLength);
      }
    }
  }

  noWirelessDataSenderStatus.connected = tcpComm && tcpComm->connected();
}

void NoWirelessDataSender::checkConnection()
{
  if(tcpComm)
  {
    unsigned char c;
    tcpComm->receive(&c, 1, false);
    if(!tcpComm->connected())
    {
      delete tcpComm;
      tcpComm = nullptr;
    }
  }
}
