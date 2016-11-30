#include "NoWirelessDataProvider.h"

MAKE_MODULE(NoWirelessDataProvider, communication)

void NoWirelessDataProvider::update(NoWirelessReceivedData& noWirelessReceivedData)
{
  SPLNoWifiPacket packet;
  if(tcpComm.receive(reinterpret_cast<unsigned char*>(&packet.header), sizeof(packet.header), false))
  {
    if(packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION
       && tcpComm.receive(reinterpret_cast<unsigned char*>(&packet.payload.location), sizeof(packet.payload.location), true))
    {
      noWirelessReceivedData.type = NoWirelessReceivedData::locationType;
      noWirelessReceivedData.data.clear();
      noWirelessReceivedData.data.push_back(static_cast<unsigned char>(((packet.payload.location.x + 4500) * 255 + 4500) / 9000));
      noWirelessReceivedData.data.push_back(static_cast<unsigned char>(((packet.payload.location.y + 3000) * 255 + 3000) / 6000));
      noWirelessReceivedData.lastReceived = theFrameInfo.time;
    }
    else if(packet.header.type == SPL_NO_WIFI_PAYLOAD_TYPE_DATA
            && tcpComm.receive(reinterpret_cast<unsigned char*>(&packet.payload.location), sizeof(packet.payload.data), true))
    {
      noWirelessReceivedData.type = NoWirelessReceivedData::dataType;
      noWirelessReceivedData.data.resize(SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN);
      std::memcpy(noWirelessReceivedData.data.data(), packet.payload.data.data, SPL_NO_WIFI_DATA_PAYLOAD_MAX_LEN);
      noWirelessReceivedData.lastReceived = theFrameInfo.time;
    }
  }
  noWirelessReceivedData.connected = tcpComm.connected();
}
