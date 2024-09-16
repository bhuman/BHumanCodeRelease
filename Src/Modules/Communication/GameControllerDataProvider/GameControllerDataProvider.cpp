/**
 * @file GameControllerDataProvider.cpp
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#include "GameControllerDataProvider.h"
#include "Framework/Settings.h"

#ifdef WINDOWS
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <ws2ipdef.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#include <cstring>

MAKE_MODULE(GameControllerDataProvider);

GameControllerDataProvider::GameControllerDataProvider()
{
  // Initialize socket
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", GAMECONTROLLER_DATA_PORT));
}

void GameControllerDataProvider::update(GameControllerData& theGameControllerData)
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
      unsigned ip = htonl(from);
      char addressBuffer[INET_ADDRSTRLEN];
      VERIFY(inet_ntop(AF_INET, &ip, addressBuffer, INET_ADDRSTRLEN) == addressBuffer);
      socket.setTarget(addressBuffer, GAMECONTROLLER_RETURN_PORT);
      static_cast<RoboCup::RoboCupGameControlData&>(theGameControllerData) = buffer;
      theGameControllerData.timeLastPacketReceived = socket.getLastReadTimestamp();
      theGameControllerData.isTrueData = false;
    }
  }

  if(theFrameInfo.getTimeSince(theGameControllerData.timeLastPacketReceived) < gameControllerTimeout &&
     theFrameInfo.getTimeSince(whenPacketWasSent) >= aliveDelay &&
     sendReturnPacket())
    whenPacketWasSent = theFrameInfo.time;
}

bool GameControllerDataProvider::sendReturnPacket()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  returnPacket.playerNum = static_cast<std::uint8_t>(Global::getSettings().playerNumber);
  returnPacket.teamNum = static_cast<std::uint8_t>(Global::getSettings().teamNumber);
  returnPacket.fallen = !(theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionInfo.executedPhase != MotionPhase::fall) ||
                        !(theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting) ?
                        1 : 0;
  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;
  returnPacket.ballAge = theBallModel.timeWhenLastSeen ? static_cast<float>(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)) / 1000.f : -1.f;;
  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();
  return socket.write(reinterpret_cast<char*>(&returnPacket), sizeof(returnPacket));
}
