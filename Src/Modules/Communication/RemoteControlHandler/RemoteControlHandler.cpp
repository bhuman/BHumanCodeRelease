/**
 * @file RemoteControlHandler.cpp
 *
 * This file implements a module that handles the communication with a remote PC.
 *
 * @author Thomas Röfer
 */

#include "RemoteControlHandler.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"
#include <algorithm>

MAKE_MODULE(RemoteControlHandler);

RemoteControlHandler::RemoteControlHandler()
  : buffer(new char[bufferSize])
{
  queue.setBuffer(buffer, 0, bufferSize);

  if(std::find(acceptingPlayerNumbers.begin(), acceptingPlayerNumbers.end(), theGameState.playerNumber) != acceptingPlayerNumbers.end())
  {
    remoteControlChannel = std::make_unique<RemoteControlChannel>();
#ifndef TARGET_ROBOT
    remoteControlChannel->startLocal(Settings::getPortForTeam(Global::getSettings().teamNumber) + offsetToTeamPort,
                                     static_cast<unsigned>(Global::getSettings().playerNumber));
#else
    remoteControlChannel->start(Settings::getPortForTeam(Global::getSettings().teamNumber) + offsetToTeamPort,
                                RemoteControlChannel::sendBack);
#endif
  }
}

void RemoteControlHandler::update(JoystickState& theJoystickState)
{
  // Is this robot connected?
  if(!remoteControlChannel)
    return;

  for(;;)
  {
    const int size = remoteControlChannel->receive(buffer, bufferSize);
    if(size <= 0)  // no packet available -> stop
      break;
    else if(static_cast<unsigned>(size) < bufferSize)
    {
      MessageQueue queue;
      queue.setBuffer(buffer, size);
      if(!queue.empty() && (*queue.begin()).id() != idFrameBegin)
      {
        for(MessageQueue::Message m : queue)
          switch(m.id())
          {
            case idJoystickState:
              m.bin() >> theJoystickState;
              timeWhenLastJoystickStateReceived = theFrameInfo.time;
          }
      }
    }
  }

  if(theFrameInfo.getTimeSince(timeWhenLastJoystickStateReceived) > joystickTimeout)
    theJoystickState.valid = false;

  if(theGameState.playerState == GameState::unstiff)
  {
    theJoystickState = JoystickState();
    return;
  }

  if(theJPEGImage.timestamp && ++frames[theCameraInfo.camera] % sendNthFrame == 0)
  {
    queue.clear();
    const char* threadName = theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower";
    queue.bin(idFrameBegin) << threadName;
    queue.bin(idJPEGImage) << theJPEGImage;
    if(theLibDemo.isDemoActive || theLibDemo.isOneVsOneDemoActive)
    {
      queue.bin(idBallPercept) << theBallPercept;
      queue.bin(idLinesPercept) << theLinesPercept;
      queue.bin(idPenaltyMarkPercept) << thePenaltyMarkPercept;
      queue.bin(idObstaclesFieldPercept) << theObstaclesFieldPercept;
      queue.bin(idObstaclesImagePercept) << theObstaclesImagePercept;
    }
    queue.bin(idFrameFinished) << threadName;
    queue.bin(idFrameBegin) << "Cognition";
    if(!theLibDemo.isDemoActive && !theLibDemo.isOneVsOneDemoActive)
    {
      queue.bin(idGameState) << theGameState;
      queue.bin(idGlobalTeammatesModel) << theGlobalTeammatesModel;
    }
    queue.bin(idBallModel) << theBallModel;
    queue.bin(idRobotPose) << theRobotPose;
    queue.bin(idTeamBallModel) << theTeamBallModel;
    queue.bin(idJointSensorData) << theJointSensorData;
    queue.bin(idRawInertialSensorData) << static_cast<RawInertialSensorData>(theInertialData);
    queue.bin(idGlobalOpponentsModel) << theGlobalOpponentsModel;
    queue.bin(idFrameFinished) << "Cognition";
    remoteControlChannel->send(buffer, static_cast<int>(queue.end() - queue.begin()));
  }
}
