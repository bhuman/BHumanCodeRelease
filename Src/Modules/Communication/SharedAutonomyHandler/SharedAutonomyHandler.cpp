/**
 * @file SharedAutonomyHandler.cpp
 *
 * This file implements a module that handles the communication with a
 * remote PC during the Shared Autonomy Challenge.
 *
 * @author Thomas Röfer
 */

#include "SharedAutonomyHandler.h"
#include "Framework/Settings.h"
#include "Streaming/Global.h"

MAKE_MODULE(SharedAutonomyHandler);

SharedAutonomyHandler::SharedAutonomyHandler()
  : buffer(new char[bufferSize])
{
  queue.setBuffer(buffer, 0, bufferSize);

  if(!(theGameState.playerNumber & 1))
  {
    sharedAutonomyChannel = std::make_unique<SharedAutonomyChannel>();
#ifndef TARGET_ROBOT
    sharedAutonomyChannel->startLocal(Settings::getPortForTeam(Global::getSettings().teamNumber) + offsetToTeamPort,
                                      static_cast<unsigned>(Global::getSettings().playerNumber));
#else
    sharedAutonomyChannel->start(Settings::getPortForTeam(Global::getSettings().teamNumber) + offsetToTeamPort,
                                 SharedAutonomyChannel::sendBack);
#endif
  }
}

void SharedAutonomyHandler::update(JoystickState& theJoystickState)
{
  // Is this the autonomous robot?
  if(!sharedAutonomyChannel)
  {
    sharedAutonomyRequest.isValid = false;
    return;
  }

  for(;;)
  {
    const int size = sharedAutonomyChannel->receive(buffer, bufferSize);
    if(size <= 0)  // no packet available -> stop
      break;
    else if(static_cast<unsigned>(size) < bufferSize)
    {
      MessageQueue queue;
      queue.setBuffer(buffer, size);
      if(!queue.empty() && (*queue.begin()).id() != idFrameBegin)
        for(MessageQueue::Message m : queue)
          switch(m.id())
          {
            case idSharedAutonomyRequest:
              m.bin() >> sharedAutonomyRequest;
              break;
            case idJoystickState:
              m.bin() >> theJoystickState;
          }
    }
  }

  if(theGameState.playerState == GameState::unstiff)
  {
    sharedAutonomyRequest.isValid = false;
    theJoystickState = JoystickState();
    return;
  }

  if(theJPEGImage.timestamp && ++frames[theCameraInfo.camera] % sendNthFrame == 0)
  {
    queue.clear();
    const char* threadName = theCameraInfo.camera == CameraInfo::upper ? "Upper" : "Lower";
    queue.bin(idFrameBegin) << threadName;
    queue.bin(idJPEGImage) << theJPEGImage;
    queue.bin(idFrameFinished) << threadName;
    queue.bin(idFrameBegin) << "Cognition";
    queue.bin(idGameState) << theGameState;
    queue.bin(idBallModel) << theBallModel;
    queue.bin(idRobotPose) << theRobotPose;
    queue.bin(idTeammatesBallModel) << theTeammatesBallModel;
    queue.bin(idJointSensorData) << theJointSensorData;
    queue.bin(idRawInertialSensorData) << theRawInertialSensorData;
    queue.bin(idGlobalOpponentsModel) << theGlobalOpponentsModel;
    queue.bin(idGlobalTeammatesModel) << theGlobalTeammatesModel;
    const_cast<SharedAutonomyRequest&>(theSharedAutonomyRequest).teammatePlaysBall = !theTeamData.teammates.empty() && theTeamData.teammates.front().theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall);
    queue.bin(idSharedAutonomyRequest) << theSharedAutonomyRequest;
    queue.bin(idFrameFinished) << "Cognition";
    sharedAutonomyChannel->send(buffer, static_cast<int>(queue.end() - queue.begin()));
  }
}
