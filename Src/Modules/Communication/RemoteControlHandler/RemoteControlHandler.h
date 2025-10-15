/**
 * @file RemoteControlHandler.h
 *
 * This file declares a module that handles the communication with a remote PC.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/RemoteControlChannel.h" // include this first to prevent WinSock2.h/Windows.h conflicts
#include "Framework/Module.h"
#include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Representations/Sensing/InertialData.h"
#include "Streaming/MessageQueue.h"

MODULE(RemoteControlHandler,
{,
  REQUIRES(BallModel),
  REQUIRES(BallPercept),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(InertialData),
  REQUIRES(JPEGImage),
  REQUIRES(JointSensorData),
  REQUIRES(LibDemo),
  REQUIRES(LinesPercept),
  REQUIRES(ObstaclesFieldPercept),
  REQUIRES(ObstaclesImagePercept),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(JoystickState),
  PROVIDES(JoystickState),
  LOADS_PARAMETERS(
  {,
    (int) offsetToTeamPort, /**< The offset of the port used to the team port. */
    (unsigned) bufferSize, /**< The maximum size of a message sent. */
    (unsigned) sendNthFrame, /**< Send every nth frame (per camera). */
    (std::vector<int>) acceptingPlayerNumbers, /**< The player numbers that accept a connection. */
    (int) joystickTimeout, /**< The duration without joystick update after which its state is invalid. */
  }),
});

class RemoteControlHandler : public RemoteControlHandlerBase
{
  std::unique_ptr<RemoteControlChannel> remoteControlChannel; /**< The channel to the control PC. */

  char* buffer; /**< The buffer that is sent. */
  MessageQueue queue; /**< The queue that writes to the buffer. */
  unsigned frames[CameraInfo::numOfCameras] = {0, 0}; /**< Count frames per camera. */
  unsigned timeWhenLastJoystickStateReceived = 0; /**< Time when the last joystick state was received from control PC. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJoystickState The representation updated.
   */
  void update(JoystickState& theJoystickState) override;

public:
  RemoteControlHandler();
  ~RemoteControlHandler() {delete [] buffer;}
};
