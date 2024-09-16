/**
 * @file SharedAutonomyHandler.h
 *
 * This file declares a module that handles the communication with a
 * remote PC during the Shared Autonomy Challenge.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/SharedAutonomyChannel.h" // include this first to prevent WinSock2.h/Windows.h conflicts
#include "Framework/Module.h"
#include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/BehaviorControl/SharedAutonomyRequest.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JPEGImage.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Streaming/EnumIndexedArray.h"
#include "Streaming/MessageQueue.h"

MODULE(SharedAutonomyHandler,
{,
  REQUIRES(BallModel),
  REQUIRES(CameraInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(JPEGImage),
  REQUIRES(JointSensorData),
  REQUIRES(RawInertialSensorData),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(TeammatesBallModel),
  REQUIRES(JoystickState),
  PROVIDES(JoystickState),
  PROVIDES(SharedAutonomyRequest),
  PROVIDES(SharedAutonomyRequest2),
  USES(SharedAutonomyRequest),
  DEFINES_PARAMETERS(
  {,
    (int)(100) offsetToTeamPort, /**< The offset of the port used to the team port. */
    (unsigned)(25000) bufferSize, /**< The maximum size of a message sent. */
    (unsigned)(3) sendNthFrame, /**< Send every nth frame (per camera). */
  }),
});

class SharedAutonomyHandler : public SharedAutonomyHandlerBase
{
  std::unique_ptr<SharedAutonomyChannel> sharedAutonomyChannel; /**< The channel to the control PC. */

  char* buffer; /**< The buffer that is sent. */
  MessageQueue queue; /**< The queue that writes to the buffer. */
  unsigned frames[CameraInfo::numOfCameras] = {0, 0}; /**< Count frames per camera. */
  SharedAutonomyRequest sharedAutonomyRequest; /**< The last request received. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSharedAutonomyRequest The representation updated.
   */
  void update(SharedAutonomyRequest& theSharedAutonomyRequest) override {theSharedAutonomyRequest = sharedAutonomyRequest;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSharedAutonomyRequest The representation updated.
   */
  void update(SharedAutonomyRequest2& theSharedAutonomyRequest2) override {static_cast<SharedAutonomyRequest&>(theSharedAutonomyRequest2) = sharedAutonomyRequest;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJoystickState The representation updated.
   */
  void update(JoystickState& theJoystickState) override;

public:
  SharedAutonomyHandler();
  ~SharedAutonomyHandler() {delete [] buffer;}
};
