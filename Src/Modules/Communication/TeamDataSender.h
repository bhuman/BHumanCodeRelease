/**
 * @file TeamDataSender.h
 * Declaration of module TeamDataSender
 * @author Colin Graf
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Communication/NetworkThumbnail.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"

MODULE(TeamDataSender,
{,
  REQUIRES(BallModel),
  REQUIRES(BehaviorStatus),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FieldCoverage),
  REQUIRES(FieldFeatureOverview),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(NetworkThumbnail),
  REQUIRES(ObstacleModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotHealth),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(SideConfidence),
  REQUIRES(SPLStandardBehaviorStatus),
  REQUIRES(TeammateRoles),
  REQUIRES(TeammateData),
  REQUIRES(Whistle),
  PROVIDES_WITHOUT_MODIFY(TeamDataSenderOutput),
  LOADS_PARAMETERS(
  {,
    (unsigned) maxNumberOfObstaclesToSend, /**< Do not send more obstacles than this. */
  }),
});

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamDataSender : public TeamDataSenderBase
{
private:
  unsigned sendFrames = 0; /** Quantity of frames in which team data was sent */
  unsigned lineToSendNext = 0; /**< Field coverage line to send next. */

  /**
   * The update function called in each cognition process cycle
   * @param teamDataSenderOutput An empty output representation
   */
  virtual void update(TeamDataSenderOutput& teamDataSenderOutput);
};
