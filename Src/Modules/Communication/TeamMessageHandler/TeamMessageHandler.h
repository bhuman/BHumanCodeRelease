/**
 * @file TeamMessageHandler.h
 *
 * Declares a module, that provide the port between SPLStandardMessage
 *          and the B-Human data-systems (Representations)
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeamTalk.h"
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
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Communication/BNTP.h"
#include "Tools/Communication/CompressedTeamCommunicationStreams.h"
#include "Tools/Communication/RobotStatus.h"

MODULE(TeamMessageHandler,
{,
  // for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  USES(OwnTeamInfo),
  USES(MotionRequest),
  USES(RawGameInfo),
  USES(TeamData),

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  USES(RobotInfo),

  // send directly
  USES(BallModel),
  USES(BehaviorStatus),
  USES(FieldCoverage),
  REQUIRES(FieldFeatureOverview),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(TeamBehaviorStatus),
  USES(TeamTalk),
  USES(Whistle),

  PROVIDES(BHumanMessageOutputGenerator),
  PROVIDES(TeamData),

  LOADS_PARAMETERS(
  {,
    (int) sendInterval, /**<  Time in ms between two messages that are sent to the teammates */
    (int) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
  }),
});

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamMessageHandler : public TeamMessageHandlerBase
{
public:
  TeamMessageHandler();

private:
  BNTP theBNTP;
  mutable RobotStatus theRobotStatus;

  CompressedTeamCommunication::TypeRegistry teamCommunicationTypeRegistry;
  const CompressedTeamCommunication::Type* teamMessageType;

  // output stuff
  mutable unsigned timeLastSent = 0;

  void update(BHumanMessageOutputGenerator& outputGenerator) override;
  void generateMessage(BHumanMessageOutputGenerator& outputGenerator) const;
  void writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const;

  // input stuff
  struct ReceivedBHumanMessage : public BHumanMessage
  {
    const SynchronizationMeasurementsBuffer* bSMB = nullptr;
    unsigned toLocalTimestamp(unsigned remoteTimestamp) const override
    {
      if(bSMB)
        return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
      else
        return 0u;
    };

    enum ErrorCode
    {
      //add more parsing errors if there is a need of distinguishing
      parsingError,
      magicNumberDidNotMatch,
      myOwnMessage
    } lastErrorCode;
  } receivedMessageContainer;

  static void regTeamMessage();

  void update(TeamData& teamData) override;
  void maintainBMateList(TeamData& teamData) const;

  unsigned timeWhenLastMimimi = 0;
  bool readSPLStandardMessage(const SPLStandardMessageBufferEntry* const m);
  Teammate& getBMate(TeamData& teamData) const;
  void parseMessageIntoBMate(Teammate& bMate);
};
