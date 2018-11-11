/**
 * @file TeamMessageHandler.h
 *
 * Declares a module, that provide the port between SPLStandardMessage
 *          and the B-Human data-systems (Representations)
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
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
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Communication/BNTP.h"

MODULE(TeamMessageHandler,
{,
  // v- using for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(GameInfo),
  USES(OwnTeamInfo),
  USES(MotionRequest),

  // v- using for teamout
  USES(FallDownState),
  USES(GroundContactState),
  USES(RawGameInfo),
  USES(RobotInfo),

  // v- directly sliding into teamout
  USES(BallModel),
  USES(FieldCoverage),
  USES(FieldFeatureOverview),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(SideConfidence),
  USES(Whistle),

  PROVIDES(BHumanMessageOutputGenerator),
  PROVIDES(TeamData),

  DEFINES_PARAMETERS(
  {,
    (int)(333) sendInterval, /**<  Time in ms between two messages that are sent to the teammates */
    (int)(4000) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */

    (int)(5000) minTimeBetween2RejectSounds, /*< Time in ms after which another sound output is allowed */
  }),
});

//#define SELF_TEST_TeamMessageHandler

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamMessageHandler : public TeamMessageHandlerBase
{
public:
  TeamMessageHandler() : TeamMessageHandlerBase(), theBNTP(theFrameInfo, theRobotInfo) {}

private:
  BNTP theBNTP;

  // v- output stuff
  mutable unsigned timeLastSent = 0;

  void update(BHumanMessageOutputGenerator& outputGenerator) override;
  void generateMessage(BHumanMessageOutputGenerator& outputGenerator) const;
  void writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const;

  // v- input stuff
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

  void update(TeamData& teamData) override;
  void maintainBMateList(TeamData& teamData) const;

  unsigned timeWhenLastMimimi = 0;
  bool readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m);
  Teammate& getBMate(TeamData& teamData) const;
  void parseMessageIntoBMate(Teammate& bMate);
};
