/**
* @file TeammateReliabilityProvider.h
*
* Declaration of a module that tries to estimate how reliable the robots of the own team are.
* The main application is the SPL drop-in competition.
*
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
*/

#pragma once

#include "Tools/Range.h"
#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/ReceivedSPLStandardMessages.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Modeling/TeammateReliability.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(TeammateReliabilityProvider,
{,
  REQUIRES(ReceivedSPLStandardMessages),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  USES(RobotPose),      // <-- Use old pose to avoid conflicts with module order
  PROVIDES(TeammateReliability),
  LOADS_PARAMETERS(
  {,
    (int) timeout,                  /**< Add some comment here */
    (int) minPackagesNeeded,        /**< Add some comment here */
    (int) minObservationTimeInPlay, /**< Add some comment here */
    (float) expectedMinTranslationRange,
    (float) expectedMinRotationRange,
    (float) maxBallAgreementDistance,
    (int) ballAgreementTimeout,
  }),
});

/**
 * @class TeammateReliabilityProvider
 *
 * Computes the reliability of the current teammates
 */
class TeammateReliabilityProvider : public TeammateReliabilityProviderBase
{
  struct TeammateInformation
  {
    TeammateInformation():firstPackageReceived(0), lastPackageReceived(0), lastTimeAgreedOnBall(0),
                          isPenalized(false), receivedPackagesInReady(0),
                          receivedPackagesInPlaying(0), xPosRange(0.f), yPosRange(0.f), rotRange(0.f),
                          hasInvalidPose(false), seesBallOutsideField(false)
    {
    }

    unsigned firstPackageReceived;
    unsigned lastPackageReceived;
    unsigned lastTimeAgreedOnBall;
    bool isPenalized;
    int receivedPackagesInReady;
    int receivedPackagesInPlaying;
    Rangef xPosRange;
    Rangef yPosRange;
    Rangef rotRange;
    bool hasInvalidPose;
    bool seesBallOutsideField;
  };

  std::vector<TeammateInformation> teammates;

  /** Provides the reliability data*/
  void update(TeammateReliability& teammateReliability);

  void updateTeammateInformation(const SPLStandardMessage& msg, unsigned timestamp, int robotNumber, TeammateInformation& teammate);

  void computeReliability(const TeammateInformation& teammate, TeammateReliability::ReliabilityState& state);
};
