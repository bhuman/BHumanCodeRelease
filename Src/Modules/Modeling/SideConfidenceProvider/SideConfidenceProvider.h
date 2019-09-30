/**
 * @file SideConfidenceProvider.h
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(SideConfidenceProvider,
{,
  USES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(OwnSideModel),
  REQUIRES(Odometer),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(WorldModelPrediction),
  USES(TeamBehaviorStatus),
  REQUIRES(CognitionStateChanges),
  PROVIDES(SideConfidence),
  LOADS_PARAMETERS(
  {,
    (float) centerBanZoneRadius,           /**< All balls closer to the field's center than this are ignored */
    (float) fieldLineBanDistance,          /**< All balls that are too close to a line or a penalty mark become ignored */
    (float) maxBallVelocity,               /**< Maximum velocity of balls that are considered for side confidence computation */
    (int)   maxBallAge,                    /**< Maximum age (in milliseconds) of a ball model that can be integrated */
    (int)   agreementTimeout,              /**< Time after which an agreement is removed from the list, if it has not been updated */
    (float) maxBallAgreementDistance,      /**< Between balls that are considerered to be "the same" should not be a distance larger than this */
    (bool)  deactivateGoalieFlipping,      /**< If true, goalie does not set the mirror flag */
    (bool)  goalieIsAlwaysRight,           /**< If true, (dis)agreement of the goalie is always used to decide the confidence */
    (int)   minAgreementCount,             /**< Minimum number of required agreements for considering an agreemate */
    (int)   jumpRemovalTimeout,            /**< If a teammate has jumped recently (or I have jumped), agreements become deleted */
  }),
});

class SideConfidenceProvider : public SideConfidenceProviderBase
{
public:
  /** Constructor */
  SideConfidenceProvider();

private:
  ENUM(TeammateBallCompatibility,
  {,
    Agree,
    Mirror,
    Unknown,
  });

  struct Agreemate
  {
    int number;
    bool isGoalkeeper;
    unsigned timeOfLastAgreement;
    TeammateBallCompatibility ballCompatibility;
    int agreementCount;
    Vector2f lastGlobalBallPosition;  // For visualization only
  };

  std::vector<Agreemate> agreemates;

  unsigned timeWhenLastBallReplacingSetPlayStarted = 0; /** The last timestamp when a set play started in which a ball is replaced. */

  /**
   * Provides the sideConfidence
   */
  void update(SideConfidence& sideConfidence) override;

  void fillRepresentation(SideConfidence& sideConfidence);

  void findAgreemates();

  void removeOldAndJumpedAgreemates();

  void addToAgreemateList(const Teammate& teammate, TeammateBallCompatibility ballCompatibility);

  TeammateBallCompatibility compareBallWithTeammateBall(const Vector2f& globalBallPosition,
                                                        const Vector2f& globalBallPositionMirrored,
                                                        const Vector2f& teammateGlobalBallPosition) const;

  bool ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const;

  bool ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const;

  void draw();
};
