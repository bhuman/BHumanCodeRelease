/**
 * @file ExpSideConfidenceProvider.h
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/TeammateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(ExpSideConfidenceProvider,
{,
  USES(RobotPose),
  REQUIRES(TeammateData),
  REQUIRES(OwnSideModel),
  REQUIRES(Odometer),
  REQUIRES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraMatrix),
  REQUIRES(FallDownState),
  REQUIRES(ArmContactModel),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
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
  }),
});

class ExpSideConfidenceProvider : public ExpSideConfidenceProviderBase
{
public:
  /** Constructor */
  ExpSideConfidenceProvider();

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
    unsigned timeOfLastAgreement;
    TeammateBallCompatibility ballCompatibility;
    Vector2f lastMatePosition;
    std::vector<int> otherAgreements;
  };

  Pose2f robotPose;   /**< The RobotPose from the last frame + odometry offset of this frame */
  std::vector<Agreemate> agreemates;

  /**
  * Provides the sideConfidence
  */
  void update(SideConfidence& sideConfidence);

  void fillRepresentation(SideConfidence& sideConfidence);

  void findAgreemates();

  void removeOldAgreemates();

  void addToAgreemateList(const Teammate& teammate, TeammateBallCompatibility ballCompatibility);

  TeammateBallCompatibility compareBallWithTeammateBall(const Vector2f& globalBallPosition,
                                                        const Vector2f& globalBallPositionMirrored,
                                                        const Vector2f& teammateGlobalBallPosition) const;

  bool ballModelCanBeUsed(const BallModel& ballModel, const Pose2f& robotPose) const;

  bool ballIsCloseToPenaltyMarkOrLine(const Vector2f& b) const;

  void draw();
};
