/**
 * @file SideConfidenceProvider.h
 *
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/RingBuffer.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/OwnSideModel.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/LocalizationTeamBall.h"

MODULE(SideConfidenceProvider,
{,
  USES(RobotPose),
  USES(LocalizationTeamBall),
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
    (float) standardDeviationBallAngle,    /**< As the name says... */
    (float) standardDeviationBallDistance, /**< As the name says... */
    (float) weightingFactor,               /**< Multiplier for defining the minimum difference between normal and mirrored pose */
    (float) sideConfidenceConfident,       /**< Value for side confidence when being definitely in own half */
    (float) sideConfidenceAlmostConfident, /**< Value for side confidence when being sure about own position but not definitely in own half */
    (float) sideConfidenceConfused,        /**< Value for side confidence when bad things have happened */
    (float) maxBallVelocity,               /**< Maximum velocity of balls that are considered for side confidence computation */
    (int) ballBufferingInterval,           /**< Time for keeping a local ball observation in buffer */
    (float) armContactModificator,         /**< How much the SideConfidence is influsenced by  having arm contact */
    (float) relativeBallDropin,            /**< The value how far the ball is dorpped in behind the robot after kicking out */
    (int) minContinuousArmContact,         /**< The minimum time of continuous arm contact to be considered disturbing (in ms) */
    (float) centerBanZoneRadius,           /**< All balls closer to the field's center than this are ignored */
    (float) minWeighting,                  /**< The minimum weighting an original or mirrored ball needs to have to be considered */
    (int)   minTeammateOverride,           /**< Number of teammates that are required to override my side belief */
    (int)   maxBufferAge,                  /**< Maximum age of a side confidence measurement to remain in the buffer */
  }),
});

class SideConfidenceProvider : public SideConfidenceProviderBase
{
public:
  /** Constructor */
  SideConfidenceProvider();

private:
  /**
    * Provides the sideConfidence
    */
  void update(SideConfidence& sideConfidence);

  bool lost; /** sideConfidence 0% and lost-sound played */
  enum {BUFFER_SIZE = 12};               /**< Number of ball state observations */
  ENUM(BallModelSideConfidence,
  {,
    OK,
    MIRROR,
    UNKNOWN,
  });                                     /**< Discrete states of confidence resulting from comparison of ball models (own vs. others) */

  struct SideConfidenceMeasurement
  {
    BallModelSideConfidence ballConfidence;
    unsigned timeStamp;
  };

  RingBuffer<SideConfidenceMeasurement, BUFFER_SIZE> confidenceBuffer; /**< Buffer of last confidences */
  BallModelSideConfidence averageBallConfidence; /**< The average side confidence based on buffered confidences */
  unsigned timeOfLastFall;               /**< Timestamp to see if the robot has fallen down */
  unsigned lastTimeWithoutArmContact;
  Vector2f lastBallObservation = Vector2f::Zero();         /**< Position (relative to the robot) of the last estimated ball that has actually been observed */
  unsigned timeOfLastBallObservation;        /**< Point of time of last observation */
  unsigned timeOfLastTeamBallObservation;    /**< Point of time of last observation of others */
  float maxDistanceToFieldCenterForArmConsideration; /**< Just as the name says ... */
  float maxDistanceToFieldCenterForFallDownConsideration; /**< Just as the name says ... */

  /**
    * Checks if the other team mates see the ball near the own estimated position
    * or near its mirrored position.
    */
  void updateSideConfidenceFromOthers(SideConfidence& sideConfidence);

  /**
    * Updates the own confidence.
    */
  void updateSideConfidenceFromOwn(SideConfidence& sideConfidence);

  /**
   * Maps sideConfidence to ConfidenceState.
   */
  void updateConfidenceState(SideConfidence& sideConfidence);

  /** Updates ball confidence buffer */
  void updateBallConfidences(SideConfidence& sideConfidence);

  /** Combines confidence based on current ball models */
  BallModelSideConfidence computeCurrentBallConfidence();

  float computeAngleWeighting(float measuredAngle, const Vector2f& modelPosition,
    const Pose2f& robotPose, float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2f& modelPosition,
    const Pose2f& robotPose, float cameraZ, float standardDeviation) const;
};
