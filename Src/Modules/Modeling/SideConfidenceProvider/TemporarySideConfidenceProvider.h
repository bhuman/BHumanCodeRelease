/**
 * @file TemporarySideConfidenceProvider.h
 * Calculates the SideConfidence
 *
 * @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
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
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Modeling/SideConfidence.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/CombinedWorldModel.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"


MODULE(TemporarySideConfidenceProvider)
  USES(RobotPose)
  USES(CombinedWorldModel)
  USES(BehaviorControlOutput)
  REQUIRES(OwnSideModel)
  REQUIRES(Odometer)
  REQUIRES(TeamMateData)
  REQUIRES(BallModel)
  REQUIRES(FieldDimensions)
  REQUIRES(CameraMatrix)
  REQUIRES(FallDownState)
  REQUIRES(ArmContactModel)
  REQUIRES(FrameInfo)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  PROVIDES_WITH_MODIFY_AND_DRAW(SideConfidence)
  LOADS_PARAMETER(float, standardDeviationBallAngle)    /**< As the name says... */
  LOADS_PARAMETER(float, standardDeviationBallDistance) /**< As the name says... */
  LOADS_PARAMETER(float, weightingFactor)               /**< Multiplier for defining the minimum difference between normal and mirrored pose */
  LOADS_PARAMETER(float, sideConfidenceConfident)       /**< Value for side confidence when being definitely in own half */
  LOADS_PARAMETER(float, sideConfidenceAlmostConfident) /**< Value for side confidence when being sure about own position but not definitely in own half */
  LOADS_PARAMETER(float, sideConfidenceConfused)        /**< Value for side confidence when bad things have happened */
  LOADS_PARAMETER(float, maxBallVelocity)               /**< Maximum velocity of balls that are considered for side confidence computation */
  LOADS_PARAMETER(int, timeInPenaltyArea)               /**< Time till the robot should be removed from own penalty area */
  LOADS_PARAMETER(int, ballBufferingInterval)           /**< Time for keeping a local ball observation in buffer */
  LOADS_PARAMETER(float, lowLocalizationValidityModificator) /**< How much the SideConfidence is influenced by falling  down */
  LOADS_PARAMETER(float, armContactModificator)         /**< How much the SideConfidence is influsenced by  having arm contact */
  LOADS_PARAMETER(float, relativeBallDropin)            /**< The value how far the ball is dorpped in behind the robot after kicking out */
  LOADS_PARAMETER(int, walkdistanceTillDrop)            /**< The distance the robot must walk till the sideconfidence will drop */
  LOADS_PARAMETER(int, minContinuousArmContact)         /**< The minimum time of continuous arm contact to be considered disturbing (in ms) */
  LOADS_PARAMETER(float, centerBanZoneRadius);          /**< A balls closer to the field's center than this are ignored */
  LOADS_PARAMETER(float, minWeighting);                 /**< The minimum weighting a an original or mirrored ball needs to have to be considered */
END_MODULE


class TemporarySideConfidenceProvider : public TemporarySideConfidenceProviderBase
{
public:
  /** Constructor */
  TemporarySideConfidenceProvider();

private:
  /**
    * Provides the sideConfidence
    */
  void update(SideConfidence& sideConfidence);

  bool lost; /** sideConfidence 0% and lost-sound played */
  enum {BUFFER_SIZE = 40};               /**< Number of ball state observations */
  ENUM(BallModelSideConfidence,
    OK,
    MIRROR,
    UNKNOWN
  );                                     /**< Discrete states of confidence resulting from comparison of ball models (own vs. others) */
  RingBuffer<BallModelSideConfidence, BUFFER_SIZE> ballConfidences; /**< Buffer of last confidences */
  BallModelSideConfidence averageBallConfidence; /**< The average side confidence based on buffered confidences */
  unsigned lastTimeOutsideOwnPenaltyArea;/**< The last time the robot's position was reported outside the own penalty area (in ms) */
  unsigned timeOfLastFall;               /**< Timestamp to see if the robot has fallen down */
  unsigned lastTimeWithoutArmContact;
  Vector2<> lastBallObservation;         /**< Position (relative to the robot) of the last estimated ball that has actually been observed */
  unsigned timeOfLastBallObservation;    /**< Point of time of last observation */
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

  float computeAngleWeighting(float measuredAngle, const Vector2<>& modelPosition,
    const Pose2D& robotPose, float standardDeviation) const;

  float computeDistanceWeighting(float measuredDistanceAsAngle, const Vector2<>& modelPosition,
    const Pose2D& robotPose, float cameraZ, float standardDeviation) const;
};
