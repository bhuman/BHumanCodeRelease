/**
 * @file SelfLocator.h
 *
 * Declares a class that performs self-localization
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "UKFRobotPoseHypothesis.h"
#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/AlternativeRobotPoseHypothesis.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/PerceptRegistration.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/SelfLocalizationHypotheses.h"
#include "Representations/Modeling/SideInformation.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/IMUValueState.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Configuration/StaticInitialPose.h"
#include "Tools/Modeling/SampleSet.h"
#include "Framework/Module.h"

MODULE(SelfLocator,
{,
  REQUIRES(AlternativeRobotPoseHypothesis),
  REQUIRES(ExtendedGameState),
  REQUIRES(Odometer),
  REQUIRES(OdometryData),
  REQUIRES(FallDownState),
  REQUIRES(GameState),
  REQUIRES(IMUValueState),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(LibDemo),
  REQUIRES(MotionInfo),
  REQUIRES(CameraInfo),
  REQUIRES(RobotPose),
  REQUIRES(SideInformation),
  REQUIRES(PerceptRegistration),
  REQUIRES(WorldModelPrediction),
  REQUIRES(SetupPoses),
  REQUIRES(StaticInitialPose),
  REQUIRES(GroundTruthRobotPose),
  PROVIDES(RobotPose),
  PROVIDES(SelfLocalizationHypotheses),
  LOADS_PARAMETERS(
  {,
    (int)      numberOfSamples,                      /**< The number of samples used by the self-locator */
    (Pose2f)   defaultPoseDeviation,                 /**< Standard deviation used for creating new hypotheses */
    (Pose2f)   walkInPoseDeviation,                  /**< Standard deviation used for creating new hypotheses at walk in positions */
    (Pose2f)   returnFromPenaltyPoseDeviation,       /**< Standard deviation used for creating new hypotheses when returning from a penalty */
    (Pose2f)   manualPlacementPoseDeviation,         /**< Standard deviation used for creating new hypotheses when being manually placed */
    (Pose2f)   penaltyShootoutPoseDeviation,         /**< Standard deviation used for creating new hypotheses in a penalty shootout */
    (Pose2f)   filterProcessDeviation,               /**< The process noise for estimating the robot pose. */
    (Pose2f)   odometryDeviation,                    /**< The percentage inaccuracy of the odometry. */
    (Vector2f) odometryRotationDeviation,            /**< A rotation deviation of each walked mm. */
    (float)  movedDistWeightRotationNoise,           /**< Weighting for considering walked distance when computing a sample's rotational error */
    (float)  movedAngleWeightRotationNoise,          /**< Weighting for considering rotation when computing a sample's rotational error */
    (float)  movedAngleWeightRotationNoiseNotWalking,/**< Weighting for considering rotation when computing a sample's rotational error, special handling for situations in which the robot is not walking and odometry might not be continuous */
    (float)  majorDirTransWeight,                    /**< Weighting for considering the major direction when computing a sample's translational error (major means, for instance, the movement in x direction when computing the translational error in x direction) */
    (float)  minorDirTransWeight,                    /**< Weighting for considering the minor direction when computing a sample's translational error (minor means, for instance, the movement in y direction when computing the translational error in x direction) */
    (float)  minValidityForSuperbLocalizationQuality,            /**< Only if(validityOfBestHypothesis >= minValidityForSuperbLocalizationQuality) [+other criteria] theRobotPose.quality can be superb. */
    (float)  maxTranslationDeviationForSuperbLocalizationQuality,/**< Only if(translational deviation <= maxTranslationDeviationForSuperbLocalizationQuality) [+other criteria] theRobotPose.quality can be superb, unit is mm. */
    (Angle)  maxRotationalDeviationForSuperbLocalizationQuality, /**< Only if(rotational deviation <= maxRotationalDeviationForSuperbLocalizationQuality) [+other criteria] theRobotPose.quality can be superb. */
    (float)  baseValidityWeighting,                  /**< Each particle has at least this weight. */
    (int)    numberOfConsideredFramesForValidity,    /**< Robot pose validity is filtered over this number of frames. */
    (bool)   considerLinesForValidityComputation,    /**< Matched / unmatched lines can be considered when computing the pose validity, but they don't have to. */
    (float)  validityFactorPoseMeasurement,          /**< Indicates how strongly a perceived pose influences a sample's validity. Higher value means stronger influence */
    (float)  validityFactorLandmarkMeasurement,      /**< Indicates how strongly a perceived landmark influences a sample's validity. Higher value means stronger influence */
    (float)  validityFactorLineMeasurement,          /**< Indicates how strongly a perceived line influences a sample's validity. Higher value means stronger influence */
    (float)  positionJumpNotificationDistance,       /**< Threshold, min position change (distance) to give a notification. */
    (int)    minNumberOfObservationsForResetting,    /**< To accept an alternative robot pose, it must be based on at least this many observations of field features. */
    (float)  translationalDeviationForResetting,     /**< To insert a new particle, the current alternative pose must be farther away from the current robot pose than this threshold. */
    (float)  rotationalDeviationForResetting,        /**< To insert a new particle, the current alternative pose rotation must be more different from the current robot pose rotation than this threshold. */
    (float)  returnFromPenaltyMaxXOffset,            /**< When poses are generated after returning from a penalty, a random x offset is added to each pose. The absolute value of this offset is defined by this parameter. */
    (bool)   demoUseCustomReturnFromPenaltyPoses,             /**< Flag to use the two following poses when localization is restarted after a penalty. This is only useful for certain demos on special fields. */
    (Pose2f) demoCustomReturnFromPenaltyPoseGoalie,           /**< Goalie pose is set to this pose after a penalty. */
    (Pose2f) demoCustomReturnFromPenaltyPoseFieldPlayer,      /**< Field player pose is set to this pose after a penalty. */
    (int)    timeoutForResamplingAfterReturnFromPenalty,      /**< Do not perform any resampling step after returning from a penalty for this time (in milliseconds).*/
    (int)    timeoutForSensorResettingAfterReturnFromPenalty, /**< Do not perform sensor resetting after returning from a penalty for this time (in milliseconds).*/
    (int)    timeoutBetweenTwoJumpSounds,                     /**< Make a pause between playing two "Jump!" sounds for this time (in milliseconds).*/
    (int)    timeoutForJumpAnnotationAfterReturnFromPenalty,  /**< If the robot pose jumps within this time (in milliseconds) after returning from a penalty, a different annotation will be written.*/
  }),
});

/**
 * @class SelfLocator
 *
 * A module for self-localization, based on a particle filter with each particle having
 * an Unscented Kalman Filter.
 */
class SelfLocator : public SelfLocatorBase
{
private:
  SampleSet<UKFRobotPoseHypothesis>* samples;   /**< Container for all samples. */
  unsigned lastTimeJumpSound;                   /**< When has the last sound been played? Avoid to flood the sound player in some situations */
  unsigned timeOfLastReturnFromPenalty;         /**< Point of time when the last penalty of this robot was over */
  bool sampleSetHasBeenReset;                   /**< Flag indicating that all samples have been replaced in the current frame */
  int nextSampleNumber;                         /**< Unique sample identifiers */
  int idOfLastBestSample;                       /**< Identifier of the best sample of the last frame */
  float averageWeighting;                       /**< The average of the weightings of all samples in the sample set */
  unsigned lastAlternativePoseTimestamp;        /**< Last time an alternative pose was valid */
  bool validitiesHaveBeenUpdated;               /**< Flag that indicates that the validities of the samples have been changed this frame */
  Pose2f lastGroundTruthRobotPose;              /**< Remember ground truth of last frame */

  int sumOfPerceivedLandmarks;                  /**< Statistics: Sum up number of all perceived landmarks */
  int sumOfPerceivedLines;                      /**< Statistics: Sum up number of all perceived lines */
  float sumOfUsedLandmarks;                     /**< Statistics: Sum up number of all integrated landmarks (average over samples) */
  float sumOfUsedLines;                         /**< Statistics: Sum up number of all integrated lines (average over samples) */

  /**
   * The method provides the robot pose
   *
   * @param robotPose The robot pose representation that is updated by this module.
   */
  void update(RobotPose& robotPose) override;

  /**
   * The method provides the list of currently evaluated hypotheses.
   *
   * @param selfLocalizationHypotheses The representation that is updated by this module.
   */
  void update(SelfLocalizationHypotheses& selfLocalizationHypotheses) override;

  /** Integrate odometry offset into hypotheses */
  void motionUpdate();

  /** Perform UKF measurement step for all samples */
  void sensorUpdate();

  /** Particle filter resampling step */
  void resampling();

  /** Tries to replace a sample, if current robot pose is much different from the alternative hypothesis
   * @param robotPose The current robot pose estimate
   * @return true, if a sample has been replaced
   */
  bool sensorResetting(const RobotPose& robotPose);

  /** Special function for testing (currently only working in simulation).
   *  Whenever the ground truth robot pose has changed to a certain extent, the samples are
   *  reinitializes at the new pose. This allows easier testing by moving the robot around without waiting
   *  for it to re-localize at the new pose.
   */
  void resetSamplesToGroundTruth();

  /** Some motions lead to low-quality percepts, as the camera pose cannot be computed precisely enough.
   *  This functions performs the necessary checks.
   * @return true, if the current motion is not safe.
   */
  bool currentMotionIsUnsafe();

  /** Checks, if the current AlternativeRobotPoseHypothesis is similar (given the module parameters)
   *  to a given robot pose
   * @param robotPose The pose to become compared with the AlternativeRobotPoseHypothesis
   * @return true, if both poses are similar
   */
  bool alternativeRobotPoseIsCompatibleToPose(const Pose2f& robotPose);

  /** Special handling for initializing samples at some game state transitions and in penalty shootout
   */
  void handleGameStateChanges();

  /** Computes (almost) all elements needed to fill RobotPose
   * @param robotPose The pose (updated by this method)
   */
  void computeModel(RobotPose& robotPose);

  /** Called by computeModel to determine how "good"
   *  the final result of the pose estimation appears to be.
   * @param validityOfBestHypothesis The validity of the best robot pose hypothesis
   * @param robotPose The pose (updated by this method)
   */
  void setLocalizationQuality(RobotPose& robotPose, float validityOfBestHypothesis);

  /** Returns a reference to the sample that has the highest validity
   * @return A reference sample
   */
  UKFRobotPoseHypothesis& getMostValidSample();

  /** Check to avoid samples with the same ID
   * @return Always true ;-)
   */
  bool allSamplesIDsAreUnique();

  /** Checks, if all samples clustered in one place or if there exist multiple clusters
   * @param robotPose The currently assumed robot pose, i.e. the pose represented by the most likely sample
   * @return true, if all samples appear to form one cluster.
   */
  bool sampleSetIsUnimodal(const RobotPose& robotPose);

  /** Returns a pose at one of the two possible positions when returning from a penalty
   * @param leftSideOfGoal Set to true, if the position left of the own goal should be returned. Set to false for the other side.
   * @return A robot pose
   */
  Pose2f getNewPoseReturnFromPenaltyPosition(bool leftSideOfGoal);

  /** Returns the pre-configured pose for a robot to enter the field
   * @return A robot pose
   */
  Pose2f getNewPoseAtWalkInPosition();

  /** Returns a pose on the manual placement positions for the 1v1 demo
   * @return A robot pose
   */
  Pose2f getNewPoseAtManualPlacementPosition();

  /** Returns a pose for a penalty shootout
   * @return A robot pose
   */
  Pose2f getNewPoseAtPenaltyShootoutPosition();

  /** Computes a new pose based on the AlternativeRobotPoseHypothesis
   * @param forceOwnHalf The new pose must be inside the own half
   * @param lastRobotPose The previous robot pose
   * @return A new pose
   */
  Pose2f getNewPoseBasedOnObservations(bool forceOwnHalf, const Pose2f& lastRobotPose) const;

  /** Checks, if a new pose is closer to the last pose or its mirrored version
   * @param true, if the mirrored version is a better fit
   */
  bool isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const;

  /** Draw debug information
   * @param robotPose The current robot pose
   */
  void draw(const RobotPose& robotPose);

public:
  /** Default constructor */
  SelfLocator();

  /** Destructor */
  ~SelfLocator();
};
