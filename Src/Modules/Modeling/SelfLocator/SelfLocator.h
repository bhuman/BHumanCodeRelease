/**
* @file SelfLocator.h
*
* Declares a class that performs self-localization
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "SelfLocatorBase.h"
#include "FieldModel.h"
#include "UKFSample.h"
#include "TemplateGenerator.h"
#include "Tools/SampleSet.h"

/**
* @class SelfLocator
*
* A new module for self-localization
*/
class SelfLocator : public SelfLocatorBase
{
private:
  SampleSet<UKFSample>* samples;       /**< Container for all samples. */
  FieldModel fieldModel;               /**< Information about the robot's environment */
  TemplateGenerator sampleGenerator;   /**< Several parameters */
  int lastPenalty;                     /**< Was the robot penalised in the last frame? */
  int lastGameState;                   /**< The game state in the last frame. */
  Pose2D lastRobotPose;                /**< The result of the last computation */
  unsigned lastTimeFarGoalSeen;        /**< Timestamp for checking goalie localization */
  unsigned lastTimeKeeperJumped;       /**< Timestamp for helping goalie localization */
  bool sampleSetHasBeenResetted;       /**< Flag indicating that all samples have been replaced in the current frame */
  int nextSampleNumber;                /**< Unique sample identifiers */
  int numberOfLastBestSample;          /**< Identifier of the best sample of the last frame */

  /**
  * The method provides the robot pose (equal to the filtered robot pose).
  *
  * @param robotPose The robot pose representation that is updated by this module.
  */
  void update(RobotPose& robotPose);

  /** Integrate odometry offset into hypotheses */
  void motionUpdate();

  void sensorUpdate();

  void resampling();

  void sensorResetting(const RobotPose& robotPose);

  void handleGameStateChanges(const Pose2D& propagatedRobotPose);

  void handleSideConfidence();

  void computeModel(RobotPose& robotPose);

  void domainSpecificSituationHandling();

  UKFSample& getMostValidSample();

  void computeSampleValidities();

  /** draw debug information */
  void draw(const RobotPose& robotPose);

public:
  /** Default constructor. */
  SelfLocator();

  /** Destructor. */
  ~SelfLocator();
};
