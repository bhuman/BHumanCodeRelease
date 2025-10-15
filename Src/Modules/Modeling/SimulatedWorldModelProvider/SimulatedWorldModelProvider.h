/**
 * @file Modules/Infrastructure/SimulatedWorldModelProvider.h
 *
 * This file implements a module that provides models based on simulated data.
 *
 * @author Tim Laue
 */


/*
 * Models to consider in the future:
 * - GlobalTeammatesModel
 * - Something that I forgot?
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

MODULE(SimulatedWorldModelProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FrameInfo),
  REQUIRES(GroundTruthWorldState),
  REQUIRES(Odometer),
  REQUIRES(RobotDimensions),
  PROVIDES(BallModel),
  PROVIDES(GroundTruthBallModel),
  PROVIDES(GlobalOpponentsModel),
  PROVIDES(ObstacleModel),
  PROVIDES(RobotPose),
  PROVIDES(GroundTruthRobotPose),
  LOADS_PARAMETERS(
  {,
    (float) obstacleModelMaxPerceptionDistance, /**< Only obstacles (players, goalposts) will be entered in the obstacle model if their distance to the robot is closer that this parameter value */
    (float) ballModelMaxPerceptionDistance,     /**< BallModel only becomes updated, if the ball is closer than this distance */
    (bool)  perfectBallModel,                   /**< Ignore distance, perspective and occlusions and always update the BallModel */
    (unsigned) ballDisappearedThreshold,        /**< Threshold for the amount of "false negatives" before the ball is considered disappeared */
  }),
});

/**
 * @class SimulatedWorldModelProvider
 * A module that provides several models
 */
class SimulatedWorldModelProvider: public SimulatedWorldModelProviderBase
{
public:
  /** Constructor*/
  SimulatedWorldModelProvider();

private:
  /** The function that actually computes the ball model*/
  void updateInternalBallModel();

  /** Check, if the robot could see the ball
   *  @param ballPosition The current position of the ball in robot-relative field coordinates
   *  @return true, if this is the case; false otherwise
   */
  bool ballIsVisible(const Vector2f& ballPosition);

  /** The function that actually computes the robot pose*/
  void computeRobotPose();

  /** One main function, might be called every cycle
   * @param ballModel The data struct to be filled
   */
  void update(BallModel& ballModel) override;

  /** One main function, might be called every cycle
   * @param groundTruthBallModel The data struct to be filled
   */
  void update(GroundTruthBallModel& groundTruthBallModel) override;

  /** One main function, might be called every cycle
   * @param globalOpponentsModel The data struct to be filled
   */
  void update(GlobalOpponentsModel& globalOpponentsModel) override;

  /** One main function, might be called every cycle
   * @param obstacleModel The data struct to be filled
   */
  void update(ObstacleModel& obstacleModel) override;

  /** One main function, might be called every cycle
   * @param robotPose The data struct to be filled
   */
  void update(RobotPose& robotPose) override;

  /** One main function, might be called every cycle
   * @param groundTruthRobotPose The data struct to be filled
   */
  void update(GroundTruthRobotPose& groundTruthRobotPose) override;

  unsigned int lastBallModelComputation;             /**< Time of last ball model computation*/
  unsigned int lastBallVelocityUpdate;               /**< Time of last ball model propagation (ball was not seen but seems to be rolling)*/
  unsigned int lastGroundTruthBallModelComputation;  /**< Time of last ground truth ball model computation*/
  unsigned int lastRobotPoseComputation;             /**< Time of last robot pose computation*/
  unsigned timeWhenBallFirstDisappeared;             /**< A point of time from which on a ball seems to have disappeared (is not seen anymore although it should be) */
  unsigned ballNotSeenButShouldBeSeenCounter;        /**< How often the ball has not been seen (although it should have been) since the last percept. */
  BallModel    theBallModel;                         /**< Internal representation for the current ball model*/
  RobotPose    theRobotPose;                         /**< Internal representation for the current robot pose*/
};
