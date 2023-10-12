/**
 * @file Modules/Infrastructure/OracledWorldModelProvider.h
 *
 * This file implements a module that provides models based on simulated data.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeammatesBallModel.h"

MODULE(OracledWorldModelProvider,
{,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(FrameInfo),
  PROVIDES(BallModel),
  PROVIDES(GroundTruthBallModel),
  PROVIDES(GlobalOpponentsModel),
  PROVIDES(ObstacleModel),
  PROVIDES(RobotPose),
  PROVIDES(GroundTruthRobotPose),
  PROVIDES(TeammatesBallModel),
  LOADS_PARAMETERS(
  {,
    (Pose2f) robotPoseOffset, /**< Offset that will be added to the robot pose. Useful for testing */
    (float) obstacleModelMaxDistance, /**< Only obstacles (players, goalposts) will be entered in the obstacle model if their distance to the robot is closer that this parameter value */
  }),
});

/**
 * @class OracledWorldModelProvider
 * A module that provides several models
 */
class OracledWorldModelProvider: public OracledWorldModelProviderBase
{
public:
  /** Constructor*/
  OracledWorldModelProvider();

private:
  /** The function that actually computes the ball model*/
  void computeBallModel();

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
   * @param teammatesBallModel The data struct to be filled
   */
  void update(TeammatesBallModel& teammatesBallModel) override;

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

  unsigned int lastBallModelComputation;  /**< Time of last ball model computation*/
  unsigned int lastRobotPoseComputation;  /**< Time of last robot pose computation*/
  BallModel    theBallModel;              /**< The current ball model*/
  RobotPose    theRobotPose;              /**< The current robot pose*/
};
