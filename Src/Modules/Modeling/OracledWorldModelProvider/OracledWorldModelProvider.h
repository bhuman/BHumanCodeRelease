/**
* @file Modules/Infrastructure/OracledWorldModelProvider.h
*
* This file implements a module that provides models based on simulated data.
*
* @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/ExpObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(OracledWorldModelProvider,
{,
  REQUIRES(GroundTruthWorldState),
  REQUIRES(FrameInfo),
  PROVIDES_WITH_MODIFY_AND_DRAW(BallModel),
  PROVIDES_WITH_MODIFY_AND_DRAW(GroundTruthBallModel),
  PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleModel),
  PROVIDES_WITH_MODIFY_AND_DRAW(ExpObstacleModel),
  PROVIDES_WITH_MODIFY_AND_DRAW(RobotPose),
  PROVIDES_WITH_MODIFY_AND_DRAW(GroundTruthRobotPose),
  DEFINES_PARAMETERS(
  {,
    (Pose2D) robotPoseOffset, /**< Offset that will be added to the robot pose. Usefull for testing */
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
  void update(BallModel& ballModel);

  /** One main function, might be called every cycle
  * @param groundTruthBallModel The data struct to be filled
  */
  void update(GroundTruthBallModel& groundTruthBallModel);

  /** One main function, might be called every cycle
  * @param obstacleModel The data struct to be filled
  */
  void update(ObstacleModel& obstacleModel);

  /** One main function, might be called every cycle
  * @param expObstacleModel The data struct to be filled
  */
  void update(ExpObstacleModel& expObstacleModel);

  /** One main function, might be called every cycle
  * @param robotPose The data struct to be filled
  */
  void update(RobotPose& robotPose);

  /** One main function, might be called every cycle
  * @param groundTruthRobotPose The data struct to be filled
  */
  void update(GroundTruthRobotPose& groundTruthRobotPose);

  /** Converts ground truth robot data to an obstacle
  * @param robot A robot
  * @param obstacleModel The model to which the robot will be added
  */
  void robotToObstacle(const GroundTruthWorldState::GroundTruthRobot& robot, ObstacleModel& obstacleModel) const;

  /** Converts ground truth robot data to an obstacle
  * @param robot A robot
  * @param expObstacleModel The model to which the robot will be added
  * @param isRed Wheter a robot is in team red or not
  */
  void robotToObstacle(const GroundTruthWorldState::GroundTruthRobot& robot, ExpObstacleModel& expObstacleModel, const bool isRed) const;

  unsigned int lastBallModelComputation;  /*< Time of last ball model computation*/
  unsigned int lastRobotPoseComputation;  /*< Time of last robot pose computation*/
  Vector2<>    lastBallPosition;          /*< The ball position after the last computation*/
  BallModel    theBallModel;              /*< The current ball model*/
  RobotPose    theRobotPose;              /*< The current robot pose*/
};
