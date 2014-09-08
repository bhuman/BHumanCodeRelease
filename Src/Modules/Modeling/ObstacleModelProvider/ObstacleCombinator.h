/**
* @file ObstacleCombinator.h
*
* This file declares a module that merges information from the ultrasonic obstacle grid
* and perceptions from vision.
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootContactModel.h"
#include "Tools/Module/Module.h"

MODULE(ObstacleCombinator,
{,
  REQUIRES(FrameInfo),
  REQUIRES(USObstacleModel),
  REQUIRES(ArmContactModel),
  REQUIRES(FootContactModel),
  PROVIDES_WITH_MODIFY_AND_DRAW(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (int) maxRobotDistance,                /**< Maximum distance for considering robots as obstacles */
    (bool) considerStandingRobots,         /**< Use standing robots for obstacle modeling or not */
    (bool) considerLyingRobots,            /**< Use lying robots for obstacle modeling or not */
    (bool) considerArmCollisions,          /**< Use robots detected by arm collision for obstacle modeling or not */
  }),
});

/**
* @class ObstacleCombinator
*
* A module for computing the occupied space in the robot's environment
*/
class ObstacleCombinator: public ObstacleCombinatorBase
{
private:
  /** Executes this module
  * @param obstacleModel The data structure that is filled by this module
  */
  void update(ObstacleModel& obstacleModel);

  void addFootObstacles(ObstacleModel& obstacleModel);

  void addArmObstacles(ObstacleModel& obstacleModel);

  void addArmObstacle(ObstacleModel& obstacleModel, bool leftArm, const ArmContactModel::PushDirection& pushDirection);
};
