/**
 * @file ObstacleGridProvider.h
 *
 * @author Nele Matschull
 */

#pragma once

#include "Representations/Modeling/ObstacleGrid.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"

MODULE(ObstacleGridProvider,
{,
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  PROVIDES(ObstacleGrid),
});

class ObstacleGridProvider : public ObstacleGridProviderBase
{
  Pose2f accumulatedOdometry;      /**< Storing odometry differences for grid update */
  Pose2f lastOdometry;             /**< Odometry value at last execution */
  Cell* cells;                     /**< Pointer to cells in grid (for shorter notation)*/
  int cellFreeInterval = 3000;
  bool initialized = false;        /**< flag for initialization */
  unsigned lastTimePenalized = 0;    /**< Last point of time the robot was penalized */
  int cellSize = 60;               /**< Length of grid cell edge */
  int cellCount = 45;              /**< Number of cells in one dimension (cellCount x cellCount Grid), must be changed in ObstacleGrid.h too */
  //float goalPostRadius = 55.f;         /**< Radius of goalPost */
  float squaredGoalPostRadius = 55.f * 55.f;
  //float squaredRobotRadius = 80.f * 80.f;

  void update(ObstacleGrid& obstacleGrid) override;
  void moveGrid();
  void ageCellState();
  int worldToGrid(const Vector2f& inWorld) const;
};
