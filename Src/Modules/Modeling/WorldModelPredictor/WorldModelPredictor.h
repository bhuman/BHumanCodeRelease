/**
 * @file WorldModelPredictor.h
 *
 * This file declares a class that predicts the current state of data computed by modeling modules in the previous frame.
 * All positions receive odometry updates. If the ball was rolling, a dynamic update is performed, too.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Tools/Module/Module.h"

MODULE(WorldModelPredictor,
{,
  USES(BallModel),
  USES(RobotPose),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(Odometer),
  REQUIRES(OdometryData),
  REQUIRES(OwnTeamInfo),
  PROVIDES(WorldModelPrediction),
});

class WorldModelPredictor : public WorldModelPredictorBase
{
  unsigned timeOfLastExecution = 0;     /**< The point of time this module was executed the last time, i.e. time of last frame */
  unsigned timeWhenLastKicked = 0;      /**< The point of time, when the last kick was detected (by having an increased ball velocity) */
  float ballSpeedAtLastExecution = 0.f; /**< Remember last ball speed to detect kicks */
  BallModel lastUsedBallModel;          /**< The ball model of the point of time when the ball was seen or kicked the last time */
  Pose2f odometryOfLastFrame;           /**< Odometry data when last cognition frame was running */
  Pose2f lastUsedBallModelOdometry;     /**< Odometry data when last ball was seen or kicked */

  /** Performs the computation of the predicted world model
   * @param worldModelPrediction The struct that is updated.
   */
  void update(WorldModelPrediction& worldModelPrediction) override;
};
