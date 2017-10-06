/**
 * @file WorldModelPredictor.h
 *
 * This file declares a class that predicts the current state of data computed by modeling modules in the previous frame .
 * All positions have receive odometry updates. If the ball was rolling, a dynamic update is been performed, too.
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/OdometryData.h"

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
  PROVIDES(WorldModelPrediction),
  DEFINES_PARAMETERS(
  {,
    (int)(100) ballValidityTimeout,     /**< If we have not seen the ball for this amount of time, the prediction is unvalid */
  }),
});

class WorldModelPredictor : public WorldModelPredictorBase
{
  unsigned timeOfLastExecution = 0;     /**< The point of time this module was executed the last time, i.e. time of last frame */
  BallModel lastSeenBallModel;          /**< The ball model of the point of time when the ball was seen the last time */
  Pose2f odometryOfLastFrame;           /**< Odometry data when last cognition frame was running */
  Pose2f lastSeenBallModelOdometry;     /**< Odometry data when last ball was seen */

  /** Performs the computation of the predicted world model
   * @param worldModelPrediction The struct that is updated.
   */
  void update(WorldModelPrediction& worldModelPrediction);
};
