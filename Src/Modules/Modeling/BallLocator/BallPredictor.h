/**
 * @file BallPredictor.h
 * Declares a class that estimates the current position of the ball based on the most actual model.
 * @author Tim Laue
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/BallPrediction.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(BallPredictor,
{,
  USES(BallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(OdometryData),
  PROVIDES(BallPrediction),
  DEFINES_PARAMETERS(
  {,
    (int)(100) validityTimeout, /**< If we have not seen the ball for this amount of time, the prediction unvalid */
  }),
});

class BallPredictor : public BallPredictorBase
{
  unsigned timeOfLastExecution = 0; /**< The point of time this module was executed the last time, i.e. time of last frame */
  BallModel lastSeenBallModel; /**< The ball model of the point of time when the ball was seen the last time */
  Pose2f odometryOfLastFrame;  /**< Odometry data when last cognition frame was running */
  Pose2f lastSeenBallModelOdometry; /**< Odometry data when last ball was seen */

  void update(BallPrediction& ballPrediction);
};
