/**
 * @file BallStateEstimator.h
 *
 * For ball tracking in the RoboCup context, two tasks need to be solved:
 *   1. Filtering and clustering the detected balls (i.e. the BallPercepts) to avoid
 *      the use of any false positive, which might lead to severe problems.
 *   2. Estimating a precise position as well as a velocity based on a set of recent
 *      ball observations.
 * In previous implementations, these two tasks have been combined within one module. For more
 * clarity and more flexibility, both tasks are split into two modules now.
 * In addition, collision detection between feet and ball has been moved to a third model.
 *
 * This module provides a solution for task 2: State estimation.
 *
 * The module declared in this file is based on the implementation that has been used by
 * B-Human for multiple years: estimating the ball position and velocity by maintaining
 * a set of normal Kalman filters.
 *
 * @author Tim Laue
 */

#pragma once

#include "KalmanFilterBallHypothesis.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/BallContactWithRobot.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FilteredBallPercepts.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(BallStateEstimator,
{,
  REQUIRES(BallContactWithRobot),
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FilteredBallPercepts),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(OdometryData),
  REQUIRES(RobotInfo),
  REQUIRES(WorldModelPrediction),
  PROVIDES(BallModel),
  DEFINES_PARAMETERS(
  {,
    (Vector4f)(0.1f, 0.1f, 1.f, 1.f) processDeviation,   /**< The process noise. (petite) */
    (Vector2f)(0.02f, 0.08f) robotRotationDeviation,     /**< Deviation of the rotation of the robot's torso */
    (Pose2f)(0.5f, 0.5f, 0.5f) odometryDeviation,        /**< The percentage inaccuracy of the odometry */
    (float)(0.1f) initialStateWeight,                    /**< The weight of newly created states (between >0 and <1) */
    (float)(1000.f) ballDisappearedMaxCheckingDistance,  /**< Balls can only "disappear" within this distance */
    (int)(250) ballDisappearedTimeout,                   /**< Threshold. For this time span, a ball can not disappear. This avoid immediate disappearance after a few false negatives */
    (int)(700) lastBallPerceptTimeout,                   /**< Threshold. Consider a previously seen ball as valid for velocity computation for this amount of milliseconds. */
    (int)(4) minNumberOfMeasurementsForRollingBalls,     /**< A internal rolling ball hypothesis can only be selected, if it incorporates at least this number of measurements. */
  }),
});

/**
 * @class BallStateEstimator
 *
 * Estimation of ball position and velocity based on a set of Kalman filters
 */
class BallStateEstimator : public BallStateEstimatorBase
{
public:
  /** Constructor */
  BallStateEstimator();

private:
  OdometryData lastOdometryData;                   /**< The odometry state at the last execution of this module */
  unsigned int lastFrameTime;                      /**< The point of time at the last execution of this module */
  KalmanFilterBallHypothesis* bestState;           /**< Pointer to the moving hypothesis that is most likely */
  KalmanFilterBallHypothesis states[12];           /**< The list of hypotheses */
  int numberOfStates;                              /**< The number of internall ball states that are tracked in parallel */
  RingBufferWithSum<unsigned short, 60> seenStats; /**< Contains a 100 for time the ball was seen and 0 when it was not, used for statistics in ball model */
  bool ballWasSeenInThisFrame;                     /**< Internal flag to keep some expressions short */
  unsigned timeWhenBallFirstDisappeared;           /**< A point of time from which on a ball seems to have disappeared (is not seen anymore although it should be) */
  bool ballDisappeared;                            /**< If true, the ball is currently considered as disappeared */
  FilteredBallPercept lastBallPercept;             /**< The last seen ball. Used for velocity computation */

  /** Initialize member variables and reset filters */
  void init();

  /** Reset all filters by deleting them */
  void reset();

  /** Set all fields in ball model
   *  @param ballModel The ball model!
   */
  void generateModel(BallModel& ballModel);

  void motionUpdate(BallModel& ballModel);
  void integrateCollisionWithFeet();
  void sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov, float ballRadius);
  void normalizeWeights(KalmanFilterBallHypothesis*& worstStationaryState, KalmanFilterBallHypothesis*& worstMovingState);
  void createNewStates(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov, KalmanFilterBallHypothesis* worstStationaryState, KalmanFilterBallHypothesis* worstMovingState);
  void plotAndDraw();

  /**
   * Computes the ball model representation
   * @param ballModel The ball model, exactly!
   */
  void update(BallModel& ballModel) override;
};
