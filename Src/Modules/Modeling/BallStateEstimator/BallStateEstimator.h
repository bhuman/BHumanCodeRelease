/**
 * @file BallStateEstimator.h
 *
 * For ball tracking in the RoboCup context, multiple tasks need to be solved:
 *   1. Filtering and clustering the detected balls (i.e. the BallPercepts) to avoid
 *      the use of any false positive, which might lead to severe problems.
 *   2. Detecting collisions of the ball and the robot.
 *   3. Estimating a precise position as well as a velocity based on a set of recent
 *      ball observations and collision information.
 *
 * This module provides a solution for task 3: State estimation.
 *
 * The module declared in this file is based on the implementation that has been used by
 * B-Human for multiple years: estimating the ball position and velocity by maintaining
 * a set of normal Kalman filters.
 * It is a reimplementation of the module used during RoboCup 2018.
 *
 * @author Tim Laue
 */

#pragma once

#include "BallStateEstimateFilters.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallContactChecker.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FilteredBallPercepts.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(BallStateEstimator,
{,
  REQUIRES(BallContactChecker),
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FilteredBallPercepts),
  REQUIRES(FrameInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(MotionInfo),
  REQUIRES(Odometer),
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
    (float)(80.f) minSpeed,                              /**< Minimum ball speed. Everything below this threshold will become clipped to 0. */
    (unsigned)(10) maxNumberOfHypotheses,                /**< Do not keep track of more hypothesis per mode than this */
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
  unsigned int lastFrameTime;                               /**< The point of time at the last execution of this module */
  BallStateEstimate* bestState;                             /**< Pointer to the moving hypothesis that is most likely */
  bool recomputeBestState;                                  /**< If true, the bestState pointer is set again. Needed, if balls are removed from a list. */
  std::vector<StationaryBallKalmanFilter,
              Eigen::aligned_allocator<StationaryBallKalmanFilter>>
                                           stationaryBalls; /**< The list of hypotheses */
  std::vector<RollingBallKalmanFilter,
              Eigen::aligned_allocator<RollingBallKalmanFilter>>
                                              rollingBalls; /**< The list of hypotheses */
  RingBufferWithSum<unsigned short, 60> seenStats;          /**< Contains a 100 for time the ball was seen and 0 when it was not, used for statistics in ball model */
  bool ballWasSeenInThisFrame;                              /**< Internal flag to keep some expressions short */
  unsigned timeWhenBallFirstDisappeared;                    /**< A point of time from which on a ball seems to have disappeared (is not seen anymore although it should be) */
  bool ballDisappeared;                                     /**< If true, the ball is currently considered as disappeared */
  FilteredBallPercept lastBallPercept;                      /**< The last seen ball. Used for velocity computation */

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
  void normalizeMeasurementLikelihoods();
  void findBestState();
  template <typename T> void pruneBallBuffer(std::vector<T, Eigen::aligned_allocator<T>>& balls);

  void createNewFilters(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov);
  void plotAndDraw();

  /**
   * Computes the ball model representation
   * @param ballModel The ball model, exactly!
   */
  void update(BallModel& ballModel) override;
};
