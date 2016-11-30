/**
 * @file ExpBallLocator.h
 * Declares a class that estimates the position of the ball using a couple of Kalman filters.
 * @author Colin Graf
 */

#pragma once

#include "BallHypothesis.h"
#include "Representations/Communication/TeammateData.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallPrediction.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"

MODULE(ExpBallLocator,
{,
  REQUIRES(BallPercept),
  REQUIRES(BallPrediction),
  REQUIRES(OdometryData),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(TorsoMatrix),
  REQUIRES(RobotModel),
  REQUIRES(RobotDimensions),
  REQUIRES(Odometer),
  REQUIRES(TeammateData),
  USES(RobotPose),
  USES(BallModel),
  PROVIDES(BallModel),
  DEFINES_PARAMETERS(
  {,
    (Vector4f)(0.1f, 0.1f, 1.f, 1.f) processDeviation, /**< The process noise. (petite) */
    (Vector2f)(0.02f, 0.08f) robotRotationDeviation, /**< Deviation of the rotation of the robot's torso */
    (Pose2f)(0.5f, 0.5f, 0.5f) odometryDeviation, /**< The percentage inaccuracy of the odometry */
    (float)(0.1f) initialStateWeight, /**< The weight of newly created states (between >0 and <1) */
    (Vector3f)(65.f, 0.f, 0.f) leftFootOffset, /**< Offset from left foot sole point to a "center" of the foot used for approximating the foot shape with a circle */
    (float)(50.f) footRadius, /**< The radius of the approximated foot shape */
    (float)(0.1f) footMass, /**< An assumed mass for each foot (in kg) */
    (float)(0.05f) ballMass, /**< The mass of the ball (in kg) */
    (Vector2f)(1.f, 1.f) kickDeviation, /**< The percentage inaccuracy of passed velocities */
    (int)(500) lowerCameraImageBallTimeout, /** If we have seen a ball in the lower image, ignore balls in upper image for this time */
    (int)(6000) fieldFeatureTimeout, /** When using self-localization information, there should have been a field feature recently */
    (float)(500.f) fieldBorderExclusionDistance, /** Stuff that is more far away from the field (not the carpet!) is excluded */
    (int)(250) ballDisappearedTimeout,
    (float)(1000.f) ballDisappearedMaxCheckingDistance,
    (int)(600) stagingTimeout,
    (unsigned)(2) stagingCountNear,
    (unsigned)(1) stagingCountFar,
    (float)(400.f) stagingDistanceNear,
    (float)(800.f) stagingDistanceFar,
    (float)(1000.f) stagingFarBegin,
    (float)(400.f) robotBanRadius,
    (int)(3) ballBorderAdditionalPixelThreshold,
    (float)(420.f) guessedRollingBallAcceptanceDistance,
  }),
});

/**
 * @class BallLocator
 * A BallLocator using some Kalman filters.
 */
class ExpBallLocator : public ExpBallLocatorBase
{
public:
  ExpBallLocator();

private:
  struct StagedBall
  {
    StagedBall(const Vector2f& p, unsigned t):position(p), timestamp(t) {}
    Vector2f position;
    unsigned timestamp;
  };

  RingBuffer<StagedBall,5> stagedBalls;

  float deltaTime; /**< Time difference in seconds to previous image */

  BallHypothesis states[12];
  unsigned int stateCount;
  BallHypothesis* bestState = nullptr;

  bool hasLastBallPercept;
  unsigned int lastBallPerceptTimeStamp;
  Vector2f lastBallPercept;
  Matrix2f lastBallPerceptCov;

  OdometryData lastOdometryData;
  unsigned int lastFrameTime;

  Vector2f lastLeftFootCenter;
  Vector2f lastRightFootCenter;
  Vector2f lastBallPosition;

  int timeBallWasBeenSeenInLowerCameraImage;
  bool ballWasSeenInThisFrame;
  RingBufferWithSum<unsigned short, 60> seenStats; /**< Contains a 100 for time the ball was seen and 0 when it was not. */
  
  unsigned timeWhenBallFirstDisappeared;
  bool ballDisappeared;

  /**
   * Initialize something.
   */
  void init();

  /**
   * Deletes all filters
   */
  void reset();

  /**
   * Provides ball model representation
   */
  void update(BallModel& ballModel);

  bool ballCanBeExcludedByLocalization();

  bool ballIsAtBorderAndCloseToTeammate();

  void motionUpdate(BallModel& ballModel);
  void handleCollisionWithFeet(const Vector2f& leftFootCenter, const Vector2f& rightFootCenter);
  void sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov);
  void normalizeWeights(BallHypothesis*& bestState, BallHypothesis*& worstStationaryState, BallHypothesis*& worstMovingState);
  void createNewStates(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov, BallHypothesis* worstStationaryState, BallHypothesis* worstMovingState);
  void generateModel(BallModel& ballModel);
};
