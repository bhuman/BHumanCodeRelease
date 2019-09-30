/**
 * @file BallStateEstimator.cpp
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

#include "BallStateEstimator.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"
#include <algorithm>

MAKE_MODULE(BallStateEstimator, modeling)

bool compareHypothesesWeightings(BallStateEstimate& a, BallStateEstimate& b)
{
  return  a.weighting > b.weighting;
}

BallStateEstimator::BallStateEstimator(): timeWhenBallFirstDisappeared(0),
  ballDisappeared(false)
{
  init();
}

void BallStateEstimator::init()
{
  lastFrameTime = theFrameInfo.time;
  reset();
}

void BallStateEstimator::reset()
{
  stationaryBalls.clear();
  rollingBalls.clear();
  stationaryBalls.reserve(maxNumberOfHypotheses * 2);
  rollingBalls.reserve(maxNumberOfHypotheses * 2);
  bestState = nullptr;
}

void BallStateEstimator::update(BallModel& ballModel)
{
  // *** initial check: if a log file is played and the user steps back: reset the state estimation process
  if(SystemCall::getMode() == SystemCall::logFileReplay &&
     theFrameInfo.time <= lastFrameTime)
  {
    if(theFrameInfo.time < lastFrameTime)
      init();
    else
      return;
  }
  // *** Do state estimation
  ballWasSeenInThisFrame = theFilteredBallPercepts.percepts.size() > 0;
  seenStats.push_front(ballWasSeenInThisFrame ? 100 : 0);
  recomputeBestState = false;
  // perform prediction step for each filter
  motionUpdate(ballModel);
  // handle collision with feet, if robot is currently upright:
  if(theMotionInfo.isMotionStable)
    integrateCollisionWithFeet();
  if(recomputeBestState)
    findBestState();
  // sensor update step
  if(ballWasSeenInThisFrame)
  {
    // reset filters, if nothing has been seen recently
    if(theFrameInfo.getTimeSince(ballModel.timeWhenLastSeen) >= 4000)
      reset();

    // the current observation of a ball
    const FilteredBallPercept& ballPercept = theFilteredBallPercepts.percepts[0];

    // maybe there is also an older observation that has not been used
    // for state estimation as it has not been confirmed by the filtering before.
    // this can potentially be used for velocity computation
    if(theFilteredBallPercepts.percepts.size() > 1)
      lastBallPercept = theFilteredBallPercepts.percepts[1];

    // add current measurement to all filters
    for(auto& state : stationaryBalls)
      state.measurementUpdate(ballPercept.positionOnField, ballPercept.covOnField, ballPercept.radiusOnField);
    for(auto& state : rollingBalls)
      state.measurementUpdate(ballPercept.positionOnField, ballPercept.covOnField, ballPercept.radiusOnField);

    // normalize all weights
    normalizeMeasurementLikelihoods();
    // if there are too many hypotheses in a buffer, remove the worst ones
    pruneBallBuffer(stationaryBalls);
    pruneBallBuffer(rollingBalls);
    // find the best state
    findBestState();

    // create new filters (after having set the best ball, as new filters need
    // one measurement step in a future cycle)
    createNewFilters(ballPercept.positionOnField, ballPercept.radiusOnField, ballPercept.covOnField);

    // If we see a new ball for the first time after multiple seconds without any ball perceptions,
    // all filter become resetted, i.e. the lists are emptied and bestState == nullptr.
    // A newly created filter usually should not be used for model generation (see comment above).
    // However, if this new filter is the only one that we have, we should take it anyway!
    if(bestState == nullptr)
      findBestState(); // should return stationaryBalls[0], but calling the "official" function is cleaner.

    // save percept
    lastBallPercept = ballPercept;
  }

  // *** Generate model and perform post execution
  generateModel(ballModel);
  lastFrameTime = theFrameInfo.time;

  // *** Some plots and maybe later also drawings
  plotAndDraw();
}

void BallStateEstimator::motionUpdate(BallModel& ballModel)
{
  // prepare odometry transformation
  const float deltaTime = static_cast<float>(theFrameInfo.getTimeSince(lastFrameTime)) * 0.001f; // in seconds
  if(deltaTime <= 0.f) // Not sure, if all equations make sense, when time is running backwards...
    return;
  const Pose2f& odometryOffset = theOdometer.odometryOffset;
  const float odometryCos = std::cos(odometryOffset.rotation);
  const float odometrySin = std::sin(odometryOffset.rotation);
  const float odometryRotationDeviation = odometryOffset.rotation * odometryDeviation.rotation;
  const float odometryDeviationCos = std::cos(odometryRotationDeviation);
  const float odometryDeviationSin = std::sin(odometryRotationDeviation);
  const Vector2f squaredOdometryTranslationDeviation = (odometryOffset.translation.cwiseProduct(odometryDeviation.translation)).cwiseAbs2();
  Vector4f odometryTranslationCov;
  odometryTranslationCov << squaredOdometryTranslationDeviation, 0.f, 0.f;

  // prepare matrices and vectors for stationary filters
  Matrix2f fixedOdometryRotation;
  fixedOdometryRotation << odometryCos, odometrySin, -odometrySin, odometryCos; // a
  const Matrix2f fixedOdometryRotationTransposed = fixedOdometryRotation.transpose();
  Matrix2f fixedOdometryRotationDeviationRotation;
  fixedOdometryRotationDeviationRotation << odometryDeviationCos, odometryDeviationSin, -odometryDeviationSin, odometryDeviationCos;
  const Vector2f fixedOdometryTranslation = - fixedOdometryRotation * odometryOffset.translation; // u

  // prepare process noise
  const Vector4f squaredProcessCov = processDeviation.cwiseAbs2(); //square every element

  // add odometry offset to lastPercept in ball model
  if(!ballWasSeenInThisFrame)
  {
    Vector2f lastPercept = fixedOdometryRotation * ballModel.lastPerception + fixedOdometryTranslation;
    ballModel.lastPerception = lastPercept;
  }

  // perform prediction step for all stationary states
  for(auto& state : stationaryBalls)
  {
    state.motionUpdate(squaredProcessCov, odometryTranslationCov,
                       fixedOdometryRotation, fixedOdometryTranslation, fixedOdometryRotationTransposed,
                       fixedOdometryRotationDeviationRotation);
  }
  // perform prediction step for all moving states
  if(rollingBalls.size() > 0)
  {
    // prepare matrices and vectors for moving filters
    Matrix4f movingOdometryRotation;
    movingOdometryRotation << odometryCos, odometrySin, 0, 0,
                             -odometrySin, odometryCos, 0, 0,
                              0, 0, odometryCos, odometrySin,
                              0, 0, -odometrySin, odometryCos; // second part of "a"
    Matrix4f movingOdometryRotationDeviationRotation;
    movingOdometryRotationDeviationRotation << odometryDeviationCos, odometryDeviationSin, 0, 0,
                                              -odometryDeviationSin, odometryDeviationCos, 0, 0,
                                               0, 0, odometryDeviationCos, odometryDeviationSin,
                                               0, 0, -odometryDeviationSin, odometryDeviationCos;
    const Vector4f movingOdometryTranslation = movingOdometryRotation * Vector4f(-odometryOffset.translation.x(), -odometryOffset.translation.y(), 0, 0); // u
    Matrix4f movingMotionMatrix = Matrix4f::Identity();
    movingMotionMatrix(0, 2) = deltaTime;
    movingMotionMatrix(1, 3) = deltaTime;// first part of "a"
    const Matrix4f movingA = movingOdometryRotation * movingMotionMatrix;
    const Matrix4f movingATransposed = movingA.transpose();
    for(auto& state : rollingBalls)
    {
      state.motionUpdate(movingA, movingATransposed, movingOdometryTranslation,
                         squaredProcessCov, odometryTranslationCov, movingOdometryRotationDeviationRotation,
                         theBallSpecification.friction, deltaTime);
    }
  }

  // odometry update for the buffered ball percept (if it is new enough)
  if(theFrameInfo.getTimeSince(lastBallPercept.timeWhenSeen) < lastBallPerceptTimeout)
  {
    lastBallPercept.positionOnField = fixedOdometryRotation * lastBallPercept.positionOnField + fixedOdometryTranslation;
    lastBallPercept.covOnField(0, 0) += odometryTranslationCov[0];
    lastBallPercept.covOnField(1, 1) += odometryTranslationCov[1];
    Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * lastBallPercept.positionOnField - lastBallPercept.positionOnField).cwiseAbs2();
    lastBallPercept.covOnField(0, 0) += squaredOdometryRotationDeviationTranslation.x();
    lastBallPercept.covOnField(1, 1) += squaredOdometryRotationDeviationTranslation.y();
  }

  // Check, if some of the rolling balls have stopped and move them to the buffer
  // that contains the stationary balls. They are added to the end of the list, which
  // now might temporarily exceed its limit.
  auto rBall = rollingBalls.begin();
  while(rBall != rollingBalls.end())
  {
    if(rBall->getVelocity().norm() < minSpeed)
    {
      stationaryBalls.push_back(rBall->toStationaryBallKalmanFilter());
      rBall = rollingBalls.erase(rBall);
      recomputeBestState = true;  // as pointer bestState might be incorrect after this operation
    }
    else
    {
      ++rBall;
    }
  }
}

void BallStateEstimator::normalizeMeasurementLikelihoods()
{
  float highestLikelihoodOfMeasurements = 0;
  for(auto& state : rollingBalls)
  {
    if(state.likelihoodOfMeasurements > highestLikelihoodOfMeasurements)
      highestLikelihoodOfMeasurements = state.likelihoodOfMeasurements;
  }
  for(auto& state : stationaryBalls)
  {
    if(state.likelihoodOfMeasurements > highestLikelihoodOfMeasurements)
      highestLikelihoodOfMeasurements = state.likelihoodOfMeasurements;
  }
  ASSERT(highestLikelihoodOfMeasurements > 0.f || (stationaryBalls.size() == 0 && rollingBalls.size() == 0));
  for(auto& state : stationaryBalls)
    state.likelihoodOfMeasurements /= highestLikelihoodOfMeasurements;
  for(auto& state : rollingBalls)
    state.likelihoodOfMeasurements /= highestLikelihoodOfMeasurements;
}

void BallStateEstimator::findBestState()
{
  bestState = nullptr;
  float bestWeighting = -1.f;
  for(auto& state : rollingBalls)
    if(bestState == nullptr || state.weighting > bestWeighting)
    {
      bestState = &state;
      bestWeighting = state.weighting;
    }
  for(auto& state : stationaryBalls)
    if(bestState == nullptr || state.weighting > bestWeighting)
    {
      bestState = &state;
      bestWeighting = state.weighting;
    }
}

template <typename T> void BallStateEstimator::pruneBallBuffer(std::vector<T, Eigen::aligned_allocator<T>>& balls)
{
  if(balls.size() <= maxNumberOfHypotheses - 1)
    return;
  std::sort(balls.begin(), balls.end(), compareHypothesesWeightings);
  while(balls.size() > maxNumberOfHypotheses - 1)
    balls.pop_back();
}

void BallStateEstimator::createNewFilters(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov)
{
  // Create a new stationary state estimator
  StationaryBallKalmanFilter newStationaryBall;
  newStationaryBall.likelihoodOfMeasurements = initialStateWeight;
  newStationaryBall.radius = ballPerceptRadius;
  newStationaryBall.x = ballPercept;
  newStationaryBall.P = ballPerceptCov;
  newStationaryBall.lastPosition = ballPercept;
  newStationaryBall.numOfMeasurements = 1;
  stationaryBalls.push_back(newStationaryBall);

  // Try to create a new filter for a rolling ball (if a second observation is available)
  if(theFrameInfo.getTimeSince(lastBallPercept.timeWhenSeen) < lastBallPerceptTimeout)
  {
    const float deltaTime = static_cast<float>(theFrameInfo.getTimeSince(lastBallPercept.timeWhenSeen)) * 0.001f; // in seconds
    if(deltaTime <= 0.f) // Not sure, if all equations make sense, when time is running backwards...
      return;
    Vector2f ballVelocity = BallPhysics::velocityAfterDistanceForTime(lastBallPercept.positionOnField, ballPercept, deltaTime, theBallSpecification.friction);
    const float ballSpeed = ballVelocity.norm();
    if(ballSpeed > minSpeed && ballSpeed < theBallSpecification.ballSpeedUpperBound()) // Do not generate slow or very fast balls
    {
      RollingBallKalmanFilter newRollingBall;
      newRollingBall.likelihoodOfMeasurements = initialStateWeight;
      newRollingBall.radius = ballPerceptRadius;
      newRollingBall.numOfMeasurements = 2;
      newRollingBall.x << ballPercept, ballVelocity;
      newRollingBall.P << ballPerceptCov, Matrix2f::Identity(),
                       Matrix2f::Identity(), (ballPerceptCov + lastBallPercept.covOnField) / deltaTime;
      newRollingBall.lastPosition = ballPercept;
      rollingBalls.push_back(newRollingBall);
    }
  }
}

void BallStateEstimator::generateModel(BallModel& ballModel)
{
  ballModel.seenPercentage = static_cast<unsigned char>(seenStats.average());
  if(bestState != nullptr)
  {
    ballModel.estimate.position = bestState->getPosition();
    // <hack>
    if(bestState->numOfMeasurements < minNumberOfMeasurementsForRollingBalls)
      ballModel.estimate.velocity = Vector2f::Zero();
    else
      ballModel.estimate.velocity = bestState->getVelocity();
    // <hack-description>This seems to be the best temporary
    // solution to avoid the generation of quite high velocities due to noise between two measurements.
    // T.L.
    // </hack-description>
    // </hack>
    ballModel.estimate.covariance = bestState->getPositionCovariance();
    ballModel.estimate.radius = bestState->radius;
    Covariance::fixCovariance(ballModel.estimate.covariance);
  }
  if(ballWasSeenInThisFrame)
  {
    ballModel.timeWhenLastSeen = theFrameInfo.time;
    ballModel.lastPerception = theFilteredBallPercepts.percepts[0].positionOnField;
  }

  // update timeWhenDisappeared
  {
    bool ballWasSeen = ballModel.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0;
    bool ballShouldBeVisible = ballModel.estimate.position.norm() < ballDisappearedMaxCheckingDistance &&
                               BallLocatorTools::ballShouldBeVisibleInCurrentImage(ballModel.estimate.position, ballModel.estimate.radius,
                                   theCameraMatrix, theCameraInfo, theBodyContour, theImageCoordinateSystem);
    if(ballWasSeen)
    {
      timeWhenBallFirstDisappeared = theFrameInfo.time;
      ballDisappeared = false;
    }
    else if(ballShouldBeVisible)
    {
      ballDisappeared = true;
    }
    else if(!ballDisappeared)
    {
      timeWhenBallFirstDisappeared = theFrameInfo.time;
    }
    if(theFrameInfo.getTimeSince(timeWhenBallFirstDisappeared) > ballDisappearedTimeout)
      ballModel.timeWhenDisappeared = timeWhenBallFirstDisappeared;
    else
      ballModel.timeWhenDisappeared = theFrameInfo.time;
  }
}

void BallStateEstimator::integrateCollisionWithFeet()
{
  BallContactInformation contactInfo;
  for(auto& state : rollingBalls)
  {
    if(theBallContactChecker.collide(state.getPosition(), state.getVelocity(), state.lastPosition, contactInfo))
    {
      state.x << contactInfo.newPosition, contactInfo.newVelocity;
      state.P(2, 2) += contactInfo.addVelocityCov.x();
      state.P(3, 3) += contactInfo.addVelocityCov.y();
    }
  }
  auto sBall = stationaryBalls.begin();
  while(sBall != stationaryBalls.end())
  {
    if(theBallContactChecker.collide(sBall->getPosition(), sBall->getVelocity(), sBall->lastPosition, contactInfo))
    {
      if(contactInfo.newVelocity.norm() < minSpeed) // Ball does not start moving
      {
        sBall->x = contactInfo.newPosition;
        ++sBall;
      }
      else // Ball is rolling after collision -> Create a rolling ball and delete stationary ball
      {
        rollingBalls.push_back(RollingBallKalmanFilter(*sBall, contactInfo.newPosition, contactInfo.newVelocity, contactInfo.addVelocityCov));
        sBall = stationaryBalls.erase(sBall);
        recomputeBestState = true;  // as pointer bestState might be incorrect after this operation
      }
    }
    else
    {
      ++sBall;
    }
  }
}

void BallStateEstimator::plotAndDraw()
{
  PLOT("module:BallStateEstimator:stationaryHypotheses", stationaryBalls.size());
  PLOT("module:BallStateEstimator:movingHypotheses", rollingBalls.size());
}
