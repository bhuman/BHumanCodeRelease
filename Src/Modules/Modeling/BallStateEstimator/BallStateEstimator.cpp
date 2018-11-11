/**
 * @file BallStateEstimator.cpp
 *
 * For ball tracking in the RoboCup context, two tasks need to be solved:
 *   1. Filtering and clustering the detected balls (i.e. the BallPercepts) to avoid
 *      the use of any false positive, which might lead to severe problems.
 *   2. Estimating a precise position as well as a velocity based on a set of recent
 *      ball observations.
 * In previous implementations, these two tasks have been combined within one module. For more
 * clarity and more flexibility, both tasks are split into two modules now.
 *
 * This module provides a solution for task 2: State estimation.
 *
 * The module implemented in this file is based on the implementation that has been used by
 * B-Human for multiple years: estimating the ball position and velocity by maintaining
 * a set of normal Kalman filters.
 *
 * @author Tim Laue
 */

#include "BallStateEstimator.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Random.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(BallStateEstimator, modeling)

BallStateEstimator::BallStateEstimator(): timeWhenBallFirstDisappeared(0),
  ballDisappeared(false)
{
  init();
}

void BallStateEstimator::init()
{
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
  reset();
}

void BallStateEstimator::reset()
{
  numberOfStates = 0;
  bestState = nullptr;
}

void BallStateEstimator::update(BallModel& ballModel)
{
  // *** initial check: if a logfile is played and the user steps back: reset the state estimation process
  if(SystemCall::getMode() == SystemCall::logfileReplay &&
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
  // perform prediction step for each filter
  motionUpdate(ballModel);
  // handle collision with feet
  if(bestState != nullptr && theMotionInfo.isMotionStable && theBallContactWithRobot.timeOfLastContact == theFrameInfo.time)
    integrateCollisionWithFeet();
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
    sensorUpdate(ballPercept.positionOnField, ballPercept.covOnField, ballPercept.radiusOnField);

    // normalize the weights and find best and worst filters
    KalmanFilterBallHypothesis* worstStationaryState, *worstMovingState;
    normalizeWeights(worstStationaryState, worstMovingState);

    // create new filters
    createNewStates(ballPercept.positionOnField, ballPercept.radiusOnField, ballPercept.covOnField, worstStationaryState, worstMovingState);

    // save percept
    lastBallPercept = ballPercept;
  }

  // *** Generate model and perform post execution
  generateModel(ballModel);
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
  
  // *** Some plots and maybe later also drawings
  plotAndDraw();
}

void BallStateEstimator::motionUpdate(BallModel& ballModel)
{
  // prepare odometry transformation
  Pose2f odometryOffset = theOdometryData - lastOdometryData;
  float deltaTime = (theFrameInfo.time - lastFrameTime) * 0.001f; // in seconds
  float odometryCos = std::cos(odometryOffset.rotation);
  float odometrySin = std::sin(odometryOffset.rotation);
  float odometryRotationDeviation = odometryOffset.rotation * odometryDeviation.rotation;
  float odometryDeviationCos = std::cos(odometryRotationDeviation);
  float odometryDeviationSin = std::sin(odometryRotationDeviation);
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

  // prepare process noise
  const Vector4f squaredProcessCov = processDeviation.cwiseAbs2(); //square every element

  // add odometry offset to lastPercept in ball model
  if(!ballWasSeenInThisFrame)
  {
    Vector2f lastPercept = fixedOdometryRotation * ballModel.lastPerception + fixedOdometryTranslation;
    ballModel.lastPerception = lastPercept;
  }

  // perform prediction step for each state
  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
  {
    state->motionUpdate(movingA, movingATransposed, movingOdometryTranslation,
                        squaredProcessCov, odometryTranslationCov, movingOdometryRotationDeviationRotation,
                        fixedOdometryRotation, fixedOdometryTranslation, fixedOdometryRotationTransposed,
                        fixedOdometryRotationDeviationRotation, theBallSpecification.friction, deltaTime);
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
}

void BallStateEstimator::sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov, float ballRadius)
{
  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
    state->sensorUpdate(measurement, measurementCov, ballRadius);
}

void BallStateEstimator::normalizeWeights(KalmanFilterBallHypothesis*& worstStationaryState, KalmanFilterBallHypothesis*& worstMovingState)
{
  bestState = nullptr;
  worstStationaryState = 0;
  worstMovingState = 0;
  KalmanFilterBallHypothesis*& worstState = worstStationaryState;
  KalmanFilterBallHypothesis*& secondWorstState = worstMovingState;

  float highestHeight = 0;
  float highestWeight = 0;
  float worstGain = 0;
  float secondWorstGain = 0;

  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
  {
    if(state->weight > highestWeight)
      highestWeight = state->weight;
    if(bestState == nullptr || state->height > highestHeight)
    {
      bestState = state;
      highestHeight = state->height;
    }
  }
  ASSERT(highestWeight > 0.f || numberOfStates == 0);

  // Find the worst states:
  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
  {
    state->weight /= highestWeight;
    if(state != bestState)
    {
      float gain = state->gain;
      if(!worstState || gain < worstGain)
      {
        secondWorstState = worstState;
        secondWorstGain = worstGain;
        worstState = state;
        worstGain = gain;
      }
      else if(!secondWorstState || gain < secondWorstGain)
      {
        secondWorstState = state;
        secondWorstGain = gain;
      }
    }
  }
}

void BallStateEstimator::createNewStates(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov, KalmanFilterBallHypothesis* worstStationaryState, KalmanFilterBallHypothesis* worstMovingState)
{
  // create new fixed state
  ASSERT(worstStationaryState || static_cast<unsigned long>(numberOfStates) < sizeof(states) / sizeof(*states));
  KalmanFilterBallHypothesis* newState = worstStationaryState;
  if(static_cast<unsigned long>(numberOfStates) < sizeof(states) / sizeof(*states))
    newState = &states[numberOfStates++];
  ASSERT(newState);
  newState->type = KalmanFilterBallHypothesis::stationary;
  ASSERT(numberOfStates > 0);
  newState->weight = initialStateWeight;
  newState->radius = ballPerceptRadius;
  newState->stationaryX = ballPercept;
  newState->stationaryCov = ballPerceptCov;
  newState->numOfMeasurements = 1;

  if(theFrameInfo.getTimeSince(lastBallPercept.timeWhenSeen) < lastBallPerceptTimeout)
  {
    ASSERT(worstMovingState || static_cast<unsigned long>(numberOfStates) < sizeof(states) / sizeof(*states));
    KalmanFilterBallHypothesis* newState = worstMovingState;
    if(static_cast<unsigned long>(numberOfStates) < sizeof(states) / sizeof(*states))
    {
      newState = &states[numberOfStates++];
    }
    // >>>> These lines should not be reached but in theory, it might be possible. This module requries a complete rewrite!
    else if(worstMovingState == 0)
    {
      newState = &states[Random::uniformInt(0,11)];
      ANNOTATION("BallStateEstimator", "Oh no!");
      if(newState == bestState) // Meh, try again in next cycle
        return;
    }
    // <<<< End of hack
    ASSERT(newState);
    newState->type = KalmanFilterBallHypothesis::moving;
    ASSERT(numberOfStates > 1);
    newState->weight = initialStateWeight;
    newState->radius = ballPerceptRadius;
    newState->numOfMeasurements = 2;
    const float dt = (theFrameInfo.time - lastBallPercept.timeWhenSeen) * 0.001f;
    Vector2f ballVelocity = BallPhysics::velocityAfterDistanceForTime(lastBallPercept.positionOnField, ballPercept, dt, theBallSpecification.friction);
    const float ballSpeed = ballVelocity.norm();
    const float upperSpeedBound = theBallSpecification.ballSpeedUpperBound();
    if(ballSpeed > upperSpeedBound)
    {
      ballVelocity.normalize(upperSpeedBound);
    }
    newState->movingX << ballPercept, ballVelocity; // z
    newState->movingCov << ballPerceptCov, Matrix2f::Identity(),
             Matrix2f::Identity(), (ballPerceptCov + lastBallPercept.covOnField) / dt;
  }
}

void BallStateEstimator::generateModel(BallModel& ballModel)
{
  ballModel.seenPercentage = static_cast<unsigned char>(seenStats.average());
  if(bestState == nullptr && numberOfStates > 0)
    bestState = &states[0];
  if(bestState != nullptr)
  {
    BallState& ballState = ballModel.estimate;
    if(bestState->type == KalmanFilterBallHypothesis::moving)
    {
      ballState.position = bestState->movingX.topRows(2);
      // <hack>
      if(bestState->numOfMeasurements < minNumberOfMeasurementsForRollingBalls)
        ballState.velocity = Vector2f::Zero();
      else
        ballState.velocity = bestState->movingX.bottomRows(2);
      // <hack-description>This module needs a complete rewrite (SOON!) and this seems to be the best temporary
      // solution to avoid the generation of quite high velocities due to noise between two measurements.
      // T.L.
      // </hack-description>
      // </hack>
      ballState.covariance = bestState->movingCov.topLeftCorner(2, 2);
    }
    else
    {
      ballState.position = bestState->stationaryX;
      ballState.velocity = Vector2f::Zero();
      ballState.covariance = bestState->stationaryCov;
    }
    ballState.radius = bestState->radius;
    Covariance::fixCovariance(ballState.covariance);
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
  const Vector2f& newVelocity = theBallContactWithRobot.newVelocity;
  const Vector2f& newPosition = theBallContactWithRobot.newPosition;
  const Vector2f& addVelocityCov = theBallContactWithRobot.addVelocityCov;
  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
  {
    if(state->type == KalmanFilterBallHypothesis::stationary)
    {
      state->stationaryX = newPosition;
    }
    else // if(state->type == KalmanFilterBallHypothesis::moving)
    {
      state->movingX << newPosition, newVelocity;
      Matrix4f& cov = state->movingCov;
      cov(2, 2) += addVelocityCov.x();
      cov(3, 3) += addVelocityCov.y();
    }
  }
  
  // Set the pointer to the best moving state (necessary for handling kicked balls)
  KalmanFilterBallHypothesis* bestMovingState = nullptr;
  if(bestState != nullptr && bestState->type == KalmanFilterBallHypothesis::moving)
  {
    bestMovingState = bestState;
  }
  else
  {
    float highestMotionHeight = 0.f;
    for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
    {
      if(state->type != KalmanFilterBallHypothesis::moving)
        continue;
      if(bestMovingState == nullptr || state->height > highestMotionHeight)
      {
        bestMovingState = state;
        highestMotionHeight = state->height;
      }
    }
  }
  
  // HACK! Activate best moving hypothesis as ball is often not seen after a kick
  if(newVelocity.norm() > 0.f && bestMovingState != nullptr)
  {
  //  OUTPUT(idText, text, "Setting best hypothesis to a moving one!" <<
  //                       "(" << bestMovingState->movingX(0) << " / " << bestMovingState->movingX(1) << ")  " <<
  //                       "(" << bestMovingState->movingX(2) << " / " << bestMovingState->movingX(3) << ")");
    bestState = bestMovingState;
    // "Workaround"
    if(bestState->numOfMeasurements < minNumberOfMeasurementsForRollingBalls)
      bestState->numOfMeasurements = minNumberOfMeasurementsForRollingBalls;
  }
  
  // Hmn, seems that we need to replace a state
  if(bestMovingState == nullptr && bestState != nullptr)
  {
    bestMovingState = bestState;
    bestMovingState->movingX << newPosition, newVelocity;
    Matrix2f& cov = bestMovingState->stationaryCov;
    bestMovingState->movingCov = Matrix4f::Identity();
    bestMovingState->movingCov.topLeftCorner(2, 2) << cov;
    bestMovingState->type = KalmanFilterBallHypothesis::moving;
    Matrix4f& covM = bestState->movingCov;
    covM(2, 2) += addVelocityCov.x();
    covM(3, 3) += addVelocityCov.y();
    //OUTPUT(idText, text, "DAMN! NO MOVING STATES!");
  }
}

void BallStateEstimator::plotAndDraw()
{
  int numberOfMovingHypotheses = 0;
  int numberOfStationaryHypotheses = 0;
  for(KalmanFilterBallHypothesis* state = states, *end = states + numberOfStates; state < end; ++state)
  {
    if(state->type == KalmanFilterBallHypothesis::stationary)
      numberOfStationaryHypotheses++;
    else // if(state->type == KalmanFilterBallHypothesis::moving)
      numberOfMovingHypotheses++;
  }
  PLOT("module:BallStateEstimator:stationaryHypotheses", numberOfStationaryHypotheses);
  PLOT("module:BallStateEstimator:movingHypotheses", numberOfMovingHypotheses);
}
