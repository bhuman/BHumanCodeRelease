/**
* @file BallLocator.cpp
* Implementation of the BallLocator module.
* @author Colin Graf
*/

#include <algorithm>
#include "BallLocator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(BallLocator, modeling)

BallLocator::BallLocator() : timeWhenBallFirstDisappeared(0), ballDisappeared(true), ballWasBeenSeenInLastLowerCameraImage(false)
{
  init();
}

void BallLocator::init()
{
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
  reset();
}

void BallLocator::reset()
{
  stateCount = 0;
  bestState = nullptr;
  hasLastBallPercept = false;
}

void BallLocator::update(BallModel& ballModel)
{
  DECLARE_DEBUG_DRAWING("module:BallLocator:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallLocator:Image", "drawingOnImage");

  if(SystemCall::getMode() == SystemCall::logfileReplay &&
     theFrameInfo.time <= lastFrameTime)
  {
    if(theFrameInfo.time < lastFrameTime)
      init();
    else
      return;
  }

  // set extra members for seen ball to achieve the following behavior:
  //    if the ball has been seen in the last lower camera image, it will be ignored in
  //    the upper image to exclude any clutter in the robot's environment
  ballWasSeenInThisFrame = theBallPercept.status == BallPercept::seen;

  seenStats.push_front(ballWasSeenInThisFrame ? 100 : 0);
  ballModel.seenPercentage = (unsigned char) seenStats.average();

  if(ballWasSeenInThisFrame && theCameraInfo.camera == CameraInfo::upper && ballWasBeenSeenInLastLowerCameraImage)
    ballWasSeenInThisFrame = false;
  else if(theCameraInfo.camera == CameraInfo::lower)
    ballWasBeenSeenInLastLowerCameraImage = ballWasSeenInThisFrame;

  // perform prediction step for each filter
  motionUpdate(ballModel);

  // detect and handle collision with feet
  Pose3f leftFoot = theTorsoMatrix;
  Pose3f rightFoot = theTorsoMatrix;
  leftFoot.conc(theRobotModel.limbs[Limbs::footLeft].translated(footOffset.x(), footOffset.y(), -theRobotDimensions.footHeight));
  rightFoot.conc(theRobotModel.limbs[Limbs::footRight].translated(footOffset.x(), -footOffset.y(), -theRobotDimensions.footHeight));
  Vector2f leftFootCenter = leftFoot.translation.topRows(2);
  Vector2f rightFootCenter = rightFoot.translation.topRows(2);
  COMPLEX_DRAWING("module:BallLocator:Image")
  {
    Vector2f leftCenterImage, leftTopImage, rightCenterImage, rightTopImage;
    if(Transformation::robotToImage(leftFootCenter, theCameraMatrix, theCameraInfo, leftCenterImage) &&
       Transformation::robotToImage(Vector2f(leftFootCenter.x() + footRadius, leftFootCenter.y()), theCameraMatrix, theCameraInfo, leftTopImage))
    {
      CIRCLE("module:BallLocator:Image", leftCenterImage.x(), leftCenterImage.y(), (leftCenterImage - leftTopImage).norm(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA());
    }
    if(Transformation::robotToImage(rightFootCenter, theCameraMatrix, theCameraInfo, rightCenterImage) &&
       Transformation::robotToImage(Vector2f(rightFootCenter.x() + footRadius, rightFootCenter.y()), theCameraMatrix, theCameraInfo, rightTopImage))
    {
      CIRCLE("module:BallLocator:Image", rightCenterImage.x(), rightCenterImage.y(), (rightCenterImage - rightTopImage).norm(), 0, Drawings::solidPen, ColorRGBA(0, 0, 0), Drawings::noBrush, ColorRGBA());
    }
  }
  if(bestState != nullptr)
    handleCollisionWithFeet(leftFootCenter, rightFootCenter);

  // sensor update step
  const Vector2f& ballPercept = theBallPercept.relativePositionOnField;
  Matrix2f ballPerceptCov;
  if(ballWasSeenInThisFrame)
  {
    // reset filters..
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) >= 4000)
      reset();

    // calculate variance of the percept
    ballPerceptCov = getCovOfPixelInWorld(theImageCoordinateSystem.toCorrected(theBallPercept.positionInImage), theBallPercept.radiusOnField);

    // add current measurement to all filters
    sensorUpdate(ballPercept, ballPerceptCov);

    // normalize the weights and find best and worst filters
    State* worstStationaryState, * worstMovingState;
    normalizeWeights(bestState, worstStationaryState, worstMovingState);

    // create new filters
    createNewStates(ballPercept, theBallPercept.radiusOnField, ballPerceptCov, worstStationaryState, worstMovingState);
  }

  //
  COMPLEX_DRAWING("module:BallLocator:Field")
  {
    for(State* state = states, * end = states + stateCount; state < end; ++state)
      if(state->type == State::moving)
      {
        DOT("module:BallLocator:Field", state->movingX[0], state->movingX[1], ColorRGBA(0xff, 0xaa, 0xaa), ColorRGBA(0xff, 0xaa, 0xaa));
      }
      else // state->type == State::stationary
      {
        DOT("module:BallLocator:Field", state->stationaryX.x(), state->stationaryX.y(), ColorRGBA(0xaa, 0xff, 0xaa), ColorRGBA(0xaa, 0xff, 0xaa));
      }
  }

  // generate model
  generateModel(ballModel);

  // post execution
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
  lastLeftFootCenter = leftFootCenter;
  lastRightFootCenter = rightFootCenter;
  lastBallPosition = ballModel.estimate.position;
  if(ballWasSeenInThisFrame)
  {
    lastBallPercept = ballPercept;
    lastBallPerceptRadius = theBallPercept.radiusOnField;
    lastBallPerceptTimeStamp = theFrameInfo.time;
    lastBallPerceptCov = ballPerceptCov;
    hasLastBallPercept = true;
  }
}

void BallLocator::motionUpdate(BallModel& ballModel)
{
  // prepare odometry transformation
  Pose2f odometryOffset = theOdometryData - lastOdometryData;
  deltaTime = (theFrameInfo.time - lastFrameTime) * 0.001f; // in seconds
  float odometryCos = std::cos(odometryOffset.rotation);
  float odometrySin = std::sin(odometryOffset.rotation);
  float odometryRotationDeviation = odometryOffset.rotation * odometryDeviation.rotation;
  float odometryDeviationCos = std::cos(odometryRotationDeviation);
  float odometryDeviationSin = std::sin(odometryRotationDeviation);
  Vector2f squaredOdometryTranslationDeviation = (odometryOffset.translation.cwiseProduct(odometryDeviation.translation)).cwiseAbs2();
  Vector4f odometryTranslationCov;
  odometryTranslationCov << squaredOdometryTranslationDeviation, 0.f, 0.f;

  // prepare matrices and vectors for stationary filters
  Matrix2f fixedOdometryRotation;
  fixedOdometryRotation << odometryCos, odometrySin, -odometrySin, odometryCos; // a
  Matrix2f fixedOdometryRotationTransposed = fixedOdometryRotation.transpose();
  Matrix2f fixedOdometryRotationDeviationRotation;
  fixedOdometryRotationDeviationRotation << odometryDeviationCos, odometryDeviationSin , -odometryDeviationSin, odometryDeviationCos;
  Vector2f fixedOdometryTranslation = -odometryOffset.translation; // u

  // prepare matrices and vectors for moving filters
  Matrix4f movingOdometryRotation;
  movingOdometryRotation << odometryCos, odometrySin, 0,  0,
                         -odometrySin, odometryCos, 0, 0,
                         0, 0, odometryCos, odometrySin,
                         0, 0, -odometrySin, odometryCos; // second part of "a"
  Matrix4f movingOdometryRotationDeviationRotation;
  movingOdometryRotationDeviationRotation << odometryDeviationCos, odometryDeviationSin, 0, 0,
                                          -odometryDeviationSin, odometryDeviationCos, 0, 0,
                                          0, 0, odometryDeviationCos, odometryDeviationSin,
                                          0, 0, -odometryDeviationSin, odometryDeviationCos;
  Vector4f movingOdometryTranslation(-odometryOffset.translation.x(), -odometryOffset.translation.y(), 0, 0); // u
  Matrix4f movingMotionMatrix = Matrix4f::Identity();
  movingMotionMatrix(0, 2) = deltaTime;
  movingMotionMatrix(1, 3) = deltaTime;// first part of "a"
  Matrix4f movingA = movingOdometryRotation * movingMotionMatrix;
  Matrix4f movingATransposed = movingA.transpose();

  // prepare process noise
  Vector4f squaredProcessCov = processDeviation.cwiseAbs2(); //square every element

  // add odometry offset to the last percept
  if(hasLastBallPercept)
  {
    lastBallPercept = fixedOdometryRotation * lastBallPercept + fixedOdometryTranslation;
    lastBallPerceptCov(0, 0) += odometryTranslationCov[0];
    lastBallPerceptCov(1, 1) += odometryTranslationCov[1];
    Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * lastBallPercept - lastBallPercept).cwiseAbs2();
    lastBallPerceptCov(0, 0) += squaredOdometryRotationDeviationTranslation.x();
    lastBallPerceptCov(1, 1) += squaredOdometryRotationDeviationTranslation.y();
  }

  // add odometry offset to lastPercept and lastSeenEstimate
  if(!ballWasSeenInThisFrame)
  {
    Vector2f lastPercept = fixedOdometryRotation * ballModel.lastPerception + fixedOdometryTranslation;
    ballModel.lastPerception = lastPercept;
  }

  // perform prediction step for each state
  for(State* state = states, * end = states + stateCount; state < end; ++state)
  {
    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Matrix4f& cov = state->movingCov;

      // predict
      x = movingA * x + movingOdometryTranslation;
      cov = movingA * cov * movingATransposed;

      // add process noise
      for(int i = 0; i < 4; ++i)
        cov(i, i) += squaredProcessCov[i];

      // add odometry translation noise
      for(int i = 0; i < 4; ++i)
        cov(i, i) += odometryTranslationCov[i];

      // add noise from odometry rotation (crude approximation)
      Vector4f squaredOdometryRotationDeviationTranslation = (movingOdometryRotationDeviationRotation * x - x).cwiseAbs2();
      for(int i = 0; i < 4; ++i)
        cov(i, i) += squaredOdometryRotationDeviationTranslation[i];
    }
    else // state->type == State::stationary
    {
      Vector2f& x = state->stationaryX;
      Matrix2f& cov = state->stationaryCov;

      // predict
      x = fixedOdometryRotation * x + fixedOdometryTranslation;
      cov = fixedOdometryRotation * cov * fixedOdometryRotationTransposed;

      // add process noise
      cov(0, 0) += squaredProcessCov[0];
      cov(1, 1) += squaredProcessCov[1];

      // add odometry translation noise
      cov(0, 0) += odometryTranslationCov[0];
      cov(1, 1) += odometryTranslationCov[1];

      // add noise from odometry rotation (crude approximation)
      Vector2f squaredOdometryRotationDeviationTranslation = (fixedOdometryRotationDeviationRotation * x - x).cwiseAbs2();
      cov(0, 0) += squaredOdometryRotationDeviationTranslation.x();
      cov(1, 1) += squaredOdometryRotationDeviationTranslation.y();
    }
    // apply friction
    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Vector2f newPosition = x.topRows(2);
      Vector2f newVelocity = x.bottomRows(2);
      BallPhysics::applyFrictionToPositionAndVelocity(newPosition, newVelocity, deltaTime, theFieldDimensions.ballFriction);
      x << newPosition, newVelocity;
      if(newVelocity.squaredNorm() < sqr(100.f))
      {
        // the estimate stopped moving
        state->type = State::stationary;
        state->stationaryX = x.topRows(2);
        Matrix4f& cov = state->movingCov;
        state->stationaryCov << cov.topLeftCorner(2, 2);
      }
    }
  }
}

void BallLocator::handleCollisionWithFeet(const Vector2f& leftFootCenter, const Vector2f& rightFootCenter)
{
  ASSERT(bestState != nullptr);
  ASSERT(leftFootCenter != Vector2f::Zero() && rightFootCenter != Vector2f::Zero());

  Vector2f ballPosition = bestState->stationaryX;
  Vector2f ballVelocity = Vector2f::Zero();
  if(bestState->type == State::moving)
  {
    ballPosition = bestState->movingX.topRows(2);
    ballVelocity = bestState->movingX.bottomRows(2);
  }

  bool leftCollision = false, rightCollision = false, centerCollision = false;
  float leftCollisionFactor, rightCollisionFactor;
  float assumedRadius = footRadius + bestState->radius;

  // detect collision with the left foot
  Vector2f leftAssumedLastBallPosition = lastBallPosition + (leftFootCenter - lastLeftFootCenter);
  Vector2f leftAssumedBallOffset = ballPosition - leftAssumedLastBallPosition;
  leftCollision = getSmallestLineWithCircleIntersectionFactor(leftAssumedLastBallPosition, leftAssumedBallOffset, lastLeftFootCenter, assumedRadius, leftCollisionFactor);
  if(leftCollision && (leftCollisionFactor < 0.f || leftCollisionFactor > 1.f))
    leftCollision = false;

  // detect collision with the right foot
  Vector2f rightAssumedLastBallPosition = lastBallPosition + (rightFootCenter - lastRightFootCenter);
  Vector2f rightAssumedBallOffset = ballPosition - rightAssumedLastBallPosition;
  rightCollision = getSmallestLineWithCircleIntersectionFactor(rightAssumedLastBallPosition, rightAssumedBallOffset, lastRightFootCenter, assumedRadius, rightCollisionFactor);
  if(rightCollision && (rightCollisionFactor < 0.f || rightCollisionFactor > 1.f))
    rightCollision = false;

  // handle collision
  if(leftCollision || rightCollision)
  {
    // pick primary collision
    if(leftCollision && rightCollision)
    {
      if(leftCollisionFactor < rightCollisionFactor)
        rightCollision = false;
      else
        leftCollision = false;
    }
    ASSERT((leftCollision || rightCollision) && !(leftCollision && rightCollision));

    // calculate new position and velocity
    float collisionFactor = leftCollision ? leftCollisionFactor : rightCollisionFactor;
    const Vector2f& footCenter = leftCollision ? leftFootCenter : rightFootCenter;
    const Vector2f& lastFootCenter = leftCollision ? lastLeftFootCenter : lastRightFootCenter;
    Vector2f collisionPoint = leftCollision ? (leftAssumedLastBallPosition + leftAssumedBallOffset * leftCollisionFactor) : (rightAssumedLastBallPosition + rightAssumedBallOffset * rightCollisionFactor);
    float collisionAngle = (collisionPoint - lastFootCenter).angle();

    ballPosition = collisionPoint;
    ballVelocity = Vector2f::Zero();

    float footMomentum = (footCenter - lastFootCenter).norm() / deltaTime * footMass;
    float passedMomentum = footMomentum * std::cos(std::abs(Angle::normalize(collisionAngle - (footCenter - lastFootCenter).angle())));
    // TODO: calculate reflected momentum?
    if(passedMomentum > 0.f)
    {
      Vector2f tmp = collisionPoint - lastFootCenter;
      ballVelocity = tmp.normalize(passedMomentum) / ballMass;
      ballPosition += ballVelocity * (1.f - collisionFactor) * deltaTime;
    }
  }
  else
  {
    // some clipping for sumo position
    float factor1, factor2;
    if(getLineWithLineIntersectionFactors(leftFootCenter, rightFootCenter - leftFootCenter, lastBallPosition, ballPosition - lastBallPosition, factor1, factor2))
      if(factor1 > 0.f && factor1 < 1.f && factor2 > 0.f && factor2 < 1.f)
      {
        centerCollision = true;
        ballPosition = leftFootCenter + (rightFootCenter - leftFootCenter) * factor1;
        ballVelocity = Vector2f::Zero();
      }
  }

  // it is still possible that the ball is within the foot circles
  // if so, calculate a ball shift vector
  Vector2f ballShift = Vector2f::Zero();
  if((ballPosition - leftFootCenter).squaredNorm() < sqr(assumedRadius))
  {
    Vector2f tmp = ballPosition - leftFootCenter;
    ballShift += tmp.normalize(assumedRadius - (ballPosition - leftFootCenter).norm());
  }
  if((ballPosition - rightFootCenter).squaredNorm() < sqr(assumedRadius))
  {
    Vector2f tmp = ballPosition - rightFootCenter;
    ballShift += tmp.normalize(assumedRadius - (ballPosition - rightFootCenter).norm());
  }
  ballPosition += ballShift;

  // apply new position and velocity
  if(leftCollision || rightCollision || centerCollision || ballShift != Vector2f::Zero())
  {
    if(ballVelocity != Vector2f::Zero())
    {
      bestState->movingX << ballPosition, ballVelocity;
      if(bestState->type != State::moving)
      {
        bestState->type = State::moving;
        Matrix2f& cov = bestState->stationaryCov;
        bestState->movingCov = Matrix4f::Identity();
        bestState->movingCov.topLeftCorner(2, 2) << cov;
      }
      Matrix4f& cov = bestState->movingCov;
      cov(2, 2) += sqr(ballVelocity.x() * kickDeviation.x());
      cov(3, 3) += sqr(ballVelocity.y() * kickDeviation.y());
    }
    else
    {
      bestState->stationaryX = ballPosition;
      if(bestState->type != State::stationary)
      {
        bestState->type = State::stationary;
        Matrix4f& cov = bestState->movingCov;
        bestState->stationaryCov << cov.topLeftCorner(2, 2);
      }
    }
  }
}

void BallLocator::sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov)
{
  static const Matrix2x4f c = Matrix2x4f::Identity();
  static const Matrix4x2f cTransposed = c.transpose();

  COVARIANCE2D("module:BallLocator:Field", measurementCov, measurement);

  for(State* state = states, * end = states + stateCount; state < end; ++state)
    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Matrix4f& cov = state->movingCov;

      ASSERT(state->weight >= 0.f);
      state->gain = getUnscaledProbabilityAt(measurement, measurementCov, x.topRows(2));
      if(!(state->gain >= 0.f && state->gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << state->gain);
        OUTPUT_ERROR("measurement x=" << measurement.x() << " y=" << measurement.y());
        OUTPUT_ERROR("measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) << " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1));
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }
      ASSERT(state->gain >= 0.f && state->gain <= 10.f);
      state->weight *= state->gain;
      state->height = state->weight * getProbabilityAtMean(cov.topLeftCorner(2, 2));
      state->age++;

      Matrix2f covPlusSensorCov = c * cov * cTransposed;
      covPlusSensorCov += measurementCov;
      Matrix4x2f k = cov * cTransposed * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - c * x;
      Vector4f correction = k * innovation;
      x += correction;
      cov -= k * c * cov;
    }
    else // state->type == State::stationary
    {
      Vector2f& x = state->stationaryX;
      Matrix2f& cov = state->stationaryCov;

      ASSERT(state->weight >= 0.f);
      state->gain = getUnscaledProbabilityAt(measurement, measurementCov, x);

      if(!(state->gain >= 0.f && state->gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << state->gain);
        OUTPUT_ERROR("measurement x=" << measurement.x() << " y=" << measurement.y());
        OUTPUT_ERROR("measurementCov r0c0=" << measurementCov(0, 0) << " r0c1=" << measurementCov(0, 1) << " r1c0=" << measurementCov(1, 0) << " r1c1=" << measurementCov(1, 1));
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }

      ASSERT(state->gain >= 0.f && state->gain <= 10.f);
      state->weight *= state->gain;
      state->height = state->weight * getProbabilityAtMean(cov);
      state->age++;

      Matrix2f covPlusSensorCov = cov;
      covPlusSensorCov += measurementCov;
      Matrix2f k = cov * covPlusSensorCov.inverse();
      Vector2f innovation = measurement - x;
      Vector2f correction = k * innovation;
      x += correction;
      cov -= k * cov;
    }
}

void BallLocator::normalizeWeights(State*& bestState, State*& worstStationaryState, State*& worstMovingState)
{
  bestState = nullptr;
  worstStationaryState = 0;
  worstMovingState = 0;
  State*& worstState = worstStationaryState;
  State*& secondWorstState = worstMovingState;

  float highestHeight = 0;
  float highestWeight = 0;
  float worstGain = 0;
  float secondWorstGain = 0;

  for(State* state = states, * end = states + stateCount; state < end; ++state)
  {
    if(state->weight > highestWeight)
      highestWeight = state->weight;
    if(bestState == nullptr || state->height > highestHeight)
    {
      bestState = state;
      highestHeight = state->height;
    }
  }

  ASSERT(highestWeight > 0.f || stateCount == 0);

  for(State* state = states, * end = states + stateCount; state < end; ++state)
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

void BallLocator::createNewStates(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov, State* worstStationaryState, State* worstMovingState)
{
  // create new fixed state
  ASSERT(worstStationaryState || stateCount < sizeof(states) / sizeof(*states));
  State* newState = worstStationaryState;
  if(stateCount < sizeof(states) / sizeof(*states))
    newState = &states[stateCount++];
  ASSERT(newState);
  newState->type = State::stationary;
  ASSERT(stateCount > 0);
  newState->weight = initialStateWeight;
  newState->age = 2;
  newState->radius = ballPerceptRadius;
  newState->stationaryX = ballPercept;
  newState->stationaryCov = ballPerceptCov;

  // create new moving state
  if(hasLastBallPercept)
  {
    ASSERT(worstMovingState || stateCount < sizeof(states) / sizeof(*states));
    State* newState = worstMovingState;
    if(stateCount < sizeof(states) / sizeof(*states))
      newState = &states[stateCount++];
    ASSERT(newState);
    newState->type = State::moving;
    ASSERT(stateCount > 1);
    newState->weight = initialStateWeight;
    newState->age = 0;
    newState->radius = lastBallPerceptRadius;
    float timeScale = 1.f / ((theFrameInfo.time - lastBallPerceptTimeStamp) * 0.001f);
    newState->movingX << ballPercept, (ballPercept - lastBallPercept) * timeScale; // z
    newState->movingCov << ballPerceptCov, Matrix2f::Identity(),
             Matrix2f::Identity(), (ballPerceptCov + lastBallPerceptCov) * timeScale;
  }
}

void BallLocator::generateModel(BallModel& ballModel)
{
  DECLARE_PLOT("module:BallLocator:variance");
  if(bestState == nullptr && stateCount > 0)
    bestState = &states[0];
  if(bestState != nullptr)
  {
    BallState& ballState = ballModel.estimate;
    if(bestState->type == State::moving)
    {
      ballState.position = bestState->movingX.topRows(2);
      ballState.velocity = bestState->movingX.bottomRows(2);
      PLOT("module:BallLocator:variance", sqrt(bestState->movingCov(0, 0) + bestState->movingCov(1, 1)));
    }
    else
    {
      ballState.position = bestState->stationaryX;
      ballState.velocity = Vector2f::Zero();
      PLOT("module:BallLocator:variance", sqrt(bestState->stationaryCov(0, 0) + bestState->stationaryCov(1, 1)));
    }
    ballState.radius = bestState->radius;
  }
  if(ballWasSeenInThisFrame)
  {
    ballModel.timeWhenLastSeen = theFrameInfo.time;
    ballModel.lastPerception = theBallPercept.relativePositionOnField;
  }

  ASSERT(!std::isnan(ballModel.estimate.position.x()) && !std::isnan(ballModel.estimate.position.y()));

  // update timeWhenDisappeared
  {
    bool ballWasSeen = theBallModel.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0;
    bool ballShouldBeVisible = false;
    if(theCameraMatrix.isValid)
    {
      Vector3f ballRel;
      ballRel << theBallModel.estimate.position, theBallModel.estimate.radius;
      Vector2f ballInImage;
      if(Transformation::robotToImage(ballRel, theCameraMatrix, theCameraInfo, ballInImage))
      {
        const float ballDistance = (theCameraMatrix.inverse() * ballRel).norm();
        const float ballRadiusInImage = Geometry::getSizeByDistance(theCameraInfo, theBallModel.estimate.radius, ballDistance);
        const float offset = std::ceil(ballRadiusInImage);
        if(ballInImage.x() >= offset && ballInImage.x() < theCameraInfo.width - offset && ballInImage.y() >= offset && ballInImage.y() < theCameraInfo.height - offset)
          ballShouldBeVisible = true;
      }
    }

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

Matrix2f BallLocator::getCovOfPixelInWorld(const Vector2f& correctedPointInImage, float pointZInWorld) const
{
  const Vector3f unscaledVectorToPoint(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedPointInImage.x(), theCameraInfo.opticalCenter.y() - correctedPointInImage.y());
  const Vector3f unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z() - pointZInWorld;
  const float scale = h / -unscaledWorld.z();
  Vector2f pointInWorld(unscaledWorld.x() * scale, unscaledWorld.y() * scale);
  const float distance = pointInWorld.norm();
  Matrix2f rot;
  Vector2f cossin = pointInWorld * (1.f / distance);
  if(distance == 0.f)
    rot = Matrix2f::Identity();
  else
    rot << cossin.x(), -cossin.y(),
        cossin.y(), cossin.x();
  Matrix2f cov;
  cov << sqr(h / std::tan((distance == 0.f ? pi_2 : std::atan(h / distance)) - robotRotationDeviation.x()) - distance), 0.f,
      0.f, sqr(std::tan(robotRotationDeviation.y()) * distance);
  return rot * cov * rot.transpose();
}

float BallLocator::getUnscaledProbabilityAt(const Vector2f& mean, const Matrix2f& cov, const Vector2f& pos) const
{
  Vector2f diff = pos - mean;
  float exponent = diff.dot(cov.inverse() * diff);
  float p = std::exp(-0.5f * exponent);
  return std::max(p, 0.01f /*0.0000001f*/);
}

float BallLocator::getProbabilityAtMean(const Matrix2f& cov) const
{
  return 1.f / std::max((pi2 * std::sqrt(std::max(cov.determinant(), 0.f))), 0.0000001f);
}

bool BallLocator::getSmallestLineWithCircleIntersectionFactor(const Vector2f& lineBase, const Vector2f& lineDir, const Vector2f& circleBase, float circleRadius, float& factor) const
{
  const Vector2f& dir = lineDir;
  float a = lineDir.dot(lineDir);
  if(a == 0.f)
    return false;
  const Vector2f base = lineBase - circleBase;
  float b = 2.f * (dir.dot(base));
  float c = base.dot(base) - sqr(circleRadius);
  float ll = b * b - 4.f * a * c;
  if(ll < 0.f)
    return false;
  if(a > 0)
    factor = (-b - std::sqrt(ll)) / (2.f * a);
  else
    factor = (-b + std::sqrt(ll)) / (2.f * a);
  return true;
}

bool BallLocator::getLineWithLineIntersectionFactors(const Vector2f& lineBase1, const Vector2f& lineDir1, const Vector2f& lineBase2, const Vector2f& lineDir2, float& factor1, float& factor2) const
{
  float h = lineDir1.x() * lineDir2.y() - lineDir1.y() * lineDir2.x();
  if(h == 0.f)
    return false;
  factor2 = ((lineBase2.x() - lineBase1.x()) * lineDir1.y() - (lineBase2.y() - lineBase1.y()) * lineDir1.x()) / h;
  factor1 = ((lineBase1.y() - lineBase2.y()) * lineDir2.x() - (lineBase1.x() - lineBase2.x()) * lineDir2.y()) / h;
  return true;
}
