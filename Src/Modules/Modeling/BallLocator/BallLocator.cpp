/**
* @file BallLocator.cpp
* Implementation of the BallLocator module.
* @author Colin Graf
*/

#include <algorithm>
#include "BallLocator.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(BallLocator, Modeling)

BallLocator::BallLocator() : timeNotSeen(100000), firstDisappearance(0), ballWasBeenSeenInLastLowerCameraImage(false)
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
  bestState = 0;
  hasLastBallPercept = false;
}

void BallLocator::update(BallModel& ballModel)
{
  DECLARE_DEBUG_DRAWING("module:BallLocator:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallLocator:Image", "drawingOnImage");

#ifdef TARGET_SIM
  if(theFrameInfo.time <= lastFrameTime)
  {
    if(theFrameInfo.time < lastFrameTime)
      init();
    else
      return;
  }
#endif

  // set extra members for seen ball to achieve the following behavior:
  //    if the ball has been seen in the last lower camera image, it will be ignored in
  //    the upper image to exclude any clutter in the robot's environment
  ballWasSeenInThisFrame = theBallPercept.ballWasSeen;
  if(ballWasSeenInThisFrame && theCameraInfo.camera == CameraInfo::upper && ballWasBeenSeenInLastLowerCameraImage)
    ballWasSeenInThisFrame = false;
  else if(theCameraInfo.camera == CameraInfo::lower)
    ballWasBeenSeenInLastLowerCameraImage = ballWasSeenInThisFrame;

  // perform prediction step for each filter
  motionUpdate(ballModel);

  // detect and handle collision with feet
  Pose3D leftFoot = theTorsoMatrix;
  Pose3D rightFoot = theTorsoMatrix;
  leftFoot.conc(theRobotModel.limbs[MassCalibration::footLeft]).translate(footOffset.x, footOffset.y, -theRobotDimensions.heightLeg5Joint);
  rightFoot.conc(theRobotModel.limbs[MassCalibration::footRight]).translate(footOffset.x, -footOffset.y, -theRobotDimensions.heightLeg5Joint);
  Vector2<> leftFootCenter(leftFoot.translation.x, leftFoot.translation.y);
  Vector2<> rightFootCenter(rightFoot.translation.x, rightFoot.translation.y);
  COMPLEX_DRAWING("module:BallLocator:Image",
  {
    Vector2<int> leftCenterImage, leftTopImage, rightCenterImage, rightTopImage;
    if(Geometry::calculatePointInImage(Vector3<>(leftFootCenter.x, leftFootCenter.y, 0), theCameraMatrix, theCameraInfo, leftCenterImage) &&
    Geometry::calculatePointInImage(Vector3<>(leftFootCenter.x + footRadius, leftFootCenter.y, 0), theCameraMatrix, theCameraInfo, leftTopImage))
    {
      CIRCLE("module:BallLocator:Image", leftCenterImage.x, leftCenterImage.y, (leftCenterImage - leftTopImage).abs(), 0, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_null, ColorRGBA());
    }
    if(Geometry::calculatePointInImage(Vector3<>(rightFootCenter.x, rightFootCenter.y, 0), theCameraMatrix, theCameraInfo, rightCenterImage) &&
    Geometry::calculatePointInImage(Vector3<>(rightFootCenter.x + footRadius, rightFootCenter.y, 0), theCameraMatrix, theCameraInfo, rightTopImage))
    {
      CIRCLE("module:BallLocator:Image", rightCenterImage.x, rightCenterImage.y, (rightCenterImage - rightTopImage).abs(), 0, Drawings::ps_solid, ColorRGBA(0, 0, 0), Drawings::bs_null, ColorRGBA());
    }
  });
  if(bestState)
    handleCollisionWithFeet(leftFootCenter, rightFootCenter);

  // sensor update step
  const Vector2f ballPercept(theBallPercept.relativePositionOnField.x, theBallPercept.relativePositionOnField.y);
  Matrix2x2f ballPerceptCov;
  if(ballWasSeenInThisFrame)
  {
    // reset filters..
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared) >= 4000)
      reset();

    // calculate variance of the percept
    ballPerceptCov = getCovOfPixelInWorld(theImageCoordinateSystem.toCorrected(theBallPercept.positionInImage), theFieldDimensions.ballRadius);

    // add current measurement to all filters
    sensorUpdate(ballPercept, ballPerceptCov);

    // normalize the weights and find best and worst filters
    State* worstStationaryState, * worstMovingState;
    normalizeWeights(bestState, worstStationaryState, worstMovingState);

    // create new filters
    createNewStates(ballPercept, ballPerceptCov, worstStationaryState, worstMovingState);
  }

  //
  COMPLEX_DRAWING("module:BallLocator:Field",
  {
    for(State* state = states, * end = states + stateCount; state < end; ++state)
      if(state->type == State::moving)
      {
        DOT("module:BallLocator:Field", state->movingX[0], state->movingX[1], ColorRGBA(0xff, 0xaa, 0xaa), ColorRGBA(0xff, 0xaa, 0xaa));
      }
      else // state->type == State::stationary
      {
        DOT("module:BallLocator:Field", state->stationaryX.x, state->stationaryX.y, ColorRGBA(0xaa, 0xff, 0xaa), ColorRGBA(0xaa, 0xff, 0xaa));
      }
  });

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
    lastBallPerceptTimeStamp = theFrameInfo.time;
    lastBallPerceptCov = ballPerceptCov;
    hasLastBallPercept = true;
  }

  EXECUTE_ONLY_IN_DEBUG(ballModel.drawEndPosition(theFieldDimensions.ballFriction););
}

void BallLocator::motionUpdate(BallModel& ballModel)
{
  // prepare odometry transformation
  Pose2D odometryOffset = theOdometryData - lastOdometryData;
  deltaTime = (theFrameInfo.time - lastFrameTime) * 0.001f; // in seconds
  float odometryCos = std::cos(odometryOffset.rotation);
  float odometrySin = std::sin(odometryOffset.rotation);
  float odometryRotationDeviation = odometryOffset.rotation * odometryDeviation.rotation;
  float odometryDeviationCos = std::cos(odometryRotationDeviation);
  float odometryDeviationSin = std::sin(odometryRotationDeviation);
  Vector2f odometryTranslationDeviation(odometryOffset.translation.x * odometryDeviation.translation.x, odometryOffset.translation.y * odometryDeviation.translation.y);
  Vector4f odometryTranslationCov(sqr(odometryTranslationDeviation.x), sqr(odometryTranslationDeviation.y), 0.f, 0.f);

  // prepare matrices and vectors for stationary filters
  Matrix2x2f fixedOdometryRotation(Vector2f(odometryCos, -odometrySin), Vector2f(odometrySin, odometryCos)); // a
  Matrix2x2f fixedOdometryRotationTransposed = fixedOdometryRotation.transpose();
  Matrix2x2f fixedOdometryRotationDeviationRotation(Vector2f(odometryDeviationCos, -odometryDeviationSin), Vector2f(odometryDeviationSin, odometryDeviationCos));
  Vector2f fixedOdometryTranslation(-odometryOffset.translation.x, -odometryOffset.translation.y); // u

  // prepare matrices and vectors for moving filters
  Matrix4x4f movingOdometryRotation(Vector4f(odometryCos, -odometrySin, 0, 0), Vector4f(odometrySin, odometryCos, 0, 0),
                                    Vector4f(0, 0, odometryCos, -odometrySin), Vector4f(0, 0, odometrySin, odometryCos)); // second part of "a"
  Matrix4x4f movingOdometryRotationDeviationRotation(Vector4f(odometryDeviationCos, -odometryDeviationSin, 0, 0), Vector4f(odometryDeviationSin, odometryDeviationCos, 0, 0),
      Vector4f(0, 0, odometryDeviationCos, -odometryDeviationSin), Vector4f(0, 0, odometryDeviationSin, odometryDeviationCos));
  Vector4f movingOdometryTranslation(-odometryOffset.translation.x, -odometryOffset.translation.y, 0, 0); // u
  Matrix4x4f movingMotionMatrix(Vector4f(1, 0, 0, 0), Vector4f(0, 1, 0, 0),
                                Vector4f(deltaTime, 0, 1, 0), Vector4f(0, deltaTime, 0, 1)); // first part of "a"
  Matrix4x4f movingA = movingOdometryRotation * movingMotionMatrix;
  Matrix4x4f movingATransponsed = movingA.transpose();

  // prepare process noise
  Vector4f processCov(sqr(processDeviation[0]), sqr(processDeviation[1]), sqr(processDeviation[2]), sqr(processDeviation[3]));

  // add odometry offset to the last percept
  if(hasLastBallPercept)
  {
    lastBallPercept = fixedOdometryRotation * lastBallPercept + fixedOdometryTranslation;
    lastBallPerceptCov[0][0] += odometryTranslationCov[0];
    lastBallPerceptCov[1][1] += odometryTranslationCov[1];
    Vector2f odometryRotationDeviationTranslation = fixedOdometryRotationDeviationRotation * lastBallPercept - lastBallPercept;
    lastBallPerceptCov[0][0] += sqr(odometryRotationDeviationTranslation.x);
    lastBallPerceptCov[1][1] += sqr(odometryRotationDeviationTranslation.y);
  }

  // add odometry offset to lastPercept and lastSeenEstimate
  if(!ballWasSeenInThisFrame)
  {
    Vector2f lastPercept = fixedOdometryRotation * Vector2f(ballModel.lastPerception.x, ballModel.lastPerception.y) + fixedOdometryTranslation;
    ballModel.lastPerception = Vector2<>(lastPercept.x, lastPercept.y);
  }

  // perform prediction step for each state
  for(State* state = states, * end = states + stateCount; state < end; ++state)
  {
    // apply friction
    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Vector2f velocity(x[2], x[3]);
      velocity *= (1+deltaTime*theFieldDimensions.ballFriction);
      x[2] = velocity.x;
      x[3] = velocity.y;
      if(velocity.abs() <100)
      {
        // the estimate stopped moving
        state->type = State::stationary;
        state->stationaryX = Vector2f(x[0], x[1]);
        Matrix4x4f& cov = state->movingCov;
        state->stationaryCov = Matrix2x2f(Vector2f(cov[0][0], cov[0][1]), Vector2f(cov[1][0], cov[1][1]));
      }
    }

    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Matrix4x4f& cov = state->movingCov;

      // predict
      x = movingA * x + movingOdometryTranslation;
      cov = movingA * cov * movingATransponsed;

      // add process noise
      for(int i = 0; i < 4; ++i)
        cov[i][i] += processCov[i];

      // add odometry translation noise
      for(int i = 0; i < 4; ++i)
        cov[i][i] += odometryTranslationCov[i];

      // add noise from odometry rotation (curde approximation)
      Vector4f odometryRotationDeviationTranslation = movingOdometryRotationDeviationRotation * x - x;
      for(int i = 0; i < 4; ++i)
        cov[i][i] += sqr(odometryRotationDeviationTranslation[i]);
    }
    else // state->type == State::stationary
    {
      Vector2f& x = state->stationaryX;
      Matrix2x2f& cov = state->stationaryCov;

      // predict
      x = fixedOdometryRotation * x + fixedOdometryTranslation;
      cov = fixedOdometryRotation * cov * fixedOdometryRotationTransposed;

      // add process noise
      cov[0][0] += processCov[0];
      cov[1][1] += processCov[1];

      // add odometry translation noise
      cov[0][0] += odometryTranslationCov[0];
      cov[1][1] += odometryTranslationCov[1];

      // add noise from odometry rotation (curde approximation)
      Vector2f odometryRotationDeviationTranslation = fixedOdometryRotationDeviationRotation * x - x;
      cov[0][0] += sqr(odometryRotationDeviationTranslation.x);
      cov[1][1] += sqr(odometryRotationDeviationTranslation.y);
    }
  }
}

void BallLocator::handleCollisionWithFeet(const Vector2<>& leftFootCenter, const Vector2<>& rightFootCenter)
{
  ASSERT(bestState);
  ASSERT(leftFootCenter != Vector2<>() && rightFootCenter != Vector2<>());

  Vector2<> ballPosition(bestState->stationaryX.x, bestState->stationaryX.y);
  Vector2<> ballVelocity;
  if(bestState->type == State::moving)
  {
    ballPosition = Vector2<>(bestState->movingX[0], bestState->movingX[1]);
    ballVelocity = Vector2<>(bestState->movingX[2], bestState->movingX[3]);
  }

  bool leftCollision = false, rightCollision = false, centerCollision = false;
  float leftCollisionFactor, rightCollisionFactor;
  float assumedRadius = footRadius + theFieldDimensions.ballRadius;

  // detect collision with the left foot
  Vector2<> leftAssumedLastBallPosition = lastBallPosition + (leftFootCenter - lastLeftFootCenter);
  Vector2<> leftAssumedBallOffset = ballPosition - leftAssumedLastBallPosition;
  leftCollision = getSmallestLineWithCircleIntersectionFactor(leftAssumedLastBallPosition, leftAssumedBallOffset, lastLeftFootCenter, assumedRadius, leftCollisionFactor);
  if(leftCollision && (leftCollisionFactor < 0.f || leftCollisionFactor > 1.f))
    leftCollision = false;

  // detect collision with the right foot
  Vector2<> rightAssumedLastBallPosition = lastBallPosition + (rightFootCenter - lastRightFootCenter);
  Vector2<> rightAssumedBallOffset = ballPosition - rightAssumedLastBallPosition;
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
    const Vector2<>& footCenter = leftCollision ? leftFootCenter : rightFootCenter;
    const Vector2<>& lastFootCenter = leftCollision ? lastLeftFootCenter : lastRightFootCenter;
    Vector2<> collisionPoint = leftCollision ? (leftAssumedLastBallPosition + leftAssumedBallOffset * leftCollisionFactor) : (rightAssumedLastBallPosition + rightAssumedBallOffset * rightCollisionFactor);
    float collisionAngle = (collisionPoint - lastFootCenter).angle();

    ballPosition = collisionPoint;
    ballVelocity = Vector2<>();

    float footMomentum = (footCenter - lastFootCenter).abs() / deltaTime * footMass;
    float passedMomentum = footMomentum * std::cos(std::abs(normalize(collisionAngle - (footCenter - lastFootCenter).angle())));
    // TODO: calculate reflected momentum?
    if(passedMomentum > 0.f)
    {
      ballVelocity = (collisionPoint - lastFootCenter).normalize(passedMomentum) / ballMass;
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
        ballVelocity = Vector2<>();
      }
  }

  // it is still possible that the ball is within the foot circles
  // if so, calculate a ball shift vector
  Vector2<> ballShift;
  if((ballPosition - leftFootCenter).squareAbs() < sqr(assumedRadius))
    ballShift += (ballPosition - leftFootCenter).normalize(assumedRadius - (ballPosition - leftFootCenter).abs());
  if((ballPosition - rightFootCenter).squareAbs() < sqr(assumedRadius))
    ballShift += (ballPosition - rightFootCenter).normalize(assumedRadius - (ballPosition - rightFootCenter).abs());
  ballPosition += ballShift;

  // apply new position and velocity
  if(leftCollision || rightCollision || centerCollision || ballShift != Vector2<>())
  {
    if(ballVelocity != Vector2<>())
    {
      bestState->movingX = Vector4f(ballPosition.x, ballPosition.y, ballVelocity.x, ballVelocity.y);
      if(bestState->type != State::moving)
      {
        bestState->type = State::moving;
        Matrix2x2f& cov = bestState->stationaryCov;
        bestState->movingCov = Matrix4x4f(Vector4f(cov[0][1], cov[0][1], 0, 0), Vector4f(cov[1][0], cov[1][1], 0, 0), Vector4f(), Vector4f());
      }
      Matrix4x4f& cov = bestState->movingCov;
      cov[2][2] += sqr(ballVelocity.x * kickDeviation.x);
      cov[3][3] += sqr(ballVelocity.y * kickDeviation.y);
    }
    else
    {
      bestState->stationaryX = Vector2f(ballPosition.x, ballPosition.y);
      if(bestState->type != State::stationary)
      {
        bestState->type = State::stationary;
        Matrix4x4f& cov = bestState->movingCov;
        bestState->stationaryCov = Matrix2x2f(Vector2f(cov[0][0], cov[0][1]), Vector2f(cov[1][0], cov[1][1]));
      }
    }
  }
}

void BallLocator::sensorUpdate(const Vector2f& measurement, const Matrix2x2f& measurementCov)
{
  static const Matrix2x4f c(Vector2f(1, 0), Vector2f(0, 1), Vector2f(), Vector2f());
  static const Matrix4x2f cTransposed = c.transpose();

  COVARIANCE2D("module:BallLocator:Field", measurementCov, measurement);

  for(State* state = states, * end = states + stateCount; state < end; ++state)
    if(state->type == State::moving)
    {
      Vector4f& x = state->movingX;
      Matrix4x4f& cov = state->movingCov;

      ASSERT(state->weight >= 0.f);
      state->gain = getUnscaledProbabilityAt(measurement, measurementCov, Vector2f(x[0], x[1]));
      if(!(state->gain >= 0.f && state->gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << state->gain);
        OUTPUT_ERROR("measurement x=" << measurement.x << " y=" << measurement.y);
        OUTPUT_ERROR("measurementCov c0x=" << measurementCov.c[0].x << " c0y=" << measurementCov.c[0].y << " c1x=" << measurementCov.c[1].x << " c1y=" << measurementCov.c[1].y);
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }
      ASSERT(state->gain >= 0.f && state->gain <= 10.f);
      state->weight *= state->gain;
      state->height = state->weight * getProbabilityAtMean(Matrix2x2f(Vector2f(cov[0][0], cov[0][1]), Vector2f(cov[1][0], cov[1][1])));
      state->age++;

      Matrix2x2f covPlusSensorCov = c * cov * cTransposed;
      covPlusSensorCov += measurementCov;
      Matrix4x2f k = cov * cTransposed * covPlusSensorCov.invert();
      Vector2f innovation = measurement - c * x;
      Vector4f correction = k * innovation;
      x += correction;
      cov -= k * c * cov;
    }
    else // state->type == State::stationary
    {
      Vector2f& x = state->stationaryX;
      Matrix2x2f& cov = state->stationaryCov;

      ASSERT(state->weight >= 0.f);
      state->gain = getUnscaledProbabilityAt(measurement, measurementCov, x);

      if(!(state->gain >= 0.f && state->gain <= 10.f))
      {
        OUTPUT_ERROR("getUnscaledProbabilityAt: gain=" << state->gain);
        OUTPUT_ERROR("measurement x=" << measurement.x << " y=" << measurement.y);
        OUTPUT_ERROR("measurementCov c0x=" << measurementCov.c[0].x << " c0y=" << measurementCov.c[0].y << " c1x=" << measurementCov.c[1].x << " c1y=" << measurementCov.c[1].y);
        OUTPUT_ERROR("x x.x=" << x[0] << " x.y=" << x[1]);
      }

      ASSERT(state->gain >= 0.f && state->gain <= 10.f);
      state->weight *= state->gain;
      state->height = state->weight * getProbabilityAtMean(cov);
      state->age++;

      Matrix2x2f covPlusSensorCov = cov;
      covPlusSensorCov += measurementCov;
      Matrix2x2f k = cov * covPlusSensorCov.invert();
      Vector2f innovation = measurement - x;
      Vector2f correction = k * innovation;
      x += correction;
      cov -= k * cov;
    }
}

void BallLocator::normalizeWeights(State*& bestState, State*& worstStationaryState, State*& worstMovingState)
{
  bestState = 0;
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
    if(!bestState || state->height > highestHeight)
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

void BallLocator::createNewStates(const Vector2f& ballPercept, const Matrix2x2f& ballPerceptCov, State* worstStationaryState, State* worstMovingState)
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
    float timeScale = 1.f / ((theFrameInfo.time - lastBallPerceptTimeStamp) * 0.001f);
    newState->movingX = Vector4f(ballPercept.x, ballPercept.y,
                                 (ballPercept.x - lastBallPercept.x) * timeScale, (ballPercept.y - lastBallPercept.y) * timeScale); // z
    newState->movingCov = Matrix4x4f(Vector4f(ballPerceptCov[0][0], ballPerceptCov[0][1], 0, 0), Vector4f(ballPerceptCov[1][0], ballPerceptCov[1][1], 0, 0),
                                     Vector4f(0, 0, (ballPerceptCov[0][0] + lastBallPerceptCov[0][0]) * timeScale, (ballPerceptCov[0][1] + lastBallPerceptCov[0][1]) * timeScale),
                                     Vector4f(0, 0, (ballPerceptCov[1][0] + lastBallPerceptCov[1][0]) * timeScale, (ballPerceptCov[1][1] + lastBallPerceptCov[1][1]) * timeScale));
  }
}

void BallLocator::generateModel(BallModel& ballModel)
{
  DECLARE_PLOT("module:BallLocator:variance");
  if(!bestState && stateCount > 0)
    bestState = &states[0];
  if(bestState)
  {
    BallState& ballState = ballModel.estimate;
    if(bestState->type == State::moving)
    {
      ballState.position = Vector2<>(bestState->movingX[0], bestState->movingX[1]);
      ballState.velocity = Vector2<>(bestState->movingX[2], bestState->movingX[3]);
      ballState.positionCovariance[0][0] = bestState->movingCov[0][0];
      ballState.positionCovariance[1][0] = bestState->movingCov[1][0];
      ballState.positionCovariance[0][1] = bestState->movingCov[0][1];
      ballState.positionCovariance[1][1] = bestState->movingCov[1][1];
      PLOT("module:BallLocator:variance", sqrt(bestState->movingCov[0][0] + bestState->movingCov[1][1]));
    }
    else
    {
      ballState.position = Vector2<>(bestState->stationaryX.x, bestState->stationaryX.y);
      ballState.velocity = Vector2<>();
      ballState.positionCovariance[0][0] = bestState->stationaryCov[0][0];
      ballState.positionCovariance[1][0] = bestState->stationaryCov[1][0];
      ballState.positionCovariance[0][1] = bestState->stationaryCov[0][1];
      ballState.positionCovariance[1][1] = bestState->stationaryCov[1][1];
      PLOT("module:BallLocator:variance", sqrt(bestState->stationaryCov[0][0] + bestState->stationaryCov[1][1]));
    }
  }
  if(ballWasSeenInThisFrame)
  {
    ballModel.timeWhenLastSeen = theFrameInfo.time;
    ballModel.lastPerception = theBallPercept.relativePositionOnField;
  }

  ASSERT(!isnan(ballModel.estimate.position.x) && !isnan(ballModel.estimate.position.y));

  // update timeWhenDisappeared
  {
    bool ballWasSeen = theBallModel.timeWhenLastSeen == theFrameInfo.time;
    bool ballShouldBeVisible = false;
    {
      const Vector3<> camera = theCameraMatrix.invert() * Vector3<>(theBallModel.estimate.position.x, theBallModel.estimate.position.y, theFieldDimensions.ballRadius);
      if(camera.x > 1)
      {
        const float scale = -theCameraInfo.focalLength / camera.x;
        Vector2<> image(theCameraInfo.opticalCenter.x + scale * camera.y, theCameraInfo.opticalCenter.y + scale * camera.z);
        image = theImageCoordinateSystem.fromCorrectedApprox(image);
        if((image - Vector2<>(theCameraInfo.opticalCenter.x, theCameraInfo.opticalCenter.y)).squareAbs() < sqr(theCameraInfo.height * 0.42f))
          ballShouldBeVisible = true;
      }
    }

    if(ballWasSeen)
    {
      timeNotSeen = 0;
      firstDisappearance = 0;
    }
    else if(ballShouldBeVisible)
    {
      timeNotSeen += (unsigned) (theFrameInfo.cycleTime * 1000.f);
      if(firstDisappearance == 0)
        firstDisappearance = theFrameInfo.time - timeNotSeen;
    }

    if(timeNotSeen > ballNotSeenTimeout)
      ballModel.timeWhenDisappeared = firstDisappearance;
    else
      ballModel.timeWhenDisappeared = theFrameInfo.time - timeNotSeen;

    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) >= (int) ballNotUpdatedTimeout)
      ballModel.timeWhenDisappeared = theBallModel.timeWhenLastSeen;
  }
}

Matrix2x2f BallLocator::getCovOfPixelInWorld(const Vector2<>& correctedPointInImage, float pointZInWorld) const
{
  const Vector3<> unscaledVectorToPoint(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedPointInImage.x, theCameraInfo.opticalCenter.y - correctedPointInImage.y);
  const Vector3<> unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z - pointZInWorld;
  const float scale = h / -unscaledWorld.z;
  Vector2f pointInWorld(unscaledWorld.x * scale, unscaledWorld.y * scale);
  const float distance = pointInWorld.abs();
  Vector2f cossin = distance == 0.f ? Vector2f(1.f, 0.f) : pointInWorld * (1.f / distance);
  Matrix2x2f rot(cossin, Vector2f(-cossin.y, cossin.x));
  Matrix2x2f cov(Vector2f(sqr(h / std::tan((distance == 0.f ? pi_2 : std::atan(h / distance)) - robotRotationDeviation.x) - distance), 0.f),
                 Vector2f(0.f, sqr(std::tan(robotRotationDeviation.y) * distance)));
  return rot * cov * rot.transpose();
}

float BallLocator::getUnscaledProbabilityAt(const Vector2f& mean, const Matrix2x2f& cov, const Vector2f& pos) const
{
  Vector2f diff = pos - mean;
  float exponent = diff * (cov.invert() * diff);
  float p = std::exp(-0.5f * exponent);
  return std::max(p, 0.01f /*0.0000001f*/);
}

float BallLocator::getProbabilityAtMean(const Matrix2x2f& cov) const
{
  return 1.f / std::max((pi2 * std::sqrt(std::max(cov.det(), 0.f))), 0.0000001f);
}

bool BallLocator::getSmallestLineWithCircleIntersectionFactor(const Vector2<>& lineBase, const Vector2<>& lineDir, const Vector2<>& circleBase, float circleRadius, float& factor) const
{
  const Vector2<>& dir = lineDir;
  float a = dir.x * dir.x + dir.y * dir.y;
  if(a == 0.f)
    return false;
  const Vector2<> base = lineBase - circleBase;
  float b = 2.f * (dir.x * base.x + dir.y * base.y);
  float c = base.x * base.x + base.y * base.y - circleRadius * circleRadius;
  float ll = b * b - 4.f * a * c;
  if(ll < 0.f)
    return false;
  if(a > 0)
    factor = (-b - std::sqrt(ll)) / (2.f * a);
  else
    factor = (-b + std::sqrt(ll)) / (2.f * a);
  return true;
}

bool BallLocator::getLineWithLineIntersectionFactors(const Vector2<>& lineBase1, const Vector2<>& lineDir1, const Vector2<>& lineBase2, const Vector2<>& lineDir2, float& factor1, float& factor2) const
{
  float h = lineDir1.x * lineDir2.y - lineDir1.y * lineDir2.x;
  if(h == 0.f)
    return false;
  factor2 = ((lineBase2.x - lineBase1.x) * lineDir1.y - (lineBase2.y - lineBase1.y) * lineDir1.x) / h;
  factor1 = ((lineBase1.y - lineBase2.y) * lineDir2.x - (lineBase1.x - lineBase2.x) * lineDir2.y) / h;
  return true;
}
