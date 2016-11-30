/**
 * @file ExpBallLocator.cpp
 * Implementation of the BallLocator module.
 * @author Colin Graf
 */

#include <algorithm>
#include "ExpBallLocator.h"
#include "BallLocatorTools.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(ExpBallLocator, modeling)

ExpBallLocator::ExpBallLocator() : timeBallWasBeenSeenInLowerCameraImage(0), timeWhenBallFirstDisappeared(0), ballDisappeared(true)
{
  init();
}

void ExpBallLocator::init()
{
  lastOdometryData = theOdometryData;
  lastFrameTime = theFrameInfo.time;
  reset();
}

void ExpBallLocator::reset()
{
  stateCount = 0;
  bestState = nullptr;
  hasLastBallPercept = false;
}

void ExpBallLocator::update(BallModel& ballModel)
{
  DECLARE_DEBUG_DRAWING("module:BallLocator:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:BallLocator:image", "drawingOnImage");

  if(SystemCall::getMode() == SystemCall::logfileReplay &&
     theFrameInfo.time <= lastFrameTime)
  {
    if(theFrameInfo.time < lastFrameTime)
      init();
    else
      return;
  }

  // we accept guessed balls, if the ball is currently assumed to be rolling and the percept is close to the propagated estimate
  bool ballIsInImage = theBallPercept.status == BallPercept::seen ||
                      (theBallPercept.status == BallPercept::guessed && ballModel.estimate.velocity.norm() != 0.f &&
                       theBallPrediction.isValid &&
                       (theBallPrediction.position - theBallPercept.positionOnField).norm() < guessedRollingBallAcceptanceDistance);
  
  // set extra members for seen ball to achieve the following behavior:
  //    if the ball has been seen recently in the lower camera image, it will be ignored in
  //    the upper image to exclude any clutter in the robot's environment
  ballWasSeenInThisFrame = ballIsInImage && !ballCanBeExcludedByLocalization() && !ballIsAtBorderAndCloseToTeammate();

  // Check staging stuff
  if(ballWasSeenInThisFrame)
  {
    float stagingDistance = stagingDistanceNear;
    unsigned stagingCount = stagingCountNear;
    if(theBallPercept.positionOnField.norm() > stagingFarBegin)
    {
      stagingDistance = stagingDistanceFar;
      stagingCount = stagingCountFar;
    }
    if(stagedBalls.size() < stagingCount)
    {
      ballWasSeenInThisFrame = false;
    }
    else
    {
      for(unsigned int i=0; i<stagingCount; i++)
      {
        StagedBall& b = stagedBalls[i];
        if(theFrameInfo.getTimeSince(b.timestamp) > stagingTimeout ||
           ((b.position - theBallPercept.positionOnField).norm() > stagingDistance &&
             theBallModel.estimate.velocity.norm() <= 0.1f))
        {
          ballWasSeenInThisFrame = false;
          break;
        }
      }
    }
    stagedBalls.push_front(StagedBall(theBallPercept.positionOnField, theFrameInfo.time));
  }

  seenStats.push_front(ballWasSeenInThisFrame ? 100 : 0);
  ballModel.seenPercentage = (unsigned char) seenStats.average();

  if(ballWasSeenInThisFrame && theCameraInfo.camera == CameraInfo::upper &&
     theFrameInfo.getTimeSince(timeBallWasBeenSeenInLowerCameraImage) < lowerCameraImageBallTimeout)
    ballWasSeenInThisFrame = false;
  else if(theCameraInfo.camera == CameraInfo::lower && ballWasSeenInThisFrame)
    timeBallWasBeenSeenInLowerCameraImage = theFrameInfo.time;

  // perform prediction step for each filter
  motionUpdate(ballModel);

  // detect and handle collision with feet
  const Vector3f rightFootOffset(leftFootOffset.x(), -leftFootOffset.y(), leftFootOffset.z());
  const Pose3f leftFoot = theTorsoMatrix * (theRobotModel.soleLeft + leftFootOffset);
  const Pose3f rightFoot = theTorsoMatrix * (theRobotModel.soleRight + rightFootOffset);
  Vector2f leftFootCenter = leftFoot.translation.topRows(2);
  Vector2f rightFootCenter = rightFoot.translation.topRows(2);
  COMPLEX_DRAWING("module:BallLocator:image")
  {
    Vector2f leftCenterImage, leftTopImage, rightCenterImage, rightTopImage;
    if(Transformation::robotToImage(leftFootCenter, theCameraMatrix, theCameraInfo, leftCenterImage) &&
       Transformation::robotToImage(Vector2f(leftFootCenter.x() + footRadius, leftFootCenter.y()), theCameraMatrix, theCameraInfo, leftTopImage))
    {
      CIRCLE("module:BallLocator:image", leftCenterImage.x(), leftCenterImage.y(), (leftCenterImage - leftTopImage).norm(), 0, Drawings::solidPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA());
    }
    if(Transformation::robotToImage(rightFootCenter, theCameraMatrix, theCameraInfo, rightCenterImage) &&
       Transformation::robotToImage(Vector2f(rightFootCenter.x() + footRadius, rightFootCenter.y()), theCameraMatrix, theCameraInfo, rightTopImage))
    {
      CIRCLE("module:BallLocator:image", rightCenterImage.x(), rightCenterImage.y(), (rightCenterImage - rightTopImage).norm(), 0, Drawings::solidPen, ColorRGBA::black, Drawings::noBrush, ColorRGBA());
    }
  }
  if(bestState != nullptr)
    handleCollisionWithFeet(leftFootCenter, rightFootCenter);

  // sensor update step
  const Vector2f& ballPercept = theBallPercept.positionOnField;
  Matrix2f ballPerceptCov;
  if(ballWasSeenInThisFrame)
  {
    // reset filters..
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) >= 4000)
      reset();

    // calculate variance of the percept
    ballPerceptCov = BallLocatorTools::getCovOfPixelInWorld(theImageCoordinateSystem.toCorrected(theBallPercept.positionInImage),
                                                            theBallPercept.radiusOnField,
                                                            theCameraInfo, theCameraMatrix, robotRotationDeviation);

    // add current measurement to all filters
    sensorUpdate(ballPercept, ballPerceptCov);

    // normalize the weights and find best and worst filters
    BallHypothesis* worstStationaryState, * worstMovingState;
    normalizeWeights(bestState, worstStationaryState, worstMovingState);

    // create new filters
    createNewStates(ballPercept, theBallPercept.radiusOnField, ballPerceptCov, worstStationaryState, worstMovingState);
  }

  // visualize the states
  COMPLEX_DRAWING("module:BallLocator:field")
  {
    for(BallHypothesis* state = states, * end = states + stateCount; state < end; ++state)
      if(state->type == BallHypothesis::moving)
      {
        DOT("module:BallLocator:field", state->movingX[0], state->movingX[1], ColorRGBA(0xff, 0xaa, 0xaa), ColorRGBA(0xff, 0xaa, 0xaa));
      }
      else // state->type == State::stationary
      {
        DOT("module:BallLocator:field", state->stationaryX.x(), state->stationaryX.y(), ColorRGBA(0xaa, 0xff, 0xaa), ColorRGBA(0xaa, 0xff, 0xaa));
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
    lastBallPerceptTimeStamp = theFrameInfo.time;
    lastBallPerceptCov = ballPerceptCov;
    hasLastBallPercept = true;
  }
}

void ExpBallLocator::motionUpdate(BallModel& ballModel)
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
  for(BallHypothesis* state = states, * end = states + stateCount; state < end; ++state)
  {
    state->motionUpdate(ballModel, movingA, movingATransposed, movingOdometryTranslation,
                        squaredProcessCov, odometryTranslationCov, movingOdometryRotationDeviationRotation,
                        fixedOdometryRotation, fixedOdometryTranslation, fixedOdometryRotationTransposed,
                        fixedOdometryRotationDeviationRotation, theFieldDimensions, deltaTime);
  }
}

void ExpBallLocator::handleCollisionWithFeet(const Vector2f& leftFootCenter, const Vector2f& rightFootCenter)
{
  ASSERT(bestState != nullptr);
  ASSERT(leftFootCenter != Vector2f::Zero() && rightFootCenter != Vector2f::Zero());

  Vector2f ballPosition = bestState->stationaryX;
  Vector2f ballVelocity = Vector2f::Zero();
  if(bestState->type == BallHypothesis::moving)
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
  leftCollision = BallLocatorTools::getSmallestLineWithCircleIntersectionFactor(leftAssumedLastBallPosition, leftAssumedBallOffset, lastLeftFootCenter, assumedRadius, leftCollisionFactor);
  if(leftCollision && (leftCollisionFactor < 0.f || leftCollisionFactor > 1.f))
    leftCollision = false;

  // detect collision with the right foot
  Vector2f rightAssumedLastBallPosition = lastBallPosition + (rightFootCenter - lastRightFootCenter);
  Vector2f rightAssumedBallOffset = ballPosition - rightAssumedLastBallPosition;
  rightCollision = BallLocatorTools::getSmallestLineWithCircleIntersectionFactor(rightAssumedLastBallPosition, rightAssumedBallOffset, lastRightFootCenter, assumedRadius, rightCollisionFactor);
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
    if(BallLocatorTools::getLineWithLineIntersectionFactors(leftFootCenter, rightFootCenter - leftFootCenter, lastBallPosition, ballPosition - lastBallPosition, factor1, factor2))
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
      if(bestState->type != BallHypothesis::moving)
      {
        bestState->type = BallHypothesis::moving;
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
      if(bestState->type != BallHypothesis::stationary)
      {
        bestState->type = BallHypothesis::stationary;
        Matrix4f& cov = bestState->movingCov;
        bestState->stationaryCov << cov.topLeftCorner(2, 2);
      }
    }
  }
}

void ExpBallLocator::sensorUpdate(const Vector2f& measurement, const Matrix2f& measurementCov)
{
  static const Matrix2x4f c = Matrix2x4f::Identity();
  static const Matrix4x2f cTransposed = c.transpose();

  COVARIANCE2D("module:BallLocator:field", measurementCov, measurement);

  for(BallHypothesis* state = states, * end = states + stateCount; state < end; ++state)
    state->sensorUpdate(measurement, measurementCov, c, cTransposed, theBallPercept);
}

void ExpBallLocator::normalizeWeights(BallHypothesis*& bestState, BallHypothesis*& worstStationaryState, BallHypothesis*& worstMovingState)
{
  bestState = nullptr;
  worstStationaryState = 0;
  worstMovingState = 0;
  BallHypothesis*& worstState = worstStationaryState;
  BallHypothesis*& secondWorstState = worstMovingState;

  float highestHeight = 0;
  float highestWeight = 0;
  float worstGain = 0;
  float secondWorstGain = 0;

  for(BallHypothesis* state = states, * end = states + stateCount; state < end; ++state)
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

  for(BallHypothesis* state = states, * end = states + stateCount; state < end; ++state)
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

void ExpBallLocator::createNewStates(const Vector2f& ballPercept, const float ballPerceptRadius, const Matrix2f& ballPerceptCov, BallHypothesis* worstStationaryState, BallHypothesis* worstMovingState)
{
  // create new fixed state
  ASSERT(worstStationaryState || stateCount < sizeof(states) / sizeof(*states));
  BallHypothesis* newState = worstStationaryState;
  if(stateCount < sizeof(states) / sizeof(*states))
    newState = &states[stateCount++];
  ASSERT(newState);
  newState->type = BallHypothesis::stationary;
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
    BallHypothesis* newState = worstMovingState;
    if(stateCount < sizeof(states) / sizeof(*states))
      newState = &states[stateCount++];
    ASSERT(newState);
    newState->type = BallHypothesis::moving;
    ASSERT(stateCount > 1);
    newState->weight = initialStateWeight;
    newState->age = 0;
    newState->radius = ballPerceptRadius;
    float timeScale = 1.f / ((theFrameInfo.time - lastBallPerceptTimeStamp) * 0.001f);
    newState->movingX << ballPercept, (ballPercept - lastBallPercept) * timeScale; // z
    newState->movingCov << ballPerceptCov, Matrix2f::Identity(),
             Matrix2f::Identity(), (ballPerceptCov + lastBallPerceptCov) * timeScale;
  }
}

void ExpBallLocator::generateModel(BallModel& ballModel)
{
  DECLARE_PLOT("module:BallLocator:variance");
  if(bestState == nullptr && stateCount > 0)
    bestState = &states[0];
  if(bestState != nullptr)
  {
    BallState& ballState = ballModel.estimate;
    if(bestState->type == BallHypothesis::moving)
    {
      ballState.position = bestState->movingX.topRows(2);
      ballState.velocity = bestState->movingX.bottomRows(2);
      ballState.covariance = bestState->movingCov.topLeftCorner(2, 2);
      PLOT("module:BallLocator:variance", sqrt(bestState->movingCov(0, 0) + bestState->movingCov(1, 1)));
    }
    else
    {
      ballState.position = bestState->stationaryX;
      ballState.velocity = Vector2f::Zero();
      ballState.covariance = bestState->stationaryCov;
      PLOT("module:BallLocator:variance", sqrt(bestState->stationaryCov(0, 0) + bestState->stationaryCov(1, 1)));
    }
    ballState.radius = bestState->radius;
    Covariance::fixCovariance(ballState.covariance);
  }
  if(ballWasSeenInThisFrame)
  {
    ballModel.timeWhenLastSeen = theFrameInfo.time;
    ballModel.lastPerception = theBallPercept.positionOnField;
  }

  ASSERT(!std::isnan(ballModel.estimate.position.x()) && !std::isnan(ballModel.estimate.position.y()));

  // update timeWhenDisappeared
  {
    bool ballWasSeen = theBallModel.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0;
    bool ballShouldBeVisible = theBallModel.estimate.position.norm() < ballDisappearedMaxCheckingDistance &&
                               BallLocatorTools::ballShouldBeVisibleInCurrentImage(ballModel.estimate.position, ballModel.estimate.radius,
                                                                                   theCameraMatrix, theCameraInfo);
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

bool ExpBallLocator::ballCanBeExcludedByLocalization()
{
  // Not sure about position
  if(theRobotPose.validity != 1.f)
    return false;
  // Not seen cool stuff recently
  if(theFrameInfo.getTimeSince(theRobotPose.timeOfLastConsideredFieldFeature) > fieldFeatureTimeout)
    return false;
  const Pose2f robotPose = theRobotPose + theOdometer.odometryOffset;
  Vector2f ballOnField = theRobotPose * theBallPercept.positionOnField;
  // Not in the field of play ...
  if(!theFieldDimensions.isInsideField(ballOnField))
  {
    const float distanceFromFieldBorder = theFieldDimensions.clipToField(ballOnField);
    // ... and too far away!
    if(distanceFromFieldBorder > fieldBorderExclusionDistance)
      return true;
  }
  return false;
}

bool ExpBallLocator::ballIsAtBorderAndCloseToTeammate()
{
  const int x = static_cast<int>(theBallPercept.positionInImage.x());
  const int y = static_cast<int>(theBallPercept.positionInImage.y());
  const int r = static_cast<int>(theBallPercept.radiusInImage) + ballBorderAdditionalPixelThreshold;

  if(x + r > theCameraInfo.width || x - r < 0 || y - r < 0)
  {
    // Ok, ball seems to be at the image border, now check, if there
    // is a teammate close to the expected ball position on the field:
    const Pose2f robotPose = theRobotPose + theOdometer.odometryOffset;
    const Vector2f ballOnField = theRobotPose * theBallPercept.positionOnField;
    for(auto const& teammate : theTeammateData.teammates)
    {
      if(teammate.status == Teammate::PLAYING || teammate.status == Teammate::FALLEN)
      {
        if((teammate.pose.translation - ballOnField).norm() < robotBanRadius)
        {
          ANNOTATION("ExpBallLocator", "Excluded border ball close to teammate.");
          return true;
        }
      }
    }
  }
  return false;
}
