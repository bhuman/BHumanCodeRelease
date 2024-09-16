/**
 * @file BallPerceptFilter.cpp
 *
 * For ball tracking in the RoboCup context, two tasks need to be solved:
 *   1. Filtering and clustering the detected balls (i.e. the BallPercepts) to avoid
 *      the use of any false positive, which might lead to severe problems.
 *   2. Estimating a precise position as well as a velocity based on a set of recent
 *      ball observations.
 * In previous implementations, these two tasks have been combined within one module. For more
 * clarity and more flexibility, both tasks are split into two modules now.
 *
 * This module provides a solution for task 1: Filtering and clustering BallPercepts.
 *
 * The module implemented in this file is based on the basic filtering implementation that has been used by
 * B-Human inside the BallLocator module in recent years.
 *
 * @author Tim Laue
 */

#include "BallPerceptFilter.h"
#include "Debugging/Annotation.h"
#include "Debugging/Plot.h"
#include "Tools/Modeling/BallLocatorTools.h"

MAKE_MODULE(BallPerceptFilter);

BallPerceptFilter::BallPerceptFilter() : timeBallWasBeenSeenInLowerCameraImage(0),
                                         timeWhenLastKickWasExecuted(0), timeOfLastFilteredPercept(0), shakiness(0.f)
{
}

void BallPerceptFilter::update(FilteredBallPercepts& filteredBallPercepts)
{
  // Declare plots as drawing code is not always executed:
  DECLARE_PLOT("module:BallPerceptFilter:angleBasedDistance");
  DECLARE_PLOT("module:BallPerceptFilter:sizeBasedDistance");
  DECLARE_PLOT("module:BallPerceptFilter:combinedDistance");

  // Reset percept:
  filteredBallPercepts.percepts.clear();

  // Update and plot shakiness information
  shakiness = 0.9f * shakiness + 0.1f * theIMUValueState.gyroValues.deviation.y();
  PLOT("module:BallPerceptFilter:shakiness", shakiness);

  // Remove elements that are too old from buffers:
  while(bufferedSeenBalls.size() != 0)
  {
    if(theFrameInfo.getTimeSince(bufferedSeenBalls.back().timeWhenSeen) > bufferedSeenBallTimeout)
      bufferedSeenBalls.pop_back();
    else
      break;
  }
  while(bufferedBalls.size() != 0)
  {
    if(theFrameInfo.getTimeSince(bufferedBalls.back().timeWhenSeen) > bufferedBallTimeout)
      bufferedBalls.pop_back();
    else
      break;
  }

  // Odometry update for buffered ball percepts:
  const Pose2f odometryOffset = theOdometer.odometryOffset.inverse();
  for(size_t i = 0; i < bufferedSeenBalls.size(); i++)
    bufferedSeenBalls[i].positionOnField = odometryOffset * bufferedSeenBalls[i].positionOnField;
  for(size_t i = 0; i < bufferedBalls.size(); i++)
    bufferedBalls[i].positionOnField = odometryOffset * bufferedBalls[i].positionOnField;

  // Update other internal stuff
  if(theMotionInfo.isKicking()) //&& theBallContactWithRobot.timeOfLastContact > timeWhenLastKickWasExecuted)
    timeWhenLastKickWasExecuted = theFrameInfo.time;

  // Do some first checks
  if(theBallPercept.status == BallPercept::notSeen)
    return;
  if(perceptCanBeExcludedByLocalization() ||
     perceptIsInsideTeammateAndCanBeExcludedByTeamBall() ||
     (disableBallInOtherHalfForTesting && perceptIsInOtherHalf()))
    return;

  Vector2f perceivedBallPosition = theBallPercept.positionOnField;
  // Only do this for far away balls:
  if(correctBallDistanceByPerceivedSize && theCameraInfo.camera == CameraInfo::upper)
  {
    const float usedShakiness = shakiness > 0.5f ? 0.5f : shakiness;
    float sizeFactor = mapToRange(usedShakiness, 0.f, 0.5f, 0.f, 1.f);
    float angleFactor = 1.f - sizeFactor;
    const float sizeBasedDistance = 1.1f * Projection::getDistanceBySize(theCameraInfo, theBallSpecification.radius, theBallPercept.radiusInImage); // Multiply by 1.1, as size-based distance seems to underestimate the real distance. Hacky! ;-)
    Vector2f sizeBasedBallPosition(theBallPercept.positionOnField);
    sizeBasedBallPosition.normalize();
    sizeBasedBallPosition *= sizeBasedDistance;
    perceivedBallPosition = theBallPercept.positionOnField * angleFactor + sizeBasedBallPosition * sizeFactor;
  }

  // If we have reached this part of the code, the ball percept contains information
  // that might be useful and that can be used for some computations.
  const FilteredBallPercept fbp(theBallPercept.positionInImage, perceivedBallPosition,
                                theBallPercept.covarianceOnField, theBallPercept.radiusOnField, theFrameInfo.time);
  bufferedBalls.push_front(fbp);

  // Analyze new percept
  if(theBallPercept.status == BallPercept::seen ||  // The ball was seen and the perceptor seems to be quite sure.
     (theGameState.isPenaltyShootout() && theGameState.isForOpponentTeam() && theBallPercept.status != BallPercept::notSeen) || // A penalty keeper accepts everything ;-)
     currentlyPerceivedGuessedBallIsCloseToPreviouslyPerceivedSeenBalls() || // There have been quite good perceptions close to the currently - not so good - one.
     robotRecentlyKickedAndThereAreGuessedBallsRollingAway() || // After a kick, a sequence of badly perceived balls should be observable
     perceptsAreOnALineThatIsCompatibleToARollingBall()) // We can accept bad percepts if they appear to form a line
  {
    bool ballCanBeUsed = true;
    // Seen balls need further check (guessed ones already had to pass some other checks)
    if(theBallPercept.status == BallPercept::seen)
      ballCanBeUsed = verifySeenBall();

    // Add a new entry for the list of seen balls:
    if(theBallPercept.status == BallPercept::seen)
      bufferedSeenBalls.push_front(fbp);

    if(!ballCanBeUsed)
      return;

    // If we saw a ball in the lower camera recently, we ignore far away balls in the upper camera for a while.
    // However, these balls are still buffered.
    if(theCameraInfo.camera == CameraInfo::upper &&
       theFrameInfo.getTimeSince(timeBallWasBeenSeenInLowerCameraImage) < farBallIgnoreTimeout &&
       theBallPercept.positionOnField.norm() > farBallIgnoreDistance)
      return;

    if(theBallPercept.status == BallPercept::seen)
      filteredBallPercepts.percepts.push_back(bufferedSeenBalls[0]);
    else //if(theBallPercept.status == BallPercept::guessed)
      filteredBallPercepts.percepts.push_back(bufferedBalls[0]);

    // Check, if there is a buffered ball that has not been sent yet.
    // This can still be done with delay, as it might allow the state estimator to compute a velocity
    for(int i = static_cast<int>(bufferedSeenBalls.size()) - 1; i >= 1; i--)
    {
      if(bufferedSeenBalls[i].timeWhenSeen > timeOfLastFilteredPercept)
      {
        filteredBallPercepts.percepts.push_back(bufferedSeenBalls[i]);
        break;
      }
    }

    // Save point of time of last percept transmission
    timeOfLastFilteredPercept = theFrameInfo.time;

    // Update timestamp, if the ball was seen in the lower camera:
    if(theCameraInfo.camera == CameraInfo::lower)
      timeBallWasBeenSeenInLowerCameraImage = theFrameInfo.time;
  }
  plotAndDraw();
}

bool BallPerceptFilter::perceptCanBeExcludedByLocalization()
{
  // Not sure about position
  if(theRobotPose.quality != RobotPose::superb)
    return false;
  Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
  // Not in the field of play ...
  if(!theFieldDimensions.isInsideField(ballOnField))
  {
    const float distanceFromFieldBorder = theFieldDimensions.clipToField(ballOnField);
    // ... and too far away!
    if(distanceFromFieldBorder > fieldBorderExclusionDistance)
    {
      ANNOTATION("BallPerceptFilter", "Excluded ball outside the field.");
      return true;
    }
  }
  return false;
}

bool BallPerceptFilter::perceptIsInsideTeammateAndCanBeExcludedByTeamBall()
{
  // First perform some checks in image coordinates that cover the most typical cases for false positives.
  // These should avoid to exclude real balls that fulfill the following conditions (which unfortunately happens quite often):
  if(((theCameraInfo.camera == CameraInfo::lower && theBallPercept.positionInImage.y() > theBallPercept.radiusInImage) || // Percept does not intersect upper border of image from lower camera
      (theCameraInfo.camera == CameraInfo::upper && theBallPercept.positionInImage.y() + theBallPercept.radiusInImage < theCameraInfo.height)) && // Percept does not intersect lower border of image from upper camera
     (theBallPercept.positionInImage.x() > theBallPercept.radiusInImage) && // Percept does not intersect left border of image
     (theBallPercept.positionInImage.x() + theBallPercept.radiusInImage < theCameraInfo.width) && // Percept does not intersect right border of image
     !ballPerceptIntersectsObstaclesPercept()) // Percept does not intersect with any robot seen in this frame
  {
    // The ball appears to be at a position in the image that is has a low likelihood for causing any false positives.
    // Thus, we do not perform the other checks and accept the percept.
    return false;
  }
  // Now check against data from teammates:
  const Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
  // Check for each teammate, if ...
  for(auto const& teammate : theTeamData.teammates)
  {
    // ... the ball is seen close to its position estimate ...
    if((teammate.getEstimatedPosition(theFrameInfo.time) - ballOnField).norm() < robotBanRadius)
    {
      // .. and the teammate has not seen the ball recently or does not see the ball in its vicinity ...
      if(theFrameInfo.getTimeSince(teammate.theBallModel.timeWhenLastSeen) > 2000 ||
         teammate.theBallModel.estimate.position.norm() > robotBanRadius)
      {
        // ... and finally check, if the ball has been seen recently by a teammate at a
        // completely different place!
        // TL: Not sure, if this is cool :-|
        if(theTeammatesBallModel.isValid && (theTeammatesBallModel.position - ballOnField).norm() > 1500)
        {
          // Only annotate seen balls to avoid spamming with information about guessed ones (which appear very often ...):
          if(theBallPercept.status == BallPercept::seen)
          {
            ANNOTATION("BallPerceptFilter", "Excluded Hackbaellchen!");
            //OUTPUT_TEXT("BallPerceptFilter : Excluded Hackbaellchen!");
          }
          return true;
        }
      }
    }
  }
  return false;
}

bool BallPerceptFilter::ballPerceptIntersectsObstaclesPercept()
{
  for(const auto& o : theObstaclesImagePercept.obstacles)
  {
    // Two opposing corners of the obstacle rectangle:
    const Vector2f topLeft(o.left, o.top);
    const Vector2f bottomRight(o.right, o.bottom);
    if(Geometry::circleIntersectsAxisAlignedRectangle(theBallPercept.positionInImage, theBallPercept.radiusInImage, topLeft, bottomRight))
      return true;
  }
  return false;
}

bool BallPerceptFilter::perceptIsInOtherHalf()
{
  const Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
  // Check, if the x components of both field coordinates have the same sign:
  if(sgn(ballOnField.x()) == sgn(theWorldModelPrediction.robotPose.translation.x()))
    return false;
  // In case of different signs, it is sufficient, if robot or ball is close to the halfway line:
  if(ballOnField.norm() <= toleranceForDisablingBallInOtherHalf ||
     theWorldModelPrediction.robotPose.translation.norm() <= toleranceForDisablingBallInOtherHalf)
    return false;
  // If both conditions are not true, the ball seems to be clearly inside the other half of the field:
  return true;
}

bool BallPerceptFilter::verifySeenBall()
{
  // Override for keeper in penalty shootout:
  // All balls can be used!
  if(theGameState.isPenaltyShootout() && theGameState.isForOpponentTeam())
    return true;
  // Otherwise check distance to other seen balls:
  float verificationDistance = requiredPerceptionDistanceNear;
  unsigned verificationCount = requiredPerceptionCountNear;
  if(theBallPercept.positionOnField.norm() > beginningOfFar)
  {
    verificationDistance = requiredPerceptionDistanceFar;
    verificationCount = requiredPerceptionCountFar;
  }
  if(bufferedSeenBalls.size() < verificationCount)
  {
    return false; // Not enough buffered balls to compare with
  }
  else
  {
    for(unsigned int i = 0; i < verificationCount; i++)
    {
      FilteredBallPercept& b = bufferedSeenBalls[i];
      // Ball is too far away ...
      if((b.positionOnField - theBallPercept.positionOnField).norm() > verificationDistance)
      {
        // .. from almost stationary ball:
        if(theWorldModelPrediction.ballVelocity.norm() <= 10.f)
        {
          return false;
        }
        // .. or from direction of a rolling ball
        Geometry::Line ballTrajectory(b.positionOnField, theWorldModelPrediction.ballVelocity);
        ballTrajectory.direction *= 3.f; // Possibility of higher velocity
        const float dist = Geometry::getDistanceToEdge(ballTrajectory, theBallPercept.positionOnField);
        if(dist > verificationDistance)
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool BallPerceptFilter::currentlyPerceivedGuessedBallIsCloseToPreviouslyPerceivedSeenBalls()
{
  // Method should not have been called in this case:
  if(theBallPercept.status != BallPercept::guessed)
    return false;
  // Not enough seen balls to compare with:
  if(bufferedSeenBalls.size() < neededSeenBallsForAcceptingGuessedOne)
  {
    return false;
  }
  else
  {
    const Angle angleToPercept = theBallPercept.positionOnField.angle();
    for(unsigned int i = 0; i < neededSeenBallsForAcceptingGuessedOne; i++)
    {
      FilteredBallPercept& b = bufferedSeenBalls[i];
      const Angle angleToBuff = b.positionOnField.angle();
      if((b.positionOnField - theBallPercept.positionOnField).norm() > maximumDistanceBetweenSeenAndGuessed ||
         angleToPercept.diffAbs(angleToBuff) > maximumAngleBetweenSeenAndGuessed)
      {
        return false;
      }
    }
  }
  return true;
}

bool BallPerceptFilter::robotRecentlyKickedAndThereAreGuessedBallsRollingAway()
{
  // Method should not have been called in this case:
  if(theBallPercept.status != BallPercept::guessed)
    return false;
  // We have not kicked recently:
  if(theFrameInfo.getTimeSince(timeWhenLastKickWasExecuted) > timeSpanForAcceptingKickedGuessedBalls)
    return false;
  // Find the percepts that are assumed to be a trace of a kicked ball:
  unsigned lastIndex = 0;
  for(unsigned int i = 0; i < bufferedBalls.size(); i++)
  {
    if(bufferedBalls[i].timeWhenSeen > timeWhenLastKickWasExecuted &&
       theFrameInfo.getTimeSince(bufferedBalls[i].timeWhenSeen) < timeSpanForAcceptingKickedGuessedBalls)
      lastIndex = i;
    else
      break;
  }
  // Do we have enough balls in buffer?
  if(static_cast<int>(lastIndex + 1) < requiredNumberOfGuessBallsForKickDetection)
    return false;
  // If yes, these balls need to have decreasing distances (due to order in buffer) as we assume that the ball is rolling away
  float distance = theFieldDimensions.xPosOpponentGoalLine * 4.f;;
  for(unsigned int i = 0; i <= lastIndex; i++)
  {
    const float currentDistance = bufferedBalls[i].positionOnField.norm();
    if(currentDistance < distance)
      distance = currentDistance;
    else
      return false;
  }
  ANNOTATION("BallPerceptFilter", "Found a sequence of " << lastIndex + 1 << " balls with an increasing distance after a kick!");
  return true;
}

bool BallPerceptFilter::perceptsAreOnALineThatIsCompatibleToARollingBall()
{
  // Method should not have been called in this case:
  if(theBallPercept.status != BallPercept::guessed)
    return false;
  // Find the percepts that are assumed to be a trace of a rolling ball:
  unsigned lastIndex = 0;
  for(unsigned int i = 0; i < bufferedBalls.size(); i++)
  {
    if(theFrameInfo.getTimeSince(bufferedBalls[i].timeWhenSeen) < timeSpanForGuessedBallsMotionDetection)
      lastIndex = i;
    else
      break;
  }
  // Do we have enough balls in buffer?
  if(static_cast<int>(lastIndex + 1) < requiredNumberOfGuessBallsForMotionDetection)
    return false;
  // If yes, check, if the motion hypothesis is more likely than a hypothesis of a rolling ball.
  // Furthermore, the rolling ball needs to have a minimum velocity.
  float stdDevStatic = computeStdDevOfStaticBallHypothesis(lastIndex);
  Vector2f velocity;
  float stdDevMoving = computeStdDevOfMovingBallHypothesis(lastIndex, velocity);
  float speed = velocity.norm();
  if(stdDevMoving < stdDevStatic && stdDevMoving < maxStandardDeviationMotionDetection &&
     speed > minimumVelocityForMotionDetection && speed < theBallSpecification.ballSpeedUpperBound())
  {
    ANNOTATION("BallPerceptFilter", "Found a sequence that indicates a rolling ball: " << stdDevMoving << " < " << stdDevStatic << "  with " << speed << "mm/s!");
    // OUTPUT_TEXT("Found a sequence that indicates a rolling ball: " << stdDevMoving << " < " << stdDevStatic << "  with " << speed << "mm/s!");
    return true;
  }
  return false;
}

float BallPerceptFilter::computeStdDevOfStaticBallHypothesis(unsigned indexOfOldestObservation)
{
  Vector2f mean = bufferedBalls[0].positionOnField;
  for(unsigned int i = 1; i <= indexOfOldestObservation; ++i)
    mean += bufferedBalls[i].positionOnField;
  mean /= static_cast<float>(indexOfOldestObservation + 1);
  float variance = 0.f;
  for(unsigned int i = 0; i <= indexOfOldestObservation; ++i)
    variance += (bufferedBalls[i].positionOnField - mean).squaredNorm();
  variance /= (indexOfOldestObservation + 1);
  return std::sqrt(variance);
}

float BallPerceptFilter::computeStdDevOfMovingBallHypothesis(unsigned indexOfOldestObservation, Vector2f& endVelocity)
{
  if(indexOfOldestObservation < 2)
    return 500.f;
  Eigen::MatrixX2f A(indexOfOldestObservation * 2, 2);
  Eigen::VectorXf  z(indexOfOldestObservation * 2);
  unsigned int t0 = bufferedBalls[indexOfOldestObservation].timeWhenSeen;
  for(unsigned int i = indexOfOldestObservation; i > 0; i--)
  {
    const FilteredBallPercept& b1 = bufferedBalls[i];
    const FilteredBallPercept& b2 = bufferedBalls[i - 1];
    const auto dt    = (b2.timeWhenSeen - b1.timeWhenSeen) / 1000.f;   // in s
    const auto tToB1 = (b1.timeWhenSeen - t0) / 1000.f;        // in s
    const auto dPos = (b2.positionOnField - b1.positionOnField) / 1000.f;      // in m
    z((i - 1) * 2)         = dPos.x() - 0.5f * theBallSpecification.friction * dt * dt + dt * tToB1;
    z((i - 1) * 2 + 1)     = dPos.y() - 0.5f * theBallSpecification.friction * dt * dt + dt * tToB1;
    A((i - 1) * 2, 0)      = dt;
    A((i - 1) * 2, 1)      = 0.f;
    A((i - 1) * 2 + 1, 0)  = 0.f;
    A((i - 1) * 2 + 1, 1)  = dt;
  }
  Vector2f x;
  x = (A.transpose() * A).inverse() * A.transpose() * z;
  //OUTPUT_TEXT("Result:  v=(" << x(0) << "," << x(1) << ")");

  // Compute residuals and print error on console:
  Eigen::VectorXf residuals = A*x - z;
  float meanError = residuals.norm() / std::sqrt(static_cast<float>(indexOfOldestObservation));
  // Compute the estimated velocity at last ball perception's position
  Vector2f startPosition = bufferedBalls[indexOfOldestObservation].positionOnField;
  Vector2f startVelocity = x * 1000.f;
  float timeSpan = (bufferedBalls[0].timeWhenSeen - bufferedBalls[indexOfOldestObservation].timeWhenSeen) / 1000.f;
  BallPhysics::propagateBallPositionAndVelocity(startPosition, startVelocity, timeSpan, theBallSpecification.friction);
  endVelocity = startVelocity;

  // Return error:
  return meanError * 1000.f; // in mm
}

void BallPerceptFilter::plotAndDraw()
{
  if(theBallPercept.status == BallPercept::seen && theCameraInfo.camera == CameraInfo::upper)
  {
    const float angleBasedDistance = theBallPercept.positionOnField.norm();
    PLOT("module:BallPerceptFilter:angleBasedDistance", angleBasedDistance);
    const float sizeBasedDistance = 1.1f * Projection::getDistanceBySize(theCameraInfo, theBallSpecification.radius, theBallPercept.radiusInImage);
    PLOT("module:BallPerceptFilter:sizeBasedDistance", sizeBasedDistance);

    const float usedShakiness = shakiness > 0.5f ? 0.5f : shakiness;
    float sizeFactor = mapToRange(usedShakiness, 0.f, 0.5f, 0.f, 1.f);
    float angleFactor = 1.f - sizeFactor;
    const float combinedDistance = sizeFactor * sizeBasedDistance + angleFactor * angleBasedDistance;
    PLOT("module:BallPerceptFilter:combinedDistance", combinedDistance);
  }
}
