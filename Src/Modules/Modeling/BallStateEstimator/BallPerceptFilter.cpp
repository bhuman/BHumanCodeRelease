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
#include "Tools/Debugging/Annotation.h"
#include "Tools/Modeling/BallLocatorTools.h"
#include "Tools/Modeling/Measurements.h"

MAKE_MODULE(BallPerceptFilter, modeling)

BallPerceptFilter::BallPerceptFilter() : timeBallWasBeenSeenInLowerCameraImage(0),
                                         timeWhenLastKickWasExecuted(0), timeOfLastFilteredPercept(0)
{
}

void BallPerceptFilter::update(FilteredBallPercepts& filteredBallPercepts)
{
  // Do some test stuff:
  // doSomeTestStuff();

  // Reset percept:
  filteredBallPercepts.percepts.clear();

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
  for(size_t i=0; i<bufferedSeenBalls.size(); i++)
    bufferedSeenBalls[i].positionOnField = odometryOffset * bufferedSeenBalls[i].positionOnField;
  for(size_t i=0; i<bufferedBalls.size(); i++)
    bufferedBalls[i].positionOnField = odometryOffset * bufferedBalls[i].positionOnField;

  // Update other internal stuff
  if(theMotionInfo.isKicking()) //&& theBallContactWithRobot.timeOfLastContact > timeWhenLastKickWasExecuted)
    timeWhenLastKickWasExecuted = theFrameInfo.time;

  // Do some first checks
  if(theBallPercept.status == BallPercept::notSeen)
    return;
  if(perceptCanBeExcludedByLocalization() ||
     perceptIsAtImageBorderAndCloseToTeammate() ||
     perceptIsInsideTeammateAndCanBeExcludedByTeamBall() ||
     (disableBallInOtherHalfForTesting && perceptIsInOtherHalf()))
    return;

  // If we have reached this part of the code, the ball percept contains information
  // that might be useful and that can be used for some computations.
  const Matrix2f ballPerceptCov = Measurements::positionToCovarianceMatrixInRobotCoordinates(
                                    theBallPercept.positionOnField, 0.f, theCameraMatrix,
                                    theCameraMatrix.inverse(), robotRotationDeviation);

  const FilteredBallPercept fbp(theBallPercept.positionInImage, theBallPercept.positionOnField,
                                ballPerceptCov, theBallPercept.radiusOnField, theFrameInfo.time);
  bufferedBalls.push_front(fbp);

  // Analyze new percept
  if(theBallPercept.status == BallPercept::seen ||  // The ball was seen and the perceptor seems to be quite sure.
     (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theTeamBehaviorStatus.role.isGoalkeeper && theBallPercept.status != BallPercept::notSeen) || // A penalty keeper accepts everything ;-)
     currentlyPerceivedGuessedBallIsCloseToPreviouslyPerceivedSeenBalls() || // There have been quite good perceptions close to the currently - not so good - one.
     robotRecentlyKickedAndThereAreGuessedBallsRollingAway() || // After a kick, a sequence of badly perceived balls should be observerable
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
}

bool BallPerceptFilter::perceptCanBeExcludedByLocalization()
{
  // Not sure about position
  if(theRobotPose.validity != 1.f)
    return false;
  // Not seen cool stuff recently
  if(theFrameInfo.getTimeSince(theRobotPose.timeOfLastConsideredFieldFeature) > fieldFeatureTimeout)
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
  const Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
  for(auto const& teammate : theTeamData.teammates)
  {
    // Teammate is on the pitch ...
    if(teammate.status == Teammate::PLAYING || teammate.status == Teammate::FALLEN)
    {
      // ... and the ball is seen close to its position estimate ...
      if((teammate.theRobotPose.translation - ballOnField).norm() < robotBanRadius)
      {
        // .. and the teammate has not seen the ball recently or does not see the ball in its vicinity ...
        if(theFrameInfo.getTimeSince(teammate.theBallModel.timeWhenLastSeen) > 2000 ||
           teammate.theBallModel.estimate.position.norm() > robotBanRadius)
        {
          // ... then we can finally check, if the ball has been seen by multiple teammates at a
          // completely different place!
          if(theTeamBallModel.isValid && theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) < 2000 &&
             (theTeamBallModel.contributors == TeamBallModel::multipleOthers || theTeamBallModel.contributors == TeamBallModel::meAndOthers) &&
             (theTeamBallModel.position - ballOnField).norm() > 1500)
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
  }
  return false;
}

bool BallPerceptFilter::perceptIsAtImageBorderAndCloseToTeammate()
{
  if(!banBallsInRobotsAtImageBorder)
    return false;
  const int x = static_cast<int>(theBallPercept.positionInImage.x());
  const int y = static_cast<int>(theBallPercept.positionInImage.y());
  const int r = static_cast<int>(theBallPercept.radiusInImage) + ballBorderAdditionalPixelThreshold;

  if(x + r > theCameraInfo.width || x - r < 0 || y - r < 0)
  {
    // Ok, ball seems to be at the image border, now check, if there
    // is a teammate close to the expected ball position on the field:
    const Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
    for(auto const& teammate : theTeamData.teammates)
    {
      if(teammate.status == Teammate::PLAYING || teammate.status == Teammate::FALLEN)
      {
        if((teammate.theRobotPose.translation - ballOnField).norm() < robotBanRadius)
        {
          ANNOTATION("BallPerceptFilter", "Excluded border ball close to teammate.");
          return true;
        }
      }
    }
  }
  return false;
}

bool BallPerceptFilter::perceptIsInOtherHalf()
{
  const Vector2f ballOnField = theWorldModelPrediction.robotPose * theBallPercept.positionOnField;
  // Check, if the x componentes of both field coordinates have the same sign:
  if(sgn(ballOnField.x()) == sgn(theWorldModelPrediction.robotPose.translation.x()))
    return false;
  // In case of different signs, it is sufficient, if robot or ball is close to the center line:
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
  if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theTeamBehaviorStatus.role.isGoalkeeper)
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
      if((b.positionOnField - theBallPercept.positionOnField).norm() > verificationDistance &&
         theWorldModelPrediction.ballVelocity.norm() <= 0.1f)
      {
        return false;
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
 float distance = theFieldDimensions.xPosOpponentGroundline * 4.f;;
 for(unsigned int i=0; i<= lastIndex; i++)
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
  // If yes, checke, if the motion hypothesis is more likely than a hypothesis of a rolling ball.
  // Furthermore the rolling ball needs to have a minimum velocity
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
  for(unsigned int i=1; i<=indexOfOldestObservation; ++i)
    mean += bufferedBalls[i].positionOnField;
  mean /= static_cast<float>(indexOfOldestObservation + 1);
  float variance = 0.f;
  for(unsigned int i=0; i<=indexOfOldestObservation; ++i)
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
    z((i-1) * 2)         = dPos.x() - 0.5f * theBallSpecification.friction * dt * dt + dt * tToB1;;
    z((i-1) * 2 + 1)     = dPos.y() - 0.5f * theBallSpecification.friction * dt * dt + dt * tToB1;;
    A((i-1) * 2, 0)      = dt;
    A((i-1) * 2, 1)      = 0.f;
    A((i-1) * 2 + 1, 0)  = 0.f;
    A((i-1) * 2 + 1, 1)  = dt;
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
  BallPhysics::propagateBallPositionAndVelocity(startPosition,startVelocity, timeSpan, theBallSpecification.friction);
  endVelocity = startVelocity;

  // Return error:
  return meanError * 1000.f; // in mm
}

void BallPerceptFilter::doSomeTestStuff()
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptFilter:testStuff", "drawingOnField");
  if(theCameraInfo.camera == CameraInfo::lower)
    return;
  Vector2f pointRelativeToRobot(2000.f,0.f);
  MODIFY("module:BallPerceptFilter:testPoint", pointRelativeToRobot);
  const Matrix2f cov = Measurements::positionToCovarianceMatrixInRobotCoordinates(
                                       pointRelativeToRobot, 0.f, theCameraMatrix,
                                       theCameraMatrix.inverse(), robotRotationDeviation);
  COVARIANCE_ELLIPSES_2D("module:BallPerceptFilter:testStuff", cov, pointRelativeToRobot);
}

