#include "BallPerceptorEvaluator.h"

MAKE_MODULE(BallPerceptorEvaluator, modeling)

BallPerceptorEvaluator::BallPerceptorEvaluator()
{
  distanceContainer = RingBuffer<float, 100>();
  lowerDifferenceContainer = RingBuffer<float, 100>();
  upperDifferenceContainer = RingBuffer<float, 100>();
}

BallPerceptorEvaluator::~BallPerceptorEvaluator()
{
}

void BallPerceptorEvaluator::update(BallPerceptorEvaluation& evaluation)
{
  //Regard the current execution
  ++frameCounter;
  keepInLimits(frameCounter, 100);
  //If there is a percept make an update
  if(theBallPercept.status == BallPercept::seen)
  {
    //We now have a real percept and calculate the actual differences
    ++perceptCounter;
    keepInLimits(perceptCounter, 100);
    currentPosition = theBallPercept.positionOnField;
    float distance = theBallPercept.positionOnField.norm();
    distanceContainer.push_front(distance);
    //Makes the jump between positions this percept a false positive?
    float differenceBetweenPositions = (currentPosition - lastPosition).norm();
    float differenceBetweenAngles = std::fabs(Angle(std::atan2(currentPosition.y(), currentPosition.x()) - std::atan2(lastPosition.y(), lastPosition.x())).toDegrees());
    if(differenceBetweenPositions > jumpOnFieldThreshold && differenceBetweenAngles > jumpAngleThreshold)
      ++falsePositiveCounter;
    keepInLimits(falsePositiveCounter, 1);
    //Calculate differences for cameras
    float inCameraDifference;
    if(theCameraInfo.camera == CameraInfo::upper)
    {
      inCameraDifference = (lastPositionUpper - currentPosition).norm();
      upperDifferenceContainer.push_front(inCameraDifference);
      lastPositionUpper = currentPosition;
    }
    else
    {
      inCameraDifference = (lastPositionLower - currentPosition).norm();
      lowerDifferenceContainer.push_front(inCameraDifference);
      lastPositionLower = currentPosition;
    }
    //Update the representation
    updateSeenPercentage(evaluation);
    updateGuessedFalsePositives(evaluation);
    updateAverageDifferenceForCamera(evaluation);
    updateDistanceStatistics(evaluation, distance);
    lastPosition = currentPosition;
  }
}

void BallPerceptorEvaluator::updateSeenPercentage(BallPerceptorEvaluation& evaluation)
{
  evaluation.seenPercentage = (float) perceptCounter / (float) frameCounter;
}

void BallPerceptorEvaluator::updateGuessedFalsePositives(BallPerceptorEvaluation& evaluation)
{
  evaluation.guessedFalsePositives = (float) falsePositiveCounter / (float) perceptCounter;
}

void BallPerceptorEvaluator::updateAverageDifferenceForCamera(BallPerceptorEvaluation& evaluation)
{
  if(theCameraInfo.camera == CameraInfo::upper)
    evaluation.averageDifferenceUpper = getAverageFromBuffer(upperDifferenceContainer);
  else
    evaluation.averageDifferenceLower = getAverageFromBuffer(lowerDifferenceContainer);
}

float BallPerceptorEvaluator::getAverageFromBuffer(const RingBuffer<float, 100>& buffer)
{
  float sum = 0.f;
  if(buffer.empty())
    return 0.f;
  int bufferSize = (int) buffer.size();
  for(int i = 0; i < bufferSize; ++i)
    sum += buffer[i];
  sum /= (float) bufferSize;
  return sum;
}

void BallPerceptorEvaluator::keepInLimits(unsigned& value, unsigned substitute)
{
  if(value % std::numeric_limits<unsigned>::max() == 0)
    value = substitute;
}

void BallPerceptorEvaluator::updateDistanceStatistics(BallPerceptorEvaluation& evaluation, const float& distance)
{
  if(distance < minimumDistance)
    minimumDistance = distance;
  if(distance > maximumDistance)
    maximumDistance = distance;
  evaluation.maximumPerceptDistance = maximumDistance;
  evaluation.minimumPerceptDistance = minimumDistance;
  evaluation.averagePerceptDistance = getAverageFromBuffer(distanceContainer);
}
