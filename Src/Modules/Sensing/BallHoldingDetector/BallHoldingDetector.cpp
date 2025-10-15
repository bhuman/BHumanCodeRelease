/**
 * @file BallHoldingDetector.h
 * @author Harm Thordsen
 */

#include "BallHoldingDetector.h"
#include "Debugging/Plot.h"

MAKE_MODULE(BallHoldingDetector);

void BallHoldingDetector::update(BallHoldingState& theBallHoldingState)
{
  DECLARE_PLOT("module:BallHoldingDetector:leftAvg");
  DECLARE_PLOT("module:BallHoldingDetector:rightAvg");

  theBallHoldingState.ballHolding = false;

  // Only detect while being upright
  if(theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::staggering)
  {
    lastLeftSoleY.clear();
    lastRightSoleY.clear();
    ringBufferLeft.clear();
    ringBufferRight.clear();
    return;
  }

  // Expected Sole positions
  const RobotModel requestedModel(theJointRequest, theRobotDimensions, theMassCalibration);
  lastLeftSoleY.push_front(requestedModel.soleLeft.translation.y());
  lastRightSoleY.push_front(requestedModel.soleRight.translation.y());

  if(!lastRightSoleY.full())
    return;

  // Get current measured values
  const float currentLeftY = theRobotModel.soleLeft.translation.y();
  const float currentRightY = theRobotModel.soleRight.translation.y();

  // Diff for left leg. Subtraction should give a positive value in case there is a ball
  const float leftFootDiff = currentLeftY - lastLeftSoleY.back();

  // Diff for right leg. Subtraction should give a positive value in case there is a ball
  const float rightFootDiff = lastRightSoleY.back() - currentRightY;

  ringBufferLeft.push_front(leftFootDiff);
  ringBufferRight.push_front(rightFootDiff);

  // Only use when both ring buffers are full
  if(ringBufferLeft.full() && ringBufferRight.full())
  {
    PLOT("module:BallHoldingDetector:leftAvg", ringBufferLeft.average());
    PLOT("module:BallHoldingDetector:rightAvg", ringBufferRight.average());

    // Ball not seen between 100ms and 1000ms and less than 1m away
    const bool lowerLimitBallSeen = theFieldBall.timeSinceBallWasSeen > minTimeSinceBallWasSeen;
    const bool upperLimitBallSeen = theFieldBall.timeSinceBallWasSeen < maxTimeSinceBallWasSeen;
    const bool distanceToBall = theBallLostModel.relativeAlternateBallPosition.squaredNorm() < sqr(maxDistanceToBall);
    if(lowerLimitBallSeen && upperLimitBallSeen && distanceToBall && theGameState.isPlaying())
    {
      // Sum of ring buffer bigger than certain threshold to count as ball holding
      if(ringBufferLeft.average() > minLegYDiff && ringBufferRight.average() > minLegYDiff)
        theBallHoldingState.ballHolding = true;
    }
  }
}
