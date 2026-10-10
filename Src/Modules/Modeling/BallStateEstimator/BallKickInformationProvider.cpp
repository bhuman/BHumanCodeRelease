/**
 * @file BallKickInformationProvider.cpp
 *
 * In ball tracking, the collisions between ball and robot feet need to be handled properly
 * for computing correct velocities and ball positions.
 *
 * The module implemented in this file is provides a representation that contains
 * a function for checking a contact between a single ball hypothesis and a robot foot.
 *
 * @author Philip Reichenberg
 */

#include "BallKickInformationProvider.h"
#include "Tools/Modeling/BallLocatorTools.h"

MAKE_MODULE(BallKickInformationProvider);

BallKickInformationProvider::BallKickInformationProvider()
{
  const float soleToEdgeLength = std::max(theRobotDimensions.soleToOuterEdgeLength, theRobotDimensions.soleToInnerEdgeLength);
  soleOffsets.emplace_back(theRobotDimensions.soleToFrontEdgeLength, soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(theRobotDimensions.soleToFrontEdgeLength, 0.f, 0.f);
  soleOffsets.emplace_back(theRobotDimensions.soleToFrontEdgeLength, -soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(theRobotDimensions.soleToFrontEdgeLength * 0.5f, -soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(0.f, -soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(-theRobotDimensions.soleToBackEdgeLength * 0.5f, -soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(-theRobotDimensions.soleToBackEdgeLength, -soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(-theRobotDimensions.soleToBackEdgeLength, 0.f, 0.f);
  soleOffsets.emplace_back(-theRobotDimensions.soleToBackEdgeLength, soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(-theRobotDimensions.soleToBackEdgeLength * 0.5f, soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(0.f, soleToEdgeLength, 0.f);
  soleOffsets.emplace_back(theRobotDimensions.soleToFrontEdgeLength * 0.5f, soleToEdgeLength, 0.f);

  // Use the largest distance between two points as detection radius
  for(std::size_t index1 = 0; index1 < soleOffsets.size(); index1++)
  {
    std::size_t index2 = index1 + 1;
    if(index2 >= soleOffsets.size())
      index2 = 0;
    const Vector3f diff = soleOffsets[index2] - soleOffsets[index1];
    if(std::abs(diff.x()) > 0.1f)
      detectionRadius = std::max(detectionRadius, std::abs(diff.x()) * 0.5f);
    else
      detectionRadius = std::max(detectionRadius, std::abs(diff.y()) * 0.5f);
  }
}

void BallKickInformationProvider::update(BallKickInformation& theBallKickInformation)
{
  // For logs that jump back in time;
  if(theBallKickInformation.lastBallContactTimestamp > theFrameInfo.time)
  {
    theBallKickInformation.lastBallContactTimestamp = 0;
    ballWasTouched = false;
  }

  // No detection during stopped
  if(theGameState.stopped)
  {
    ballWasTouched = false;
    return;
  }

  const Vector2f& ball = theMotionRequest.ballEstimate.position;
  if(ballWasTouched && (ball - theOdometryDataPreview.inverse() * odometryWhenClose * ballRelativeWhenClose).squaredNorm() > sqr(ballDistanceAfterKick))
  {
    theBallKickInformation.lastBallContactTimestamp = theFrameInfo.time;
    ballWasTouched = false;
  }

  if(theFrameInfo.getTimeSince(theMotionRequest.ballTimeWhenLastSeen) > ballSeenThreshold
     || ball.squaredNorm() > sqr(minBallDistance))
    return;

  // Optimize search sequence
  const Pose3f& firstSole = ball.y() > 0.f ? theRobotModel.soleLeft : theRobotModel.soleRight;
  const Pose3f& secondSole = ball.y() > 0.f ? theRobotModel.soleRight : theRobotModel.soleLeft;

  for(const auto& offset : soleOffsets)
  {
    const Vector3f point = (theTorsoMatrix * (firstSole + offset)).translation;
    if((ball - point.head<2>()).squaredNorm() < sqr(theBallSpecification.radius + detectionRadius))
    {
      ballWasTouched = true;
      odometryWhenClose = theOdometryDataPreview;
      ballRelativeWhenClose = theMotionRequest.ballEstimate.position;
      break;
    }
  }

  // In case the left sole found a collision
  if(!ballWasTouched)
  {
    for(const auto& offset : soleOffsets)
    {
      const Vector3f point = (theTorsoMatrix * (secondSole + offset)).translation;
      if((ball - point.head<2>()).squaredNorm() < sqr(theBallSpecification.radius + detectionRadius))
      {
        ballWasTouched = true;
        odometryWhenClose = theOdometryDataPreview;
        ballRelativeWhenClose = theMotionRequest.ballEstimate.position;
        break;
      }
    }
  }
}
