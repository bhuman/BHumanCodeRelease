/**
 * @file Goalkeeper.cpp
 *
 * This file implements the goalkeeper role.
 *
 * @author Arne Hasselbring
 */

#include "Goalkeeper.h"

Pose2f Goalkeeper::position(Side, const Pose2f& basePose, const std::vector<Vector2f>&, const Agents&)
{
  return theGameState.isGoalKick() && theGameState.isForOwnTeam() ? basePose : theGoaliePose.goaliePoseField;
}

Pose2f Goalkeeper::tolerance() const
{
  const float distanceFactor = mapToRange(theFieldBall.recentBallPositionRelative().norm(), ballDistanceInterpolationRange.min, ballDistanceInterpolationRange.max, 0.f, 1.f);
  const Angle rotationThreshold = distanceFactor * rotationThresholdRange.max + (1.f - distanceFactor) * rotationThresholdRange.min;
  const float translationXThreshold = distanceFactor * translationXThresholdRange.max + (1.f - distanceFactor) * translationXThresholdRange.min;
  const float translationYThreshold = distanceFactor * translationYThresholdRange.max + (1.f - distanceFactor) * translationYThresholdRange.min;
  return Pose2f(rotationThreshold, translationXThreshold, translationYThreshold);
}

bool Goalkeeper::shouldStop(const Pose2f& target) const
{
  const Pose2f error = target.inverse() * theRobotPose;
  return std::abs(error.rotation) < shouldStopRotation &&
         std::abs(error.translation.x()) < shouldStopTranslation.x() &&
         std::abs(error.translation.y()) < shouldStopTranslation.y();
}
