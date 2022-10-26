/**
 * @file FreeKickWall.cpp
 *
 * This file implements a behavior to position between the ball and the own penalty mark.
 *
 * @author Arne Hasselbring
 * @author Sina Schreiber
 */

#include "FreeKickWall.h"

SkillRequest FreeKickWall::execute(const Agent&, const Agents&)
{
  const Vector2f ballPosition = theFieldBall.recentBallPositionOnField(); // the position of the ball on the field
  const Vector2f defenderVector = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f) - ballPosition; // change of direction
  const Vector2f newTarget = ballPosition + defenderVector.normalized(normalizeValue); // calculated new target
  const Angle rotationAngle = defenderVector.angle(); // angle for the rotation of the direction
  const Vector2f rotatedNewTargetPosition = newTarget.rotated(-rotationAngle); // rotated new target
  const Vector2f rotatedOldTargetPosition = currentTarget.rotated(-rotationAngle); // rotated old target
  /**
   * if the new target is within the threshold keep the position otherwise walk to the new target.
   */
  if(std::abs(rotatedNewTargetPosition.x() - rotatedOldTargetPosition.x()) > maxXValue ||
     (std::abs(rotatedNewTargetPosition.y() - rotatedOldTargetPosition.y()) > maxYValue))
    currentTarget = newTarget;
  return SkillRequest::Builder::walkTo(Pose2f((ballPosition - currentTarget).angle(), currentTarget));
}
