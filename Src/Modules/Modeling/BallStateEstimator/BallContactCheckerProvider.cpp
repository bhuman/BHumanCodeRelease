/**
 * @file BallContactCheckerProvider.cpp
 *
 * In ball tracking, the collisions between ball and robot feet need to be handled properly
 * for computing correct velocities and ball positions.
 *
 * The module implemented in this file is provides a representation that contains
 * a function for checking a contact between a single ball hypothesis and a robot foot.
 *
 * @author Tim Laue
 */

#include "BallContactCheckerProvider.h"
#include "Tools/Modeling/BallLocatorTools.h"


MAKE_MODULE(BallContactCheckerProvider, modeling)


BallContactCheckerProvider::BallContactCheckerProvider()
{
  lastLeftFootCenter = Vector2f::Zero();
  lastRightFootCenter = Vector2f::Zero();
  deltaTime = 1.f;
  lastMotionFrameTime = 0;
}

void BallContactCheckerProvider::update(BallContactChecker& ballContactChecker)
{
  // Finally update some members
  lastLeftFootCenter = leftFootCenter;
  lastRightFootCenter = rightFootCenter;

  const Vector3f rightFootOffset(leftFootOffset.x(), -leftFootOffset.y(), leftFootOffset.z());
  const Pose3f leftFoot = theTorsoMatrix * (theRobotModel.soleLeft + leftFootOffset);
  const Pose3f rightFoot = theTorsoMatrix * (theRobotModel.soleRight + rightFootOffset);
  leftFootCenter = leftFoot.translation.topRows(2);
  rightFootCenter = rightFoot.translation.topRows(2);
  ASSERT(leftFootCenter != Vector2f::Zero() && rightFootCenter != Vector2f::Zero());
  deltaTime = (static_cast<int>(theJointAngles.timestamp) - static_cast<int>(lastMotionFrameTime)) * 0.001f; // in seconds
  lastMotionFrameTime = theJointAngles.timestamp;

  ballContactChecker.collide = [this](const Vector2f& p, const Vector2f& v, const Vector2f& lastP, BallContactInformation& contactInfo) -> bool
  {
    if(theFrameInfo.getTimeSince(theWorldModelPrediction.timeWhenBallLastSeen) > timeout)
      return false;
    if(!theMotionInfo.isMotionStable)
      return false;
    if(theWorldModelPrediction.ballIsPredictedByRule)
       return false;
    return handleCollisionWithFeet(leftFootCenter, rightFootCenter, p, v, lastP, contactInfo);
  };
}

bool BallContactCheckerProvider::handleCollisionWithFeet(const Vector2f& leftFootCenter,
                                                         const Vector2f& rightFootCenter,
                                                         const Vector2f& position, const Vector2f& velocity,
                                                         const Vector2f& lastPosition,
                                                         BallContactInformation& contactInfo)
{
  // Get the currently estimated ball state
  Vector2f ballPosition = position;
  Vector2f ballVelocity = velocity;

  // Declare variables for further reasoning
  const float assumedRadius = footRadius + theBallSpecification.radius;
  const Vector2f lastBallPosition = lastPosition;

  // detect collisions with feet
  bool leftCollision = false;
  bool rightCollision = false;
  bool centerCollision = false;

  if(deltaTime > 0.f)
  {
    float leftCollisionFactor, rightCollisionFactor;
    Vector2f leftAssumedLastBallPosition, rightAssumedLastBallPosition;
    Vector2f leftAssumedBallOffset, rightAssumedBallOffset;
    leftCollision = collisionForFootCalculation(leftAssumedLastBallPosition, leftAssumedBallOffset, ballPosition, lastBallPosition, leftFootCenter, lastLeftFootCenter, assumedRadius, leftCollisionFactor);
    rightCollision = collisionForFootCalculation(rightAssumedLastBallPosition, rightAssumedBallOffset, ballPosition, lastBallPosition, rightFootCenter, lastRightFootCenter, assumedRadius, rightCollisionFactor);

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
        const float ballSpeed = ballVelocity.norm();
        const float upperSpeedBound = theBallSpecification.ballSpeedUpperBound();
        if(ballSpeed > upperSpeedBound)
        {
          ballVelocity.normalize(upperSpeedBound);
        }
        ballPosition += ballVelocity * (1.f - collisionFactor) * deltaTime;
      }
    }
  }

  if(!(leftCollision || rightCollision))
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

  // Compute new position and velocity
  if(leftCollision || rightCollision || centerCollision || ballShift != Vector2f::Zero())
  {
    contactInfo.newPosition = ballPosition;
    contactInfo.newVelocity = ballVelocity;
    if(ballVelocity != Vector2f::Zero())
    {
      contactInfo.addVelocityCov.x() = sqr(ballVelocity.x() * kickDeviation.x());
      contactInfo.addVelocityCov.y() = sqr(ballVelocity.y() * kickDeviation.y());
    }
    else
    {
      contactInfo.addVelocityCov = Vector2f(1.f,1.f);
    }
    if(centerCollision)
      contactInfo.contactType = BallContactInformation::center;
    else if(leftCollision)
      contactInfo.contactType = BallContactInformation::leftFoot;
    else if(rightCollision)
      contactInfo.contactType = BallContactInformation::rightFoot;
    else
      contactInfo.contactType = BallContactInformation::none;
    return true;
  }
  return false;
}

bool BallContactCheckerProvider::collisionForFootCalculation(Vector2f& assumedLastBallPosition, Vector2f& assumedBallOffset, const Vector2f& ballPosition, const Vector2f& lastBallPosition, const Vector2f& footCenter, const Vector2f& lastFootCenter, const float radius, float& collisionFactor)
{
  assumedLastBallPosition = lastBallPosition + (footCenter - lastFootCenter);
  assumedBallOffset = ballPosition - assumedLastBallPosition;
  bool collision = BallLocatorTools::getSmallestLineWithCircleIntersectionFactor(assumedLastBallPosition, assumedBallOffset, lastFootCenter, radius, collisionFactor);
  if(collision && (collisionFactor < 0.f || collisionFactor > 1.f))
    collision = false;
  return collision;
}
