#include <cfloat>
#include <algorithm>
#include "ObstacleWheelProvider.h"
#include "Tools/Math/Common.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Vector.h"
#include "Representations/Infrastructure/JointData.h"
MAKE_MODULE(ObstacleWheelProvider, Modeling)

ObstacleWheelProvider::ObstacleWheelProvider() : oldConeWidth(0),
                                                 oldWheelRadius(0), lastDecreaseTimestamp(0), coneCount(0)
{}

void ObstacleWheelProvider::update(ObstacleWheel& wheel)
{
  if(coneWidth != oldConeWidth || wheelRadius != oldWheelRadius)
  {
    initialize(wheel);
    oldConeWidth = coneWidth;
    oldWheelRadius = wheelRadius;
  }

  //clear wheel if we are penalized, falling, getting up, etc.
  if (theGameInfo.state == STATE_INITIAL || theGameInfo.state == STATE_FINISHED ||
  theFallDownState.state == FallDownState::onGround ||
  theFallDownState.state == FallDownState::falling ||
  theMotionInfo.motion == MotionRequest::getUp ||
  theRobotInfo.penalty != PENALTY_NONE ||
  !theGroundContactState.contact)
  {
    clearWheel(wheel);
    return;
  }

  //always decrease the seenCount and update odometry
  decreaseSeenCount(wheel);
  updateOdometry(wheel);
  //but only enter new obstacles if we are not doing anything special right now
  if(theMotionInfo.motion == MotionRequest::walk ||
     theMotionInfo.motion == MotionRequest::stand)
  {
    enterObstacles(wheel);
    stuffHoles(wheel);
  }


}

int ObstacleWheelProvider::calcConeIndex(const Vector2<float>& pInField, ObstacleWheel& wheel) const
{
  const float angle = pInField.angle();
  int index;
  if(angle < 0 && angle > -0.01f)
  {
    /** If the angle is very close to negative zero (angle / wheel.coneWidth)
     becomes very very small. If a very very small negative number is added to
     wheel.cones.size() it has no effect. This special case fixes the issue.*/
    return wheel.cones.size() - 1;
  }
  else if(angle < 0)
  {
    index = int(wheel.cones.size() + (angle / wheel.coneWidth));
  }
  else
  {
    index = int(angle / wheel.coneWidth);
  }

  ASSERT(index >= 0);
  ASSERT(index < coneCount);

  return index;
}

void ObstacleWheelProvider::updateOdometry(ObstacleWheel& wheel) const
{
  const float rot = -theOdometer.odometryOffset.rotation;
  const Vector2<float> translation = theOdometer.odometryOffset.translation * -1;

  Matrix2x2<float> rotationMatrix(std::cos(rot), -std::sin(rot),
                                  std::sin(rot), std::cos(rot));

  std::vector<ObstacleWheel::Spot> newSpots; //FIXME slooooooooowww

  //move all spots according to odometry difference
  for(int i = 0; i < coneCount; ++i)
  {
    ObstacleWheel::Cone& cone = wheel.cones[i];
    if(cone.spot.seenCount > 0) //only do something if there is a spot in this cone
    {
      ObstacleWheel::Spot& spot = cone.spot;
      spot.position =rotationMatrix * spot.position;
      spot.position += translation;
      newSpots.push_back(spot);//this creates a copy
      cone.hasObstacle = false;
      cone.spot.seenCount = 0;
    }
    cone.seenThisFrame = false;
  }

  for(ObstacleWheel::Spot& spot : newSpots)
  {
    const float dist = spot.position.abs();
    if(dist > wheelRadius)
    {//remove points that are too far away
      continue;
    }
    enterSpot(wheel, spot.position, std::max(1, spot.seenCount), spot.seenBy,
              false, 0);
  }
}

void ObstacleWheelProvider::initialize(ObstacleWheel& wheel)
{
  ASSERT(coneWidth > 0);
  ASSERT((360 % coneWidth) == 0);
  coneCount = 360 / coneWidth;
  ASSERT(coneCount > 0);
  wheel.cones.resize(coneCount, ObstacleWheel::Cone());
  ASSERT(static_cast<int>(wheel.cones.size()) == coneCount);
  wheel.coneWidth = fromDegrees(coneWidth);
  wheel.wheelRadius = wheelRadius;
  float currentAngle = 0.f;

  for(ObstacleWheel::Cone& cone : wheel.cones)
  {
    cone.angle = normalize(currentAngle);
    currentAngle += wheel.coneWidth;
  }
}

void ObstacleWheelProvider::enterObstacles(ObstacleWheel& wheel) const
{
  const int obstacleCount = theObstacleSpots.obstacles.size();

  for(int i = 0; i < obstacleCount; ++i)
  {
    //enter a spot into each cone that is inside the banzone
    const ObstacleSpots::Obstacle& obstacle = theObstacleSpots.obstacles[i];
    Vector2<float> leftInField;
    Vector2<float> rightInField;
    Geometry::calculatePointOnField(obstacle.banZoneTopLeft.x, obstacle.banZoneBottomRight.y,
                                    theCameraMatrix, theCameraInfo, leftInField);
    Geometry::calculatePointOnField(obstacle.banZoneBottomRight.x, obstacle.banZoneBottomRight.y,
                                    theCameraMatrix, theCameraInfo, rightInField);
    const int rightConeIndex = calcConeIndex(rightInField, wheel);
    int leftConeIndex = calcConeIndex(leftInField, wheel);

    Geometry::Line obstacleLine(leftInField, leftInField - rightInField);

    while(leftConeIndex != rightConeIndex)
    {
      Vector2<float> centerOfCone(1, 0);
      centerOfCone = centerOfCone.rotate(wheel.cones[leftConeIndex].angle);
      Geometry::Line centerOfConeLine(0, 0, centerOfCone.x, centerOfCone.y);
      Vector2<float> intersection;
      if(Geometry::getIntersectionOfLines(obstacleLine, centerOfConeLine, intersection))
      {
        enterSpot(wheel, intersection, 1, theCameraInfo.camera, true, 1);
      }
      --leftConeIndex;
      leftConeIndex = (leftConeIndex + wheel.cones.size()) % wheel.cones.size(); //mathematically correct modulo
    }
    //last iteration
    enterSpot(wheel, rightInField, 1, theCameraInfo.camera, true, 1);
  }
}

float ObstacleWheelProvider::calcAngleDist(const float dist) const
{
  return pi_2 - std::atan2(theCameraMatrix.translation.z, dist);
}

void ObstacleWheelProvider::decreaseSeenCount(ObstacleWheel& wheel)
{
  if(static_cast<int>(theFrameInfo.time) - lastDecreaseTimestamp >= decreaseSeenTickCount)
  {
    for(ObstacleWheel::Cone& cone : wheel.cones)
    {
      if(cone.spot.seenCount > 0)
      {
        --(cone.spot.seenCount);
        cone.hasObstacle = cone.spot.seenCount > spotIsObstacleCount;
      }
    }
    lastDecreaseTimestamp = theFrameInfo.time;
  }
}

void ObstacleWheelProvider::enterSpot(ObstacleWheel& wheel, const Vector2<>& pInField,
                                      const int initialSeenCount, const CameraInfo::Camera seenBy,
                                      const bool seenThisFrame, const int seenCountIncrement) const
{
  const int coneIndex = calcConeIndex(pInField, wheel);
  ObstacleWheel::Cone& cone = wheel.cones[coneIndex];

  if(seenBy == CameraInfo::upper &&
     cone.spot.seenCount > 0 && cone.spot.seenBy == CameraInfo::lower)
  {
    /*ignore a spot if it was seen by the upper camera and the lower camera
      has already seen a spot in the same cone before*/
    //FIXME hier sollte man noch gucken ob man die Punkte mergen kann, ansonsten verwerfen
    return;
  }

  const float distanceToPoint = pInField.abs();
  if(distanceToPoint > wheelRadius)
  {//skip point if it is too far away
    return;
  }
  const float distanceToPointRad = calcAngleDist(distanceToPoint);

  if(cone.spot.seenCount > 0)
  {//merge the spots
    if(cone.spot.seenCount < maxSeenCount && !cone.seenThisFrame)
    {//only increase count if this is the first time that we see the obstacle in this frame
      cone.spot.seenCount = std::min(cone.spot.seenCount + seenCountIncrement, maxSeenCount);
    }
    float oldFactor = 0;  //factor for the old measurement
    float newFactor = 0;  //the factor that is used to weight the new measurement

    if(cone.angleDistance < distanceToPointRad)
    {
      oldFactor = mergeCloserWeight;
      newFactor = mergeFurtherWeight;
    }
    else
    {
      oldFactor = mergeFurtherWeight;
      newFactor = mergeCloserWeight;
    }
    const float mergeMultiplicator = 1 / (mergeCloserWeight + mergeFurtherWeight);
    cone.spot.position = (cone.spot.position * oldFactor + pInField * newFactor) * mergeMultiplicator;
    cone.distance = cone.spot.position.absFloat();
    cone.angleDistance = calcAngleDist(cone.distance);
    cone.spot.seenBy = seenBy;
  }
  else
  {//enter new obstacle
    cone.spot.position = pInField;
    cone.angleDistance = distanceToPointRad;
    cone.distance = distanceToPoint;
    cone.spot.seenCount = initialSeenCount;
    cone.spot.seenBy = seenBy;
  }
  cone.seenThisFrame = seenThisFrame;
  cone.hasObstacle = cone.spot.seenCount > spotIsObstacleCount;
}

void ObstacleWheelProvider::stuffHoles(ObstacleWheel& wheel) const
{
  std::vector<ObstacleWheel::Cone>::iterator left = wheel.cones.begin();
  std::vector<ObstacleWheel::Cone>::iterator middle = left + 1;
  std::vector<ObstacleWheel::Cone>::iterator right = left + 2;

  while(right != wheel.cones.end())
  {
    stuffHole(left, middle, right);
    ++left;
    ++middle;
    ++right;
  }

  //special cases:
  left = wheel.cones.end() - 1;
  middle = wheel.cones.begin();
  right = middle + 1;
  stuffHole(left, middle, right);

  left = wheel.cones.end() - 2;
  middle = left + 1;
  right = wheel.cones.begin();
  stuffHole(left, middle, right);
}

void ObstacleWheelProvider::stuffHole(std::vector<ObstacleWheel::Cone>::iterator left,
                 std::vector<ObstacleWheel::Cone>::iterator middle,
                 std::vector<ObstacleWheel::Cone>::iterator right) const
{
  if((!middle->hasObstacle) && left->hasObstacle && right->hasObstacle &&
     left->distance < stuffHoleMaxRadius && right->distance < stuffHoleMaxRadius)
  {
    middle->spot.position = (left->spot.position + right->spot.position) * 0.5; //average position
    middle->spot.seenCount = std::min(left->spot.seenCount, right->spot.seenCount); //to avoid circle oscillation
    middle->spot.seenBy = left->spot.seenBy; //this doesn't make any more sense than taking the one from right
    middle->distance = middle->spot.position.absFloat();
    middle->angleDistance = calcAngleDist(middle->distance);
    middle->hasObstacle = middle->spot.seenCount > spotIsObstacleCount;
    middle->seenThisFrame = left->seenThisFrame || right->seenThisFrame;
  }
}

void ObstacleWheelProvider::clearWheel(ObstacleWheel& wheel) const
{
  for(ObstacleWheel::Cone& cone : wheel.cones)
  {
    cone.spot.seenCount = 0;
    cone.hasObstacle = false;
  }
}