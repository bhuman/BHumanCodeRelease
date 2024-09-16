/**
 * @file SharedAutonomyRequest.cpp
 *
 * The file implements the draw method for the representation that contains
 * the requested command in the Shared Autonomy Challenge.
 *
 * @author Thomas Röfer
 */

#include "SharedAutonomyRequest.h"
#include "Debugging/DebugDrawings3D.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Platform/Time.h"
#include "Representations/Sensing/RobotModel.h"

void SharedAutonomyRequest::draw() const
{
  DEBUG_DRAWING3D("representation:SharedAutonomyRequest", "robot")
  {
    if(Blackboard::getInstance().exists("BallModel")
       && Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel")
       && Blackboard::getInstance().exists("RobotPose"))
    {
      const BallModel& theBallModel = static_cast<const BallModel&>(Blackboard::getInstance()["BallModel"]);
      if((controlOperations == playBallDribbleInDirection
          || controlOperations == dribbleInDirection
          || controlOperations == playBallKickToPoint
          || controlOperations == passToPoint) && Time::getTimeSince(theBallModel.timeWhenLastSeen) < 5000)
      {
        const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
        const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
        const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
        TRANSLATE3D("representation:SharedAutonomyRequest", 0, 0, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
        ROTATE3D("representation:SharedAutonomyRequest", -orientation.x(), -orientation.y(), -orientation.z());

        const RobotPose& theRobotPose = static_cast<const RobotPose&>(Blackboard::getInstance()["RobotPose"]);
        const Vector2f target = controlOperations == playBallDribbleInDirection || controlOperations == dribbleInDirection
                                ? Pose2f(targetPose.rotation - theRobotPose.rotation) * Vector2f(750.f, 0.f)
                                : theRobotPose.inverse() * targetPose.translation - theBallModel.estimate.position;
        const float distance = target.norm();
        const Pose2f ball(target.angle(), theBallModel.estimate.position);
        const Vector2f points[] =
        {
          ball * Vector2f(0, 25.f),
          ball * Vector2f(0, -25.f),
          ball * Vector2f(distance - 50.f, -25.f),
          ball * Vector2f(distance - 50.f, 25.f),
          ball * Vector2f(distance, 0)
        };
        QUAD3D("representation:SharedAutonomyRequest",
               Vector3f(points[0].x(), points[0].y(), 3.f),
               Vector3f(points[1].x(), points[1].y(), 3.f),
               Vector3f(points[2].x(), points[2].y(), 3.f),
               Vector3f(points[3].x(), points[3].y(), 3.f),
               ColorRGBA::orange);
        QUAD3D("representation:SharedAutonomyRequest",
               Vector3f(points[3].x(), points[3].y(), 3.f),
               Vector3f(points[2].x(), points[2].y(), 3.f),
               Vector3f(points[4].x(), points[4].y(), 3.f),
               Vector3f(points[4].x(), points[4].y(), 3.f),
               ColorRGBA::orange);
      }
    }
  }

  COMPLEX_DRAWING3D("representation:GameState")
  {
    if(allowGoalKicks)
    {
      LINE3D("representation:GameState", -870, 0, 800, -670, 0, 800, 2, ColorRGBA::white);
      LINE3D("representation:GameState", -770, 0, 900, -770, 0, 700, 2, ColorRGBA::white);
    }
    if(teammatePlaysBall)
    {
      LINE3D("representation:GameState", -100, 0, -620, 0, 0, -820, 3, ColorRGBA::white);
      LINE3D("representation:GameState", 100, 0, -620, 0, 0, -820, 3, ColorRGBA::white);
    }
  }

  DEBUG_DRAWING("representation:SharedAutonomyRequest", "drawingOnField")
  {
    if(mousePressed && controlOperations == Operations::walkToTarget)
    {
      ARROW("representation:SharedAutonomyRequest", targetPose.translation.x(), targetPose.translation.y(), arrowPosition.x(), arrowPosition.y(), 20, Drawings::solidPen, ColorRGBA::black);
    }
    CROSS("representation:SharedAutonomyRequest", targetPose.translation.x(), targetPose.translation.y(), 50, 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0));
    Vector2f rotation(500.f, 0.f);
    rotation.rotate(targetPose.rotation);
    ARROW("representation:SharedAutonomyRequest", targetPose.translation.x(), targetPose.translation.y(), targetPose.translation.x() + rotation.x(), targetPose.translation.y() + rotation.y(), 0, Drawings::solidPen, ColorRGBA(0xcd, 0, 0, 127));
  }
}
