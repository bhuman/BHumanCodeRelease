/**
 * @file TeamData.cpp
 *
 * This representation should soon be obsolete (04.2022).
 */

#include "TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Framework/Blackboard.h"
#include "Debugging/DebugDrawings3D.h"
#include <algorithm>

Vector2f Teammate::getEstimatedPosition(unsigned time) const
{
  return getEstimatedPosition(theRobotPose, theBehaviorStatus.walkingTo, theBehaviorStatus.speed, -theFrameInfo.getTimeSince(time));
}

Vector2f Teammate::getEstimatedPosition(const Pose2f& pose, const Vector2f& target, float speed,
                                        const int timeSinceUpdate)
{
  const float timeSinceLastPacket = static_cast<float>(std::max(0, timeSinceUpdate)) / 1000.f;
  const float distanceToTarget = target.norm(); // Do not overshoot the target.
  const float correctionFactor = 0.5f;
  const Vector2f walkDirectionRelative = target.normalized();
  return pose * (std::min(distanceToTarget, timeSinceLastPacket * correctionFactor * speed) * walkDirectionRelative);
}

void TeamData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamData", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:TeamData:3d", "field");
  for(auto const& teammate : teammates)
  {
    ColorRGBA posCol;
    if(teammate.theRobotStatus.isUpright)
      posCol = ColorRGBA::green;
    else
      posCol = ColorRGBA::yellow;

    const Vector2f& rPos = teammate.theRobotPose.translation;
    const float radius = std::max(50.f, teammate.theRobotPose.getTranslationalStandardDeviation());
    Vector2f dirPos = teammate.theRobotPose * Vector2f(radius, 0.f);

    // Current estimated position
    if(Blackboard::getInstance().exists("FrameInfo"))
    {
      const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
      const Vector2f estimatedPosition = teammate.getEstimatedPosition(theFrameInfo.time);
      CIRCLE("representation:TeamData", estimatedPosition.x(), estimatedPosition.y(), 100, 30, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::red);
      SPHERE3D("representation:TeamData:3d", estimatedPosition.x(), estimatedPosition.y(), 35, 35, ColorRGBA::yellow);
    }

    // Circle around Player
    CIRCLE("representation:TeamData", rPos.x(), rPos.y(), radius, 20, Drawings::solidPen,
           posCol, Drawings::noBrush, ColorRGBA::white);
    // Direction of the Robot
    LINE("representation:TeamData", rPos.x(), rPos.y(), dirPos.x(), dirPos.y(), 20,
         Drawings::solidPen, posCol);
    // Player number
    DRAW_TEXT("representation:TeamData", rPos.x() + 100, rPos.y(), 100, ColorRGBA::black, teammate.number);

    // Role
    DRAW_TEXT("representation:TeamData", rPos.x() + 100, rPos.y() - 150, 100,
             ColorRGBA::black, TypeRegistry::getEnumName(teammate.theStrategyStatus.position));

    // Line from Robot to WalkTarget
    const Vector2f target = teammate.theRobotPose * teammate.theBehaviorStatus.walkingTo;
    LINE("representation:TeamData", rPos.x(), rPos.y(), target.x(), target.y(),
         10, Drawings::dashedPen, ColorRGBA::magenta);

    // Ball position
    const Vector2f ballPos = teammate.theRobotPose * teammate.theBallModel.estimate.position;
    CIRCLE("representation:TeamData", ballPos.x(), ballPos.y(), 50, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::yellow);
  }
}
