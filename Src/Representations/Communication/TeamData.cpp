/**
 * @file TeamData.cpp
 */

#include "TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Module/Blackboard.h"

#define HANDLE_PARTICLE(particle) case id##particle: return the##particle.handleArbitraryMessage(message, [this](unsigned u){return this->toLocalTimestamp(u);})
bool Teammate::handleMessage(InMessage& message)
{
  switch(message.getMessageID())
  {
      HANDLE_PARTICLE(RobotPose);
      HANDLE_PARTICLE(BallModel);
      HANDLE_PARTICLE(ObstacleModel);
      HANDLE_PARTICLE(BehaviorStatus);
      HANDLE_PARTICLE(Whistle);
      HANDLE_PARTICLE(TeamBehaviorStatus);
      HANDLE_PARTICLE(SideConfidence);
      HANDLE_PARTICLE(FieldCoverage);
      HANDLE_PARTICLE(RobotHealth);
      HANDLE_PARTICLE(TeamTalk);
    default:
      return false;
  }
}

void TeamData::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:TeamData", "drawingOnField");
  for(auto const& teammate : teammates)
  {
    ColorRGBA posCol;
    if(teammate.status == Teammate::PLAYING)
      posCol = ColorRGBA::green;
    else if(teammate.status == Teammate::FALLEN)
      posCol = ColorRGBA::yellow;
    else
      posCol = ColorRGBA::red;

    const Vector2f& rPos = teammate.theRobotPose.translation;
    const float radius = std::max(50.f, teammate.theRobotPose.deviation);
    Vector2f dirPos = teammate.theRobotPose * Vector2f(radius, 0.f);

    // Circle around Player
    CIRCLE("representation:TeamData", rPos.x(), rPos.y(), radius, 20, Drawings::solidPen,
           posCol, Drawings::noBrush, ColorRGBA::white);
    // Direction of the Robot
    LINE("representation:TeamData", rPos.x(), rPos.y(), dirPos.x(), dirPos.y(), 20,
         Drawings::solidPen, posCol);
    // Player number
    DRAWTEXT("representation:TeamData", rPos.x() + 100, rPos.y(), 100, ColorRGBA::black, teammate.number);

    if(teammate.mateType == Teammate::BHumanRobot)
    {
      // Role
      DRAWTEXT("representation:TeamData", rPos.x() + 100, rPos.y() - 150, 100,
               ColorRGBA::black, teammate.theTeamBehaviorStatus.role.getName());

      // Time to reach ball
      int ttrb = teammate.theTeamBehaviorStatus.role.playBall
                 ? static_cast<int>(teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBallStriker)
                 : static_cast<int>(teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall);
      if(Blackboard::getInstance().exists("FrameInfo"))
      {
        const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);
        ttrb = theFrameInfo.getTimeSince(ttrb);
      }
      DRAWTEXT("representation:TeamData", rPos.x() + 100, rPos.y() - 300, 100, ColorRGBA::black, "TTRB: " << ttrb);

      //Line from Robot to WalkTarget
      const Vector2f target = teammate.theRobotPose * teammate.theBehaviorStatus.walkingTo;
      LINE("representation:TeamData", rPos.x(), rPos.y(), target.x(), target.y(),
           10, Drawings::dashedPen, ColorRGBA::magenta);
    }

    // Ball position
    const Vector2f ballPos = teammate.theRobotPose * teammate.theBallModel.estimate.position;
    CIRCLE("representation:TeamData", ballPos.x(), ballPos.y(), 50, 20, Drawings::solidPen, ColorRGBA::yellow, Drawings::solidBrush, ColorRGBA::yellow);
  }
}
