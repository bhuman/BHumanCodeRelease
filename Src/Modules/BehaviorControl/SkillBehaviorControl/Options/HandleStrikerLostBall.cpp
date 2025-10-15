#include "SkillBehaviorControl.h"
#include "Tools/Math/Transformation.h"

option((SkillBehaviorControl) HandleStrikerLostBall,
       defs((float)(-100.f) minBackwalk,
            (int)(500) minBallDisappearedTime, /**< This option can become active after the ball has disappeared for this time. */
            (float)(700.f) maxDistanceToBall, /**< This option can only become active if the ball is closer than this. */
            (int)(2000) nonStrikerSearch),
       vars((Angle)(0_deg) turnAngle,
            (Pose2f)(0_deg) lastRefOdometry,
            (unsigned)(0) lastTimeStriker,
            (bool)(false) lastWasPlayBall))
{
  bool playBall = theSkillRequest.skill == SkillRequest::shoot || theSkillRequest.skill == SkillRequest::pass
                  || theSkillRequest.skill == SkillRequest::dribble || theSkillRequest.skill == SkillRequest::clear;

  if(playBall)
    lastTimeStriker = theFrameInfo.time;
  if(lastWasPlayBall && !playBall && !theGameState.isFreeKick() && theFrameInfo.getTimeSince(lastTimeStriker) < nonStrikerSearch)
    playBall = true;

  lastWasPlayBall = playBall;

  auto ballIsOccludedByAnObstacle = [this]() -> bool
  {
    const Angle ballAngle = theFieldBall.positionRelative.angle();
    const float ballDistance = theFieldBall.positionRelative.norm();
    for(const Obstacle& o : theObstacleModel.obstacles)
      if(Rangea(o.right.angle(), o.left.angle()).isInside(ballAngle) && o.center.norm() < ballDistance - 100.f)
        return true;
    return false;
  };

  initial_state(knownBallPosition)
  {
    transition
    {
      if(playBall &&
         theFieldBall.timeSinceBallDisappeared > minBallDisappearedTime &&
         theFieldBall.positionRelative.squaredNorm() < sqr(maxDistanceToBall) &&
         !ballIsOccludedByAnObstacle())
        goto searchForBall;
    }
  }

  state(searchForBall)
  {
    transition
    {
      if(!playBall || theFieldBall.ballWasSeen())
        goto knownBallPosition;
      else if(state_time > 1000 || // don't get stuck in backwalking
              (state_time > 500 && (lastRefOdometry.inverse() * theOdometryData).translation.x() < minBackwalk))
      {
        if(turnAngle < 0.f)
          goto turnCW;
        else
          goto turnCCW;
      }
    }
    action
    {
      if(state_time == 0)
      {
        turnAngle = theBallLostModel.relativeAlternateBallDirectionWhenLastSeen;
        lastRefOdometry = theOdometryData;
      }
      LookLeftAndRight({.startLeft = turnAngle > 0});
      WalkAtRelativeSpeed({.speed = {0.f, -1.f, 0.f}});
    }
  }

  state(turnCW)
  {
    transition
    {
      if(!playBall || theFieldBall.ballWasSeen())
        goto knownBallPosition;
    }
    action
    {
      LookForward(); // look slightly right
      WalkAtRelativeSpeed({.speed = {-0.7f, 0.f, 0.f}});
    }
  }
  state(turnCCW)
  {
    transition
    {
      if(!playBall || theFieldBall.ballWasSeen())
        goto knownBallPosition;
    }
    action
    {
      LookForward(); // look slightly left
      WalkAtRelativeSpeed({.speed = {0.7f, 0.f, 0.f}});
    }
  }
}
