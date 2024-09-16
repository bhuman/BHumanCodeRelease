#include "SkillBehaviorControl.h"
#include "Tools/Math/Transformation.h"

option((SkillBehaviorControl) HandleStrikerLostBall,
       defs((Angle)(45_deg) minTurn,
            (float)(-100.f) minBackwalk),
       vars((Angle)(0_deg) turnAngle,
            (Pose2f)(0_deg) lastRefOdometry))
{
  const int minBallDisappearedTime = 500; // This option can become active after the ball has disappeared for this time.
  const float maxDistanceToBall = 700.f; // This option can only become active if the ball is closer than this.
  const bool playBall = theSkillRequest.skill == SkillRequest::shoot || theSkillRequest.skill == SkillRequest::pass
                        || theSkillRequest.skill == SkillRequest::dribble || theSkillRequest.skill == SkillRequest::clear;

  initial_state(knownBallPosition)
  {
    transition
    {
      auto ballIsOccludedByAnObstacle = [this]() -> bool
      {
        const Angle ballAngle = theFieldBall.positionRelative.angle();
        const float ballDistance = theFieldBall.positionRelative.norm();
        for(const Obstacle& o : theObstacleModel.obstacles)
          if(Rangea(o.right.angle(), o.left.angle()).isInside(ballAngle) && o.center.norm() < ballDistance - 100.f)
            return true;
        return false;
      };

      if(playBall &&
         theFieldBall.timeSinceBallDisappeared > minBallDisappearedTime &&
         theFieldBall.positionRelative.squaredNorm() < sqr(maxDistanceToBall) &&
         !ballIsOccludedByAnObstacle())
      {
        goto searchForBall;
      }
    }
  }

  state(searchForBall)
  {
    transition
    {
      if(!playBall ||
         theFieldBall.ballWasSeen())
      {
        goto knownBallPosition;
      }
      else if(state_time > 2000 || // don't get stuck in backwalking
              (state_time > 1000 && (lastRefOdometry.inverse() * theOdometryData).translation.x() < minBackwalk))
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
        turnAngle = theBallLostModel.relativeAlternativBallDirectionWhenLastSeen;
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
      if(!playBall ||
         theFieldBall.ballWasSeen())
      {
        goto knownBallPosition;
      }
      if(turnAngle <= 0.f && (theOdometryData.rotation - lastRefOdometry.rotation) < -minTurn)  // Check if odometry changed by > 45_deg
        goto turnCW;
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
      if(!playBall ||
         theFieldBall.ballWasSeen())
      {
        goto knownBallPosition;
      }
      if(turnAngle > 0.f && (theOdometryData.rotation - lastRefOdometry.rotation) > minTurn) // Check if odometry changed by > 45_deg
        goto turnCW;
    }

    action
    {
      LookForward(); // look slightly left
      WalkAtRelativeSpeed({.speed = {0.7f, 0.f, 0.f}});
    }
  }
}
