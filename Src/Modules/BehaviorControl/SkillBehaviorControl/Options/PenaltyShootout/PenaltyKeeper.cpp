#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/Interception.h"

option((SkillBehaviorControl) PenaltyKeeper)
{
  // Martin Kroker, 14.04.2013?
  auto penaltyKeeperShouldCatchBall = [&]
  {
    const float offsetToYAxis = std::max(0.f, (theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalArea, 0.f)).x());

    const Vector2f& ballPositionRel = theBallModel.estimate.position;
    const Vector2f& ballVelocityRel = theBallModel.estimate.velocity;
    // Return false if the ball does not move or moves away from the robot.
    if(ballVelocityRel.x() >= 0.f)
      return false;
    // Return false if the ball has already intersected the line.
    if((ballPositionRel.x() < 0.f && offsetToYAxis < ballPositionRel.x()) ||
       (ballPositionRel.x() > 0.f && offsetToYAxis > ballPositionRel.x()))
      return false;

    Vector2f ballToIntersectPositionVector = theFieldInterceptBall.intersectionPositionWithOwnYAxis - ballPositionRel;
    const float vectorLengthFactor = (ballPositionRel.x() - offsetToYAxis) / ballToIntersectPositionVector.x();
    ballToIntersectPositionVector *= vectorLengthFactor;
    const float s = ballToIntersectPositionVector.norm();
    const float a = theBallSpecification.friction;
    ASSERT(a < 0.f);
    const float timeToIntersectYAxis = BallPhysics::timeForDistance(ballVelocityRel, s, a);

    return between<float>(timeToIntersectYAxis, 0.01f, 10.5f) && theFieldBall.ballWasSeen(500) && (theGameState.isPenaltyShootout() || theFieldBall.isRollingTowardsOwnGoal);
  };

  initial_state(initial)
  {
    transition
    {
      if(penaltyKeeperShouldCatchBall())
        goto intercept;
    }
    action
    {
      LookForward();
      //Stand(); // TODO test if we want to take the risk of walking
      Dive({.request = MotionRequest::Dive::prepare});
    }
  }

  state(intercept)
  {
    transition
    {
      if(action_done || theBallModel.estimate.velocity.x() >= 0.f)
        goto done;
    }
    action
    {
      // Don't add other interception methods here until InterceptBall can handle them.
      const unsigned interceptionMethods = bit(Interception::jumpLeft) | bit(Interception::jumpRight);// | bit(Interception::walk);
      InterceptBall({.interceptionMethods = interceptionMethods,
                     .allowGetUp = !theGameState.isPenaltyShootout(),
                     .allowDive = theBehaviorParameters.keeperJumpingOn});
    }
  }

  target_state(done)
  {
    action
    {
      // Don't add other interception methods here until InterceptBall can handle them.
      const unsigned interceptionMethods = bit(Interception::jumpLeft) | bit(Interception::jumpRight);// | bit(Interception::walk);
      InterceptBall({.interceptionMethods = interceptionMethods,
                     .allowGetUp = !theGameState.isPenaltyShootout(),
                     .allowDive = theBehaviorParameters.keeperJumpingOn});
    }
  }
}
