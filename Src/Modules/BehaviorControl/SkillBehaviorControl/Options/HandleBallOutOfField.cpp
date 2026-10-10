#include "SkillBehaviorControl.h"

/**
 * This option stops the robot for a short moment, to give the referee the chance to call stop play and prevent the robot from running out if the field.
 *
 * @author Philip Reichenberg
 */
option((SkillBehaviorControl) HandleBallOutOfField,
       defs((int)(10000) timeBetweenWaits,
            (int)(1000) waitTime,
            (float)(500.f) minBallOutFieldDistance,
            (float)(200.f) minBallOutFieldDistanceSwitchBack,
            (float)(1000.f) maxDistanceToFieldBorder,
            (float)(3000.f) maxBallClippedDistance))
{
  initial_state(ballInField)
  {
    transition
    {
      Vector2f ballOnField = theFieldBall.recentBallPositionOnField();
      Vector2f robotOnField = theRobotPose.translation;
      const float ballClippedDistance = theFieldDimensions.clipToField(ballOnField);
      if(theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall) // is striker
         && theFrameInfo.getTimeSince(lastBallOutOfFieldWaitTimestamp) > timeBetweenWaits // option can be active
         && theFieldDimensions.clipToField(robotOnField, maxDistanceToFieldBorder) > 0.f // robot is near field border
         && ballClippedDistance > minBallOutFieldDistance // ball is far enough outside of the field
         && (ballOnField - robotOnField).squaredNorm() < sqr(maxBallClippedDistance)) // clipped ball and clipped robot are close enough
        goto ballOutOfField;
    }
  }

  state(ballOutOfField)
  {
    transition
    {
      Vector2f ballOnField = theFieldBall.recentBallPositionOnField();
      if(state_time > waitTime || theFieldDimensions.clipToField(ballOnField) < minBallOutFieldDistanceSwitchBack)
        goto ballInField;
    }
    action
    {
      if(state_time == 0)
        lastBallOutOfFieldWaitTimestamp = theFrameInfo.time;
      Stand();
      LookActive({.withBall = true});
    }
  }
}
