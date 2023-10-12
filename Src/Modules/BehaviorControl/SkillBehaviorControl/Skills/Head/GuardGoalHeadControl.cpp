/**
 * @file GuardGoalHeadControl.cpp
 *
 * This file implements an implementation for the GuardGoalHeadControl skill.
 * It will always try to keep the ball in the image. If the ball is not very far away,
 * approaching the own goal or not seen, the ball (or team ball) is kept focused by a camera.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/TeammatesBallModel.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(GuardGoalHeadControlImpl,
{,
  IMPLEMENTS(GuardGoalHeadControl),
  CALLS(LookActive),
  CALLS(LookAtBall),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(TeammatesBallModel),
  DEFINES_PARAMETERS(
  {,
    (int)(7000) lookAtGlobalThreshold, /**< If the ball is not seen for this amount of time, the global ball is used. */
    (int)(6000) maxLookActiveTime, /**< The maximum time to look active (i.e. not keeping the ball focused). */
    (int)(3000) minLookAtBallTime, /**< The minimum time to look at the ball. */
    (float)(500.f) ballIsFarHighThreshold, /**< If the ball is further in the opponent half than this, looking active becomes possible. */
    (float)(100.f) ballIsFarLowThreshold, /**< If the ball is closer in the own half than this, looking active is disallowed. */
    (float)(1800.f) approachingDistance, /**< Distance of keeper to the ball's end position to consider as 'ball is approaching'. */
  }),
});

class GuardGoalHeadControlImpl : public GuardGoalHeadControlImplBase
{
  option(GuardGoalHeadControl)
  {
    initial_state(lookAtBall)
    {
      transition
      {
        if((!theFieldBall.ballWasSeen(lookAtGlobalThreshold) || theFieldBall.timeSinceBallDisappeared > 1000) &&
           theTeammatesBallModel.isValid)
          goto lookAtGlobal;

        if(state_time > minLookAtBallTime)
        {
          const Vector2f ballOnField = theFieldBall.recentBallPositionOnField();
          if(ballOnField.x() > ballIsFarHighThreshold && !isBallApproaching())
            goto lookActive;
        }
      }
      action
      {
        theLookAtBallSkill();
      }
    }

    state(lookActive)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(lookAtGlobalThreshold))
          goto lookAtGlobal;

        const Vector2f ballOnField = theFieldBall.recentBallPositionOnField();
        if(state_time > maxLookActiveTime || ballOnField.x() < ballIsFarLowThreshold || isBallApproaching())
          goto lookAtBall;
      }
      action
      {
        theLookActiveSkill({.withBall = true});
      }
    }

    state(lookAtGlobal)
    {
      transition
      {
        if(theFieldBall.ballWasSeen(100))
          goto lookAtBall;
      }
      action
      {
        theLookAtGlobalBallSkill();
      }
    }
  }

  /* code is from 2013 (BossPositionProvider) by Simon Taddiken */
  bool isBallApproaching() const
  {
    const Vector2f theBallPositionOnField(theFieldBall.recentBallPositionOnField(3000));
    Vector2f theBallEndPositionOnField, theBallEndPositionRelative;
    theFieldBall.recentBallEndPositions(theBallEndPositionOnField, theBallEndPositionRelative, 3000);

    // if estimated end position of ball is less than 10 cm away from position its current position (in x direction on field), return false
    if(std::abs((theBallPositionOnField - theBallEndPositionOnField).x()) < 100.f)
      return false;

    // Check, if ball intersect center line
    if(theBallPositionOnField.x() >= 0.f && theBallEndPositionOnField.x() < 0.f)
      return true;

    // check whether the end position is within a certain distance to us
    return theBallEndPositionRelative.norm() < approachingDistance;
  }
};

MAKE_SKILL_IMPLEMENTATION(GuardGoalHeadControlImpl);
