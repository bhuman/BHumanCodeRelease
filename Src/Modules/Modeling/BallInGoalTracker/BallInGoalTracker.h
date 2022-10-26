/**
 * @file BallInGoalTracker.h
 *
 * This file declares a module that identifies the Status of the ball regarding whether it's in a goal.
 *
 * @author Jonah Jaeger
 * @author Yannik Meinken
 */

#pragma once

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallInGoal.h"
#include "Representations/Modeling/RobotPose.h"
#include "Math/RingBufferWithSum.h"

MODULE(BallInGoalTracker,
{,
  USES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  USES(RobotPose),
  PROVIDES(BallInGoal),
  DEFINES_PARAMETERS(
  {,
    (int)(300) changeStatusTriggerTime, /**<  time the ball has to be consistently considered as in goal*/
    (float)(200.f) extraGoalWidth, /**< 20cm added on both sides of the goal to prevent false negatives*/
    (float)(200.f) minDistance, /**< if the robot is nearer than this to the Ball he has to see the ball inside the goal (negative distanceToGoal)*/
    (float)(1500) maxDistanceToGoal, /**< if the ball is further away from the goal it is never considered as in goal*/
    (float)(0.001) convergingRate, /**< how fast converge to maxDistanceToGoal*/
  }),
});

class BallInGoalTracker : public BallInGoalTrackerBase
{
  int lastTimeOutGoal = 0; /**< Time the ball was last not seen the goal */

  //Initial value needed so that there is a valid value at the beginning of the game should be negative
  int stableLastTimeInGoal = 0; /**< Last time the ball was considered as in goal */

  RingBufferWithSum<char, 3> ballBuffer; /**< Buffer witch contains the last three "positions" of the ball */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theWhistle The representation updated.
   */
  void update(BallInGoal& theBallInGoal) override;

  /**
   * Checks whether the ball is currently near to a goal relativ to my distance to the ball
   */
  bool nearGoalNow() const;

  /**
   * Checks whether the ball is currently rolling towards a goal and should arrive it
   */
  bool goalShotNow() const;

  /**
   * Computes the distance from the ball to the nearer goal
   */
  float distanceBallToGoal() const;
};
