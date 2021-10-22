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
#include "Tools/RingBufferWithSum.h"

MODULE(BallInGoalTracker,
{,
  USES(FieldBall),
  PROVIDES(BallInGoal),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  DEFINES_PARAMETERS(
  {,
    (int)(300) changeStatusTriggerTime, /**<  */
  }),
});

class BallInGoalTracker : public BallInGoalTrackerBase
{
  int lastTimeOutGoal = 0; /**< Time the ball was last not seen the goal */

  //Initial value needed so that there is a valid value at the beginning of the game should be negative
  int stableLastTimeInGoal = INT_MIN/2; /**< Last time the ball was considered as in goal */

  RingBufferWithSum<char, 3> ballBuffer; /**< Buffer witch contains the last three "positions" of the ball */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theWhistle The representation updated.
   */
  void update(BallInGoal& theBallInGoal) override;

  /**
   * Checks whether the ball is currently in a goal
   */
  bool checkInGoalNow() const;

  /**
   * Checks whether the ball is currently rolling towards a goal and should arrive it
   */
  bool goalShotNow() const;

  /**
   * Checks whether the ball is currently in a goal area or in the goal
   */
  bool nearGoalNow() const;
};
