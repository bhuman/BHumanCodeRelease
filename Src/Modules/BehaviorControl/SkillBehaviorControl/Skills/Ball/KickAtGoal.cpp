/**
 * @file KickAtGoal.cpp
 *
 * This file defines an implementation of the KickAtGoal skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include <limits>

namespace KickAtGoal
{
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };

  STREAMABLE(KickParameter,
  {,
    (KickInfo::KickType) kickType, /**< The kick type. */
    (Angle) minSectorWidth, /**< The minimal required sector width to allow this kick. */
  });
}
using namespace KickAtGoal;

option((SkillBehaviorControl) KickAtGoal,
       vars((Angle)(0_deg) kickAngleInField,
            (KickInfo::KickType)(KickInfo::rlKickDynamic) kickType))
{
  common_transition
  {
    if(!(theGameState.isFreeKick() && theGameState.isForOwnTeam()))
      goto execute;
  }

  initial_state(execute)
  {
    transition
    {
      if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
      {
        if(theKickAtGoalEvaluation.kickInfo.isValid && theIndirectKick.allowDirectKick)
          goto kick;
        else if(theClearTarget.isValid)
          goto clear;
        else
          goto dribble;
      }
    }
    action
    {
      theKickAtGoalEvaluation.update(); // Necessary in case shooting is forced by behavior.
      if(theKickAtGoalEvaluation.kickInfo.isValid && theIndirectKick.allowDirectKick)
      {
        GoToBallAndKick({.targetDirection = theKickAtGoalEvaluation.kickInfo.kickDirection,
                         .kickType = theKickAtGoalEvaluation.kickInfo.kickType});
      }
      else if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
      {
        theClearTarget.update();
        if(theClearTarget.isValid)
        {
          ClearBall();
        }
        else
          DribbleToGoal();
      }
      else
        DribbleToGoal();
    }
  }

  state(kick)
  {
    action
    {
      if(state_time == 0)
      {
        kickAngleInField = theKickAtGoalEvaluation.kickInfo.kickDirection + theRobotPose.rotation;
        kickType = theKickAtGoalEvaluation.kickInfo.kickType;
      }
      GoToBallAndKick({.targetDirection = kickAngleInField - theRobotPose.rotation,
                       .kickType = kickType});
    }
  }

  state(clear)
  {
    action
    {
      ClearBall();
    }
  }

  state(dribble)
  {
    action
    {
      DribbleToGoal();
    }
  }
};
