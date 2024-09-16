#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleGameState)
{
  common_transition
  {
    if(theGameState.isInitial())
      goto initial;
    else if(theGameState.isReady())
      goto ready;
    else if(theGameState.isSet())
      goto set;
    else if(theGameState.isPlaying())
      goto playing;
    else if(theGameState.isFinished())
      goto finished;
    FAIL("Unknown game state.");
  }

  // The default state is "playing".
  initial_state(playing)
  {
    action
    {
      ArmContact();
      if(theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper))
        ArmObstacleAvoidance();
    }
  }

  state(initial)
  {
    action
    {
      LookAtAngles({.pan = 0_deg,
                    .tilt = 0_deg,
                    .speed = 150_deg});
      Stand({.high = true});
    }
  }

  state(ready)
  {
    action
    {
      ArmContact();
      if(theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper))
        ArmObstacleAvoidance();
      if(theSkillRequest.skill == SkillRequest::walk)
        WalkToPointReady({.target = theSkillRequest.target,
                          .reduceWalkingSpeed = ReduceWalkSpeedType::normal}); // TODO maybe more under specific conditions?
      else
      {
        LookActive({.ignoreBall = true});
        Stand();
      }
    }
  }

  state(set)
  {
    action
    {
      if(!theLibDemo.isOneVsOneDemoActive)
      {
        const Vector2f targetOnField = theGameState.isPenaltyKick() ?
                                       Vector2f(theGameState.isForOwnTeam() ?
                                                theFieldDimensions.xPosOpponentPenaltyMark :
                                                theFieldDimensions.xPosOwnPenaltyMark, 0.f) :
                                       Vector2f::Zero();
        LookAtPoint({.target = (Vector3f() << theRobotPose.inverse() * targetOnField, theBallSpecification.radius).finished()});
      }
      else
        LookActive({.ignoreBall = true});
      Stand({.high = true});
    }
  }

  state(finished)
  {
    action
    {
      LookForward();
      Stand();
    }
  }
}
