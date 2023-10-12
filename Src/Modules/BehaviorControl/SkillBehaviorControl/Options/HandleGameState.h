option(HandleGameState)
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
      theArmContactSkill();
      if(theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper))
        theArmObstacleAvoidanceSkill();
      if(!select_option(playingOptions)) //@playingOptions
        executeRequest();
    }
  }

  state(initial)
  {
    action
    {
      theLookAtAnglesSkill({.pan = 0_deg,
                            .tilt = 0_deg,
                            .speed = 150_deg});
      theStandSkill({.high = true});
    }
  }

  state(ready)
  {
    action
    {
      theArmContactSkill();
      if(theStrategyStatus.role != PositionRole::toRole(PositionRole::goalkeeper))
        theArmObstacleAvoidanceSkill();
      if(theSkillRequest.skill == SkillRequest::walk)
        theWalkToPointReadySkill({.target = theSkillRequest.target});
      else
      {
        theLookActiveSkill({.ignoreBall = true});
        theStandSkill();
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
        theLookAtPointSkill({.target = (Vector3f() << theRobotPose.inverse() * targetOnField, theBallSpecification.radius).finished()});
      }
      else
        theLookActiveSkill({.ignoreBall = true});
      theStandSkill({.high = true});
    }
  }

  state(finished)
  {
    action
    {
      theLookForwardSkill();
      theStandSkill();
    }
  }
}
