/** This option lets the robot stand up when it has fallen down. */
option(GetUp)
{
  /** Determine falldown direction */
  initial_state(lyingDown)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::onGround && (theFallDownState.direction == FallDownState::back || theFallDownState.direction == FallDownState::front))
        goto lyingOnFrontOrBack;
      else if(theFallDownState.state == FallDownState::onGround && (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right))
        goto lyingOnSide;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }

  /** Get up from back */
  state(lyingOnFrontOrBack)
  {
    transition
    {
      if(action_done)
        goto stand;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      GetUpEngine();
    }
  }
  /** Get up from side
   *  Try to move legs to turn on front or back */
  state(lyingOnSide)
  {
    transition
    {
      if(state_time > 2000)
        goto stand;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      WalkAtSpeedPercentage(Pose2D(0.f, 1.f, 0.f));
    }
  }

  /** Try to stand
   *  If the robot still lies down again, try to
   *  stand up again (this changes only anything if the robot is still on the floor after pick up (and nobody sent penalize and game-state is still playing)*/
  state(stand)
  {
    transition
    {if(theFallDownState.state == FallDownState::onGround && (theFallDownState.direction == FallDownState::back || theFallDownState.direction == FallDownState::front))
        goto lyingOnFrontOrBack;
     else if(theFallDownState.state == FallDownState::onGround && (theFallDownState.direction == FallDownState::left || theFallDownState.direction == FallDownState::right))
        goto lyingOnSide;
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }
}
