/**
 * The root option sets the fall down states for undefined, upright, and
 * staggering. A fall is handled by the suboption Fall.
 */
option(Root)
{
  common_transition
  {
    if(theMotionInfo.executedPhase == MotionPhase::playDead || theMotionInfo.executedPhase == MotionPhase::prepare)
      goto pickedUp;
  }

  initial_state(pickedUp)
  {
    transition
    {
      if(toUpright)
        goto upright;
      else if(forcePickUpFallTransition)
        goto fall;
    }
    action
    {
      setState(FallDownState::pickedUp);
    }
  }

  state(upright)
  {
    transition
    {
      if(falling)
      {
        goto preFall;
      }
      else if(!torsoUpright)
        goto staggering;
    }
    action
    {
      setState(FallDownState::upright);
    }
  }

  state(staggering)
  {
    transition
    {
      if(falling)
        goto preFall;
      else if(torsoUpright)
        goto upright;
      else if(state_time > maxTimeStaggering)
      {
        ANNOTATION("FallDownStateProvider", "Staggering state kept too long. Resetting filter");
        OUTPUT_WARNING("FallDownStateProvider: Staggering state kept too long. Resetting filter.");
        resetFilter = true;
        goto upright;
      }
    }
    action
    {
      setState(FallDownState::staggering);
    }
  }

  state(preFall)
  {
    transition
    {
      if(falling)
      {
        say("Falling");
        goto fall;
      }
      else if(toUpright && stable)
      {
        say("Upright");
        goto upright;
      }
      else
      {
        goto staggering;
      }
    }
    action
    {
      setState(FallDownState::staggering);
    }
  }

  state(fall)
  {
    transition
    {
      if(toUpright && stable)
      {
        say("Upright");
        goto upright;
      }
      else if(isPickedUp)
      {
        say("Picked up");
        goto pickedUp;
      }
    }
    action
    {
      Fall();
    }
  }
}

/**
 * This option handles a fall and sets the fall down states falling and onGround.
 * If the torso's orientation is big enough, it also updates the odometry to
 * handle situations where the robot is on its side and then rolls on its front
 * or back.
 */
option(Fall)
{
  initial_state(fallingWithoutOdometryUpdate)
  {
    transition
    {
      if(useTorsoOrientation)
        goto fallingWithOdometryUpdate;
    }
    action
    {
      setState(FallDownState::falling, direction);
    }
  }

  state(fallingWithOdometryUpdate)
  {
    transition
    {
      if(!useTorsoOrientation)
        goto fallingWithoutOdometryUpdate;
      else if(stable)
      {
        say("Fallen");
        goto fallen;
      }
    }
    action
    {
      setStateWithPossibleDirectionChange(FallDownState::falling);
    }
  }

  state(fallen)
  {
    action
    {
      setStateWithPossibleDirectionChange(FallDownState::fallen);
    }
  }
}
