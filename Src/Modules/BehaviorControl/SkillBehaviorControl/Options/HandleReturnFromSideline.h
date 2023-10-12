option(HandleReturnFromSideline)
{
  const Angle maxAngle = 70_deg;
  const Angle maxSpeed = 50_deg;
  const Angle tilt = 20_deg;

  initial_state(initialized)
  {
    transition
    {
      if((theExtendedGameState.wasPenalized() &&
          theGameState.isPlaying() &&
          theExtendedGameState.playerStateLastFrame != GameState::penalizedIllegalMotionInSet) ||
         (theExtendedGameState.wasInitial() &&
          theGameState.isReady() &&
          theGameState.kickOffSetupFromSidelines))
        goto lookAround;
    }
  }

  state(lookAround)
  {
    transition
    {
      if(state_time > 4 * maxAngle.toDegrees() / maxSpeed.toDegrees() * 1000)
        goto initialized;
    }

    action
    {
      theStandSkill();
      theLookLeftAndRightSkill({.startLeft = true,
                                .maxPan = maxAngle,
                                .tilt = tilt,
                                .speed = maxSpeed});
    }
  }
}
