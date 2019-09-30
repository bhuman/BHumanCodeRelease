/**
 * The option fills a request for the UNSW walk generator based on the current B-Human
 * motion request.
 */
option(UpdateRequest)
{
  const WalkRequest& w = theMotionRequest.walkRequest;

  initial_state(standing)
  {
    transition
    {
      if(theMotionRequest.motion == MotionRequest::walk
         && theLegMotionSelection.ratios[MotionRequest::walk] == 1.f
         && theGroundContactState.contact
         && (theFallDownState.state == FallDownState::upright
             || theFallDownState.state == FallDownState::staggering))
      {
        if(theFallDownState.state == FallDownState::staggering)
          ANNOTATION("UNSWWalkingEngine", std::string("Stand to ") + TypeRegistry::getEnumName(w.walkKickRequest.kickType) + " while staggering");
        if(w.walkKickRequest.kickType != WalkKicks::Type::none)
          goto inWalkKick;
        else
          goto walking;
      }
    }
    action
    {
      stand();
    }
  }

  state(walking)
  {
    transition
    {
      if(!theGroundContactState.contact
         || (theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::staggering))
        goto noGroundContact;
      else if(theMotionRequest.motion != MotionRequest::walk && theWalkGenerator.t == 0)
        goto standing;
      else if(w.walkKickRequest.kickType != WalkKicks::Type::none)
        goto inWalkKick;
    }
    action
    {
      walkRequest = theMotionRequest.walkRequest;
      lastTimeWalking = theFrameInfo.time;
      Walking();
    }
  }

  state(inWalkKick)
  {
    transition
    {
      if(!theGroundContactState.contact
         || (theFallDownState.state != FallDownState::upright && theFallDownState.state != FallDownState::staggering))
        goto noGroundContact;
      else if(action_done && theWalkGenerator.t == 0)
      {
        if(theMotionRequest.motion == MotionRequest::walk)
          goto walking;
        else
          goto standing;
      }
    }
    action
    {
      InWalkKick();
    }
  }

  state(noGroundContact)
  {
    transition
    {
      goto standing;
    }
    action
    {
      stand();
    }
  }
}
