/**
 * @file HandleRefereeSignal.h
 *
 * This file defines options that handle the in-game referee detection.
 * A robot only tries to look at the referee if in a preliminary game and the
 * robot stands inside a ring segment surrounding the referee position when a
 * new game state starts. The robot will not try to actively reach such a
 * position. Since the referee signal is not shown earlier than 5 s after the
 * state change, the robot continues its normal behavior for 3 s, before
 * it turns around to look at the referee.  It will then not wait longer than
 * 10 s until it returns to normal game play. If the referee was not detected
 * within the first 6 s, a side step will be performed. The robot usually
 * steps closer to the halfway line to get a more perpendicular view at the
 * referee. However, if already close to the halfway line in the own half, it
 * will walk in the opposite direction to avoid entering the opponent half.
 * The referee signal "substitution" consists of two gestures. It will be
 * detected in sequence. If the first part is detected but the second one is
 * not, a default substitution message is sent, which is at least the correct
 * signal and with a chance of 1/2 also the correct team color.
 * The first part of "substitution" can also be mixed up with one of the
 * two gestures for "fullTime". Therefore, substitution can still result in
 * "fullTime" if that one is detected later.  The gesture for goal can be
 * detected by accident. Therefore, sending is  delayed to give this
 * robot and other robots the chance to come up with a  different signal.
 * If the referee gesture has not been detected within 10 s, the robot will
 * send a wildcard gesture, which grants a 6/13 chance of guessing the right
 * team color. The wildcard is a two handed signal, of which only two exist,
 * so it has a slightly higher chance of being correct if the three
 * categories are used at similar frequencies.
 *
 * @author Thomas Röfer
 */

/**
 * Are we in a situation where a referee signal could be shown?
 * @return Are we?
 */
bool beginOfRefereeSignal() const
{
  return theGameState.competitionPhase == GameState::roundRobin
         && theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) < 2000
         && (((theGameState.isKickOff() || GameState::isKickOff(theExtendedGameState.stateLastFrame))
              && ((theExtendedGameState.wasSet() && theGameState.isPlaying())
                  || (theExtendedGameState.wasPlaying() && theGameState.isReady())))
             || (theExtendedGameState.stateLastFrame != GameState::afterHalf && theGameState.state == GameState::afterHalf));
}

/**
 * Send the actual referee signal and maybe also speak it out load.
 * Usually, speaking should not be impacting the whistle detection,
 * because there should not be another one so shortly after the previous
 * one.
 * @param gesture The referee signal.
 * @param timeAfterWhistle How long ago was the whistle heard (in ms)?
 * @param sayIt Also say the signal.
 */
void sendRefereeSignal(const RefereePercept::Gesture gesture, int timeAfterWhistle, bool sayIt = true)
{
  static const std::regex upperCase("([A-Z])");
  if(sayIt)
  {
    theSaySkill({.text = std::regex_replace(TypeRegistry::getEnumName(gesture), upperCase, " $1"),
                 .force = true});
    ANNOTATION("HandleRefereeSignal", "Sent " << TypeRegistry::getEnumName(gesture));
  }
  theRefereeSignal.signal = gesture;
  theRefereeSignal.timeWhenWhistleWasHeard = theFrameInfo.time - timeAfterWhistle;
  theRefereeSignal.timeWhenLastDetected = theFrameInfo.time;
};

/**
 * Watchdog to send a wildcard signal after 10 seconds.
 * It always stays in its initial state, so that further options
 * in \c select_option are always executed.
 */
option(HandleRefereeWatchdog)
{
  initial_state(watchdog)
  {
    action
    {
      RefereeWatchdog();
    }
  }
}

/**
 * The actual watchdog. If a signal could be detected, it waits for
 * 10 seconds and then sends the wildcard signal.
 */
option(RefereeWatchdog)
{
  initial_state(waiting)
  {
    transition
    {
      if(beginOfRefereeSignal())
      {
        if(theGameState.whistled || theGameState.state == GameState::afterHalf)
          goto delaying;
        else
          goto sending;
      }
    }
  }

  target_state(delaying)
  {
    transition
    {
      if(state_time > 13000)
      {
        if(theFrameInfo.getTimeSince(theRefereeSignal.timeWhenLastDetected) > state_time)
          goto sending;
        else
          goto waiting;
      }
    }
  }

  state(sending)
  {
    transition
    {
      goto waiting;
    }
    action
    {
      sendRefereeSignal(RefereePercept::goalBlue, 13000, false);
    }
  }
}

/**
 * Detect begin of state change and then wait a while. The option reaches
 * its target state if the referee detection should become active.
 */
option(RefereeActivation)
{
  initial_state(waiting)
  {
    transition
    {
      if(beginOfRefereeSignal() && (theGameState.whistled || theGameState.state == GameState::afterHalf))
        goto delaying;
    }
  }

  state(delaying)
  {
    transition
    {
      if(state_time > 3000)
        goto active;
    }
  }

  target_state(active)
  {
    transition
    {
      goto waiting;
    }
  }
}

/**
 * The detection of the referee signal. In only becomes active if the robot is
 * in the right place.
 */
option(HandleRefereeSignal)
{
  const Vector2f refereeOnField(theFieldDimensions.xPosHalfWayLine,
                                (theFieldDimensions.yPosLeftSideline + theFieldDimensions.yPosLeftFieldBorder) / 2.f
                                * (theGameState.leftHandTeam ? 1 : -1));
  const Vector2f refereeOffsetOnField = refereeOnField - theRobotPose.translation;
  const Vector2f refereeOffsetRelative = theRobotPose.inverse() * refereeOnField;

  const auto standAndLook = [&]
  {
    theStandSkill();
    theLookAtPointSkill({.target = {refereeOffsetRelative.x(), refereeOffsetRelative.y(), 1200.f},
                         .camera = HeadMotionRequest::upperCamera});
    theOptionalImageRequest.sendImage = true;
  };

  const auto blocked = [&]()
  {
    // It is important to see the arms next to the body. Assume +/-40 cm around the referee position are enough.
    const Angle refereeEdge1 = (theRobotPose.inverse() * (refereeOnField + Vector2f(400.f, 0.f))).angle();
    const Angle refereeEdge2 = (theRobotPose.inverse() * (refereeOnField - Vector2f(400.f, 0.f))).angle();
    const Rangea referee(std::min(refereeEdge1, refereeEdge2), std::max(refereeEdge1, refereeEdge2));

    // Assume that we need to see the referee from 80 cm upward. We look from a height of 50 cm and other robots
    // have a height of less than 60 cm. So they are in the way if closer than 1/3 of the distance to the referee.
    const float distanceThreshold = refereeOffsetOnField.norm() / 3.f;
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      switch(obstacle.type)
      {
        case Obstacle::someRobot:
        case Obstacle::opponent:
        case Obstacle::teammate:
          if(obstacle.center.norm() < distanceThreshold
             && obstacle.right.angle() < referee.max && obstacle.left.angle() > referee.min)
            return true;
      }
    return false;
  };

  common_transition
  {
    if(option_time > 10000
       || theStrategyStatus.role == ActiveRole::toRole(ActiveRole::playBall)
       || (theStrategyStatus.role == ActiveRole::toRole(ActiveRole::closestToTeammatesBall)
           && theTeammatesBallModel.isValid
           && BallPhysics::getEndPosition(theTeammatesBallModel.position, theTeammatesBallModel.velocity, theBallSpecification.friction).x() < -2000.f)
       || theStrategyStatus.position == Tactic::Position::goalkeeper)
      goto inactive;
  }

  initial_state(inactive)
  {
    transition
    {
      DEBUG_RESPONSE_ONCE("option:HandleRefereeSignal:now")
        goto turnToReferee;
      if(action_done
         && Rangef(2500.f, 6000.f).isInside(refereeOffsetOnField.norm())
         && Rangea(55_deg, 125_deg).isInside(std::abs(refereeOffsetOnField.angle())))
        goto turnToReferee;
    }
    action
    {
      RefereeActivation();
    }
  }

  state(turnToReferee)
  {
    transition
    {
      if(std::abs(refereeOffsetRelative.angle()) < 45_deg)
      {
        ANNOTATION("HandleRefereeSignal", "Look at referee");
        goto lookAtReferee;
      }
    }
    action
    {
      const Angle lookDir = Rangea(-45_deg, 45_deg).clamped(refereeOffsetRelative.angle());
      const Vector2f lookOffset = Pose2f(lookDir) * Vector2f(refereeOffsetRelative.norm(), 0.f);
      theLookAtPointSkill({.target = {lookOffset.x(), lookOffset.y(), 1200.f},
                           .camera = HeadMotionRequest::upperCamera});
      const Angle rotationDiff = refereeOffsetRelative.angle();
      theWalkToPoseSkill({.target = {std::max(0.f, std::abs(rotationDiff) - 43_deg) * sgn(rotationDiff)}});
    }
  }

  state(lookAtReferee)
  {
    transition
    {
      if(option_time > 3000 && !blocked())
      {
        if(theRefereePercept.gesture == RefereePercept::substitution)
          goto detectSecondGesture;
        else if(theRefereePercept.gesture == RefereePercept::goalBlue)
          goto goalBlue;
        else if(theRefereePercept.gesture == RefereePercept::goalRed)
          goto goalRed;
        else if(option_time > 3000 + 1000 * std::abs(std::abs(refereeOffsetOnField.angle()) - 90_deg)
                && theRefereePercept.gesture != RefereePercept::none)
          goto sendSignal;
      }
      if(option_time > 6000)
        goto sideStep;
    }
    action
    {
      standAndLook();
    }
  }

  state(sideStep)
  {
    transition
    {
      if(state_time > 1000)
        goto lookAtRefereeAgain;
    }
    action
    {
      theLookAtPointSkill({.target = {refereeOffsetRelative.x(), refereeOffsetRelative.y(), 1200.f},
                           .camera = HeadMotionRequest::upperCamera});
      const Vector2f robotOffsetOnField = theRobotPose.translation - refereeOnField;
      const float stepDirection = theGameState.leftHandTeam ^ (std::abs(refereeOffsetOnField.angle()) < 80_deg) ? -1.f : 1.f;
      const Angle angle = 250.f / robotOffsetOnField.norm() * stepDirection * static_cast<float>(state_time) / 1000.f;
      const Vector2f targetOnField = refereeOnField + Pose2f(robotOffsetOnField.angle() + angle) * Vector2f(robotOffsetOnField.norm(), 0.f);
      const Vector2f targetRelative = theRobotPose.inverse() * targetOnField;
      theWalkToPoseSkill({.target = {refereeOffsetRelative.angle(), targetRelative.x(), targetRelative.y()}});
      theOptionalImageRequest.sendImage = true;
    }
  }

  state(lookAtRefereeAgain)
  {
    transition
    {
      if(state_time > 1000 && !blocked())
      {
        if(theRefereePercept.gesture == RefereePercept::substitution)
          goto detectSecondGesture;
        else if(theRefereePercept.gesture == RefereePercept::goalBlue)
          goto goalBlue;
        else if(theRefereePercept.gesture == RefereePercept::goalRed)
          goto goalRed;
        else if(theRefereePercept.gesture != RefereePercept::none)
          goto sendSignal;
      }
    }
    action
    {
      standAndLook();
    }
  }

  state(detectSecondGesture)
  {
    transition
    {
      if((state_time > 1000 &&
          (theRefereePercept.gesture == RefereePercept::kickInRed || theRefereePercept.gesture == RefereePercept::kickInBlue))
          || option_time > 9500)
        goto sendSubstitution;
      else if(state_time > 3000 && theRefereePercept.gesture == RefereePercept::fullTime)
        goto sendSignal;
    }
    action
    {
      standAndLook();
    }
  }

  state(goalBlue)
  {
    transition
    {
      if(!blocked() && theRefereePercept.gesture != RefereePercept::none && theRefereePercept.gesture != RefereePercept::goalBlue)
        goto sendSignal;
      else if(option_time > 9500)
        goto sendGoalBlue;
    }
    action
    {
      standAndLook();
    }
  }

  state(goalRed)
  {
    transition
    {
      if(!blocked() && theRefereePercept.gesture != RefereePercept::none && theRefereePercept.gesture != RefereePercept::goalRed)
        goto sendSignal;
      else if(option_time > 9500)
        goto sendGoalRed;
    }
    action
    {
      standAndLook();
    }
  }

  state(sendSignal)
  {
    transition
    {
      goto inactive;
    }
    action
    {
      standAndLook();
      sendRefereeSignal(theRefereePercept.gesture, option_time);
    }
  }

  state(sendSubstitution)
  {
    transition
    {
      goto inactive;
    }
    action
    {
      standAndLook();
      sendRefereeSignal(theRefereePercept.gesture == RefereePercept::kickInRed
                        ? RefereePercept::substitutionRed : RefereePercept::substitutionBlue,
                        option_time);
    }
  }

  state(sendGoalBlue)
  {
    transition
    {
      goto inactive;
    }
    action
    {
      standAndLook();
      sendRefereeSignal(RefereePercept::goalBlue, option_time);
    }
  }

  state(sendGoalRed)
  {
    transition
    {
      goto inactive;
    }
    action
    {
      standAndLook();
      sendRefereeSignal(RefereePercept::goalRed, option_time);
    }
  }
}
