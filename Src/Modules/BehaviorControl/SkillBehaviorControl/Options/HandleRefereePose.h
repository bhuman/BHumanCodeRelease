/**
 * @file HandleRefereePose.h
 *
 * This file defines an option that controls the robot during the Visual Referee Challenge.
 *
 * @author Sina Schreiber
 * @author Ayleen Lührsen
 * @author Michelle Gusev
 * @author Thomas Röfer
 */

option(HandleRefereePose)
{
  using enum RefereePercept::Gesture;
  using enum ArmKeyFrameRequest::ArmKeyFrameId;

  // Look at referee, stand upright, and announce gesture if known.
  auto gesture = [this](const ArmKeyFrameRequest::ArmKeyFrameId left,
                        const ArmKeyFrameRequest::ArmKeyFrameId right,
                        const std::string& text = "")
  {
    const Vector2f refereePos = theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline);
    theLookAtPointSkill({.target = {refereePos.x(), refereePos.y(), 1200.f},
                         .camera = HeadMotionRequest::upperCamera,
                         .speed = 150_deg});
    theStandSkill({.high = true});
    theKeyFrameSingleArmSkill({.motion = left, .arm = Arms::left});
    theKeyFrameSingleArmSkill({.motion = right, .arm = Arms::right});
    if(!SystemCall::soundIsPlaying() && !text.empty())
      this->theSaySkill({.text = text});
  };

  common_transition
  {
    // Unstiffen the robot brings it back to the game state "initial". Go back to the initial state in that case.
    if(!theGameState.isPlaying())
      goto initial;
    // After a pose was recognized, the robot shall be reset by pressing its forehead button for a second to make it wait for a whistle again.
    else if(theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 1000) 
      goto waitForWhistle;
  }

  /** Waits in a high stand until the robot is switched to the playing state, usually by unpenalizing it. */
  initial_state(initial)
  {
    transition
    {
      if(theGameState.isPlaying())
        goto waitForWhistle;
    }
    action
    {
      theLookAtAnglesSkill({.pan = 0_deg,
                            .tilt = 20_deg,
                            .speed = 150_deg});
      theStandSkill({.high = true});
    }
  }

  /**
   * The state after the robot is unpenalized but isn't acting on any recognized poses yet, since the whistle wasn't
   * blown (or just not heard).
   */
  state(waitForWhistle)
  {
    transition
    {
      if(theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) < state_time)
      {
        theSaySkill({.text = "Let's go!"});
        goto detectGesture;
      }
    }
    action
    {
      gesture(useDefault, useDefault);
    }
  }

  /** Wait a second after the whistle was blown until the referee percept is checked for a detected gesture. */
  state(detectGesture)
  {
    transition
    {
      if(state_time > 1000) // wait for a second until detections are accepted
        switch(theRefereePercept.gesture) // as soon as the refereePercept is assigned a detected pose, go to the according state to act on it
        {
          case goalKickBlue:
            goto goalKickBlue;
          case goalKickRed:
            goto goalKickRed;
          case kickInBlue:
            goto kickInBlue;
          case kickInRed:
            goto kickInRed;
          case cornerKickBlue:
            goto cornerKickBlue;
          case cornerKickRed:
            goto cornerKickRed;
          case goalBlue:
            goto goalBlue;
          case goalRed:
            goto goalRed;
          case pushingFreeKickBlue:
            goto pushingFreeKickBlue;
          case pushingFreeKickRed:
            goto pushingFreeKickRed;
          case fullTime:
            goto fullTime;
        }
    }
    action
    {
      gesture(useDefault, useDefault);
    }
  }

  // States for each pose that can be taken by referee and robot.
  // Each setting the poses of each arm and a voice line to say.
  state(kickInBlue) {action{gesture(armHorizontalSideways, useDefault, "Kick in blue team");}}
  state(kickInRed) {action{gesture(useDefault, armHorizontalSideways, "Kick in red team");}}
  state(goalKickBlue) {action{gesture(arm45degreeUpSideways, useDefault, "Goal kick blue team");}}
  state(goalKickRed) {action{gesture(useDefault, arm45degreeUpSideways, "Goal kick red team");}}
  state(cornerKickBlue) {action{gesture(arm45degreeDownSideways, useDefault, "Corner kick blue team");}}
  state(cornerKickRed) {action{gesture(useDefault, arm45degreeDownSideways, "Corner kick red team");}}
  state(goalBlue) {action{gesture(armHorizontalSideways, arm45degreeUpFront, "Goal blue team");}}
  state(goalRed) {action{gesture(arm45degreeUpFront, armHorizontalSideways, "Goal red team");}}
  state(pushingFreeKickBlue) {action{gesture(armHorizontalSideways, armHandToChest, "Pushing free kick blue team");}}
  state(pushingFreeKickRed) {action{gesture(armHandToChest, armHorizontalSideways, "Pushing free kick red team");}}

  /** This one is a bit different since the arms are moving. */
  state(fullTime)
  {
    action
    {
      // Switch between both poses every 1.5 seconds.
      if(state_time / 1500 & 1)
        gesture(armHandToChest, armHandToChest, "Full time");
      else
        gesture(armHorizontalSideways, armHorizontalSideways, "Full time");
    }
  }
}
