/**
 * @file HandleJoystick.cpp
 *
 * This file implements an option that handles joystick input.
 * It takes over if a specific button combination is pressed
 * and it releases control if another combination is pressed.
 *
 * @author Thomas Röfer
 */

#include "SkillBehaviorControl.h"

namespace HandleJoystick
{
  STREAMABLE(Axis,
  {
    Axis() = default;
    Axis(int axis, float deadZone, float scale, float offset)
    : axis(axis) COMMA deadZone(deadZone) COMMA scale(scale) COMMA offset(offset) {}

    float value(const JoystickState& joystickState) const
    {
      const float value = joystickState.axes[axis];
      return offset + (std::abs(value) <= deadZone ? 0.f : (value - deadZone * sgn(value)) * scale / (1.f - deadZone));
    },

    (int) axis, /**< The index of this axis in the joystick state. */
    (float) deadZone, /**< Up to this value, axis measurements are considered to be 0 [0 .. 1[. */
    (float) scale, /**< Values outside the dead zone are scaled by this factor. */
    (float) offset, /**< An offset that is added to the scaled value. */
  });

  STREAMABLE(CombineAxes,
  {
    CombineAxes() = default;
    CombineAxes(int source, int target, float factor)
    : source(source) COMMA target(target) COMMA factor(factor) {}

    void apply(JoystickState& joystickState) const
    {
      joystickState.axes[target] += joystickState.axes[source] * factor;
    },

    (int) source, /**< The axis that is added. */
    (int) target, /**< The axis the source axis is added to. */
    (float) factor, /**< Factor for the source axis before adding. */
  });

  STREAMABLE(ButtonToAxis,
  {
    ButtonToAxis() = default;
    ButtonToAxis(int button, int axis, float value)
    : button(button) COMMA axis(axis) COMMA value(value) {}

    void apply(JoystickState& joystickState) const
    {
      if(joystickState.pressed(button))
        joystickState.axes[axis] = value;
    },

    (int) button, /**< The button to map to an axis. */
    (int) axis, /**< The axis it is mapped to. */
    (float) value, /**< The value that is set if the button is pressed. */
  });

  STREAMABLE(AxisToButton,
  {
    AxisToButton() = default;
    AxisToButton(int axis, const Rangef& values, int button)
    : axis(axis) COMMA values(values) COMMA button(button) {}

    void apply(JoystickState& joystickState) const
    {
      joystickState.buttons &= ~(1 << button);
      if(values.isInside(joystickState.axes[axis]))
        joystickState.buttons |= 1 << button;
    },

    (int) axis, /**< The axis to map to a button. */
    (Rangef) values, /**< The range of values that is mapped to the button. */
    (int) button, /**< The button that is mapped to. */
  });

  STREAMABLE(ButtonToKick,
  {,
    (int) button, /**< The button that is mapped. */
    (KickInfo::KickType) kick, /**< The kick it is mapped to. */
  });
}
using namespace HandleJoystick;

/**
 * This option handles joystick control activation and deactivation.
 * If active, it handles head control itself and lets the next option
 * handle walking and kicking.
 */
option((SkillBehaviorControl) HandleJoystick,
       load((std::vector<CombineAxes>) combineAxes, /**< Each pairs of axes is combined into a single axis. */
            (std::vector<ButtonToAxis>) buttonsToAxes, /**< Map buttons to specific values for axes. */
            (std::vector<AxisToButton>) axesToButtons, /**< Map ranges of axis values to buttons. */
            (int) leftWaveButton, /** Map button to start waving with left arm. */
            (int) rightWaveButton, /** Map button to start waving with right arm. */
            (int) headTiltToggle, /**< Toggle default head tilt. */
            (std::vector<int>) activateButtons, /**< The buttons that activate joystick control. */
            (std::vector<int>) deactivateButtons, /**< The buttons that deactivate joystick control. */
            (int) walkStyleButton), /**< Button to toggle walk mode. */
       vars((bool)(false) headTogglePressed, /**< Is head toggle pressed currently? */
            (bool)(false) isLookDown, /**< Head tilt state. */
            (bool)(false) isOmnidirectionalWalk, /**< Is typical omnidirectional walk active? */
            (bool)(false) walkToggled)) /**< Was walk style toggled? */
{
  // Apply all mappings
  JoystickState joystickState = theJoystickState;
  for(const CombineAxes& combine : combineAxes)
    combine.apply(joystickState);
  for(const ButtonToAxis& buttonToAxis : buttonsToAxes)
    buttonToAxis.apply(joystickState);
  for(const AxisToButton& axisToButton : axesToButtons)
    axisToButton.apply(joystickState);

  if(joystickState.pressed(headTiltToggle))
  {
    if(!headTogglePressed)
    {
      headTogglePressed = true;
      isLookDown = !isLookDown;
    }
  }
  else
    headTogglePressed = false;

  if(joystickState.pressed(walkStyleButton))
  {
    if(!walkToggled)
      isOmnidirectionalWalk = !isOmnidirectionalWalk;
    walkToggled = true;
    if(isOmnidirectionalWalk)
      Say({.text = "Ohmnie Control"});
    else
      Say({.text = "Diff Control"});
  }
  else
    walkToggled = false;

  // Helper for checking for multi-button combinations
  const auto& allPressed = [&](const std::vector<int>& buttons)
  {
    return std::all_of(buttons.begin(), buttons.end(), [&](int button) {return joystickState.pressed(button);});
  };

  // Joystick control is inactive. It can only be activate if the joystick is
  // connected and the robot is not in play dead.
  initial_state(inactive)
  {
    transition
    {
      if(joystickState.valid
         && allPressed(activateButtons)
         && !theMotionInfo.isMotion(MotionPhase::playDead))
      {
        Say({.text = "Remote control activated"});
        goto active;
      }
    }
  }

  // Joystick control is active. Please note that when the joystick
  // connection is lost, the robot stays in this state (and stands).
  state(active)
  {
    transition
    {
      if(allPressed(deactivateButtons))
      {
        Say({.text = "Remote control deactivated"});
        goto inactive;
      }
    }
    action
    {
      BehaviorStateRemoteControlActive();
      HandleJoystickHead({.joystickState = joystickState, .isOmnidirectionalWalk = isOmnidirectionalWalk, .isLookDown = isLookDown, .isKicking = action_done});
      if(joystickState.pressed(leftWaveButton))
        KeyFrameLeftArm({.motion = ArmKeyFrameRequest::waving1});
      if(joystickState.pressed(rightWaveButton))
        KeyFrameRightArm({.motion = ArmKeyFrameRequest::waving1});
      // Must be last, so action_done for head control is set correctly
      HandleJoystickWalk({.joystickState = joystickState, .isOmnidirectionalWalk = isOmnidirectionalWalk});
    }
  }
}

/**
 * The option handles standing, walking, and kicking.
 */
option((SkillBehaviorControl) HandleJoystickWalk,
       args((const JoystickState&) joystickState,
            (bool) isOmnidirectionalWalk),
       load((Axis) forwardAxis, /**< Dead zone and scale for forwards axis. */
            (Axis) sidewaysAxis, /**< Dead zone and scale for sideways axis. */
            (Axis) turnAxis, /**< Dead zone and scale for turn axis. */
            (int) standButton, /**< The button that lets the robot stand. */
            (Angle) maxTurnAngle, /**< Max turn angle for optimized walk. */
            (std::vector<ButtonToKick>) buttonsToKicks, /**< Map buttons to kicks. */
            (float) kickLength), /**< How far shall the kick go? */
       vars((Angle)(0_deg) startRotation)) /**< Ref kick angle. */
{
  const Pose2f walkSpeed(isOmnidirectionalWalk ? turnAxis.value(joystickState) : 0.f,
                         forwardAxis.value(joystickState),
                         sidewaysAxis.value(joystickState));

  KickInfo::KickType kickType = KickInfo::newKick;
  for(const ButtonToKick& buttonToKick : buttonsToKicks)
    if(joystickState.pressed(buttonToKick.button))
      kickType = buttonToKick.kick;

  common_transition
  {
    if(!joystickState.valid || joystickState.pressed(standButton))
      goto stand;
  }

  initial_state(stand)
  {
    transition
    {
      if(walkSpeed != Pose2f())
      {
        if(isOmnidirectionalWalk)
          goto walkOmnidirectional;
        else
          goto walkStraight;
      }
    }
    action
    {
      Stand();
    }
  }

  state(walkOmnidirectional)
  {
    transition
    {
      if(kickType != KickInfo::newKick && theFieldBall.ballWasSeen())
        goto kick;
      else if(!isOmnidirectionalWalk)
        goto walkStraight;
    }
    action
    {
      WalkAtRelativeSpeed({.speed = walkSpeed});
    }
  }

  state(walkStraight)
  {
    transition
    {
      if(kickType != KickInfo::newKick && theFieldBall.ballWasSeen())
        goto kick;
      else if(isOmnidirectionalWalk)
        goto walkOmnidirectional;
    }
    action
    {
      const Angle walkAngle = walkSpeed.translation.angle();
      const Angle rotationSpeed = Rangea(-maxTurnAngle, maxTurnAngle).limit(Rangef::OneRange().limit(std::min(std::abs(walkAngle), Constants::pi - std::abs(walkAngle)) * sgn(walkAngle) * std::sin(std::abs(walkAngle))));
      WalkAtRelativeSpeed({.speed = {rotationSpeed, Rangef::OneRange().limit(Rangef::OneRange().limit(walkSpeed.translation.norm()) * (1.f - std::sin(std::abs(walkAngle))) * (std::abs(walkAngle) > 90_deg ? -1.f : 1.f)), 0.f}});
    }
  }

  target_state(kick)
  {
    transition
    {
      if(action_done || kickType == KickInfo::newKick || !theFieldBall.ballWasSeen())
      {
        if(isOmnidirectionalWalk)
          goto walkOmnidirectional;
        else
          goto walkStraight;
      }
    }
    action
    {
      if(state_time == 0)
        startRotation = theOdometryData.rotation;
      WalkToBallAndKick({.targetDirection = Angle::normalize(startRotation - theOdometryData.rotation),
                         .kickType = kickType,
                         .kickLength = kickLength});
    }
  }
}

/**
 * The option handles standing, walking, and kicking.
 */
option((SkillBehaviorControl) HandleJoystickHead,
       args((const JoystickState&) joystickState,
            (bool) isOmnidirectionalWalk,
            (bool) isLookDown,
            (bool) isKicking),
       load((Axis) tiltAxis, /**< Dead zone and scale for head tilt axis. */
            (Axis) yawAxis, /**< Dead zone and scale for head yaw axis. */
            (Angle) panTiltSpeed, /**< The pan/tilt speed for the head. */
            (float) lowPassFactor), /**< Filter pan/tilt axis values by this factor. */
       vars((Angle)(0_deg) currentTilt, /**< Current head tilt. */
            (Angle)(0_deg) currentYaw)) /**< Current head yaw. */
{
  common_transition
  {
    if(isKicking)
      goto lookActive;
    else if(isLookDown)
      goto lookDown;
    else
      goto lookForward;
  }

  initial_state(lookForward)
  {
    action
    {
      if(state_time == 0 || isOmnidirectionalWalk)
        currentTilt = currentYaw = 0_deg;

      if(!isOmnidirectionalWalk)
      {
        currentTilt = currentTilt * lowPassFactor + tiltAxis.value(joystickState) * (1.f - lowPassFactor);
        currentYaw = currentYaw * lowPassFactor + yawAxis.value(joystickState) * (1.f - lowPassFactor);
      }

      LookAtAngles({.pan = currentYaw,
                    .tilt = currentTilt,
                    .speed = panTiltSpeed});
    }
  }

  state(lookDown)
  {
    action
    {
      LookAtAngles({.pan = 0_deg,
                    .tilt = theHeadLimits.getTiltBound(0_deg).max,
                    .speed = panTiltSpeed});
    }
  }

  state(lookActive)
  {
    action
    {
      LookActive({.withBall = true});
    }
  }
}
