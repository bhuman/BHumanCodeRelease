/**
 * @file JoystickControl.h
 *
 * This file declares a module that controls the robot through a joystick.
 * It provides a motion request and a head motion request.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(JoystickControl,
{,
  REQUIRES(BallModel),
  REQUIRES(JoystickState),
  REQUIRES(OdometryData),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  LOADS_PARAMETERS(
  {
    STREAMABLE(Axis,
    {
      Axis() = default;
      Axis(int axis, float deadZone, float scale, float offset)
        : axis(axis) COMMA deadZone(deadZone) COMMA scale(scale) COMMA offset(offset) {}

      float value(JoystickState& joystickState) const
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
    }),

    (std::vector<CombineAxes>) combineAxes, /**< Each pairs of axes is combined into a single axis. */
    (std::vector<ButtonToAxis>) buttonsToAxes, /**< Map buttons to specific values for axes. */
    (std::vector<AxisToButton>) axesToButtons, /**< Map ranges of axis values to buttons. */
    (Axis) forwardAxis, /**< Dead zone and scale for forwards axis. */
    (Axis) sidewaysAxis, /**< Dead zone and scale for sideways axis. */
    (Axis) turnAxis, /**< Dead zone and scale for turn axis. */
    (Axis) panAxis, /**< Dead zone and scale for head pan axis. */
    (Axis) tiltAxis, /**< Dead zone and scale for head tilt axis. */
    (Angle) panTiltSpeed, /**< The pan/tilt speed for the head. */
    (int) sitButton, /**< The button that sits down the robot. */
    (int) standButton, /**< The button that lets the robot stand (high on the second press). */
    (std::vector<ButtonToKick>) buttonsToKicks, /**< Map buttons to kicks. */
  }),
});

class JoystickControl : public JoystickControlBase
{
  HeadMotionRequest headMotionRequest; /**< The head motion request provided. */
  unsigned lastButtons = 0; /**< The buttons that were pressed during the previous execution. */
  JoystickState joystickState; /**< The joystick state with mapped buttons and axis. */

  const Rangef deadZone = Rangef(-0.2f, 0.2f); /**< Only start walking if the speed is high enough. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theHeadMotionRequest The representation updated.
   */
  void update(HeadMotionRequest& theHeadMotionRequest) override {theHeadMotionRequest = headMotionRequest;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theMotionRequest The representation updated.
   */
  void update(MotionRequest& theMotionRequest) override;
};
