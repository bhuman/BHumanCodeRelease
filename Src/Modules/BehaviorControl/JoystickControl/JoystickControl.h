/**
 * @file JoystickControl.h
 *
 * This file declares a module that controls the robot through a joystick.
 * It can either provide a motion request and a head motion request or
 * a shared autonomy request (not both at the same time).
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/BehaviorControl/SharedAutonomyRequest.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(JoystickControl,
{,
  REQUIRES(BallModel),
  REQUIRES(ExtendedGameState),
  REQUIRES(GameState),
  REQUIRES(JoystickState),
  REQUIRES(OdometryData),
  REQUIRES(RobotPose),
  REQUIRES(SharedAutonomyRequest2),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  PROVIDES(SharedAutonomyRequest),
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
    (int) scoreButton, /**< Toggle allowing the teammate to score goals.  */
    (int) passButton, /**< The robot passes to its teammate while this button is pressed. */
    (int) dribbleButton, /**< The robot dribbles in the direction indicated by the forward/sideways axes. */
    (int) kickButton, /**< The robot kicks in the direction indicated by the forward/sideways axes. */
    (int) forceButton, /**< Enforce passing, dribbling, or kicking, skipping Zweikampf. */
    (Angle) directionThreshold, /**< The minimum change in forward/sideways axes to adapt the dribble/kick direction. */
    (bool) passThroughSharedAutonomyRequest, /**< Just pass through the shared autonomy request? */
  }),
});

class JoystickControl : public JoystickControlBase
{
  HeadMotionRequest headMotionRequest; /**< The head motion request provided. */
  unsigned lastButtons = 0; /**< The buttons that were pressed during the previous execution. */
  JoystickState joystickState; /**< The joystick state with mapped buttons and axis. */
  MotionRequest motionRequest; /**< Internal motion request if the shared autonomy request is provided. */
  Angle direction = 0_deg; /**< The absolute dribble/kick direction used during the previous execution. */
  bool passThrough;

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

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSharedAutonomyRequest The representation updated.
   */
  void update(SharedAutonomyRequest& theSharedAutonomyRequest) override;

public:
  /**
   * Constructor.
   * Selects standing as initial motion, which is only relevant if the shared
   * autonomy request is provided.
   */
  JoystickControl() : passThrough(passThroughSharedAutonomyRequest) {motionRequest.motion = MotionRequest::stand;}
};
