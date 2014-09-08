/**
* @file Controller/Platform/Windows/Joystick.h
* Declares a joystick interface class.
* This is the Windows implementation.
* @author Colin Graf
*/

#pragma once

/**
* A joystick interface class.
*/
class Joystick
{
public:
  enum
  {
    numOfAxes = 8, /**< Number of supported axes. */
    numOfButtons = 40, /**< Number of supported buttons. */
  };

  /**
  * Default constructor.
  */
  Joystick();

  /**
  * Destructor; closes the joystick.
  */
  ~Joystick();

  /**
  * Opens the first joystick that is not in use.
  * @return Whether a joystick was opened successfully
  */
  bool init();

  /**
  * Updates the pending events and the state of the axes.
  * @return Whether the update was successful.
  */
  bool update();

  /**
  * Returns and removes the next pending event.
  * @param buttonId The button that caused the event.
  * @param pressed Whether the button was pressed or released.
  * @return Whether there was an pending event or not.
  */
  bool getNextEvent(unsigned int& buttonId, bool& pressed);

  /**
  * Returns the state of an axis.
  * @param axisId The id of the axis which state should be returned.
  * @return The state of the axis in the range [-1...1]
  */
  float getAxisState(unsigned int axisId) const;

  /**
  * Returns the state of a button.
  * @param buttonId The id of the buttton which state should be returned.
  * @return Whether the button is currently pressed or not.
  */
  bool isButtonPressed(unsigned int buttonId) const;

private:

  static unsigned int usedJoysticks; /**< Joystick usage indicator. One bit per joystick. */

  unsigned int joystickId; /**< The id of the joystick. */
  unsigned int buttonEvents[2]; /**< Pending button changed events. One bit per button. */
  unsigned int buttonState[2]; /**< The button pressed states. One bit per button. */
  unsigned int axisState[numOfAxes]; /**< The state of each axis in the range [1...65534?]. */

  /** Deactivates the joystick in order to avoid polling for joystick events */
  void deactivate();
};
