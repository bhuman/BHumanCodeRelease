/**
 * Declares a joystick interface class.
 * @author Colin Graf
 */

#pragma once

#include <memory>

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

  Joystick();
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
  class Private;
  std::unique_ptr<Private> p;
};
