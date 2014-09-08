/**
* @file Controller/Platform/OSX/Joystick.h
* Declares a joystick interface class.
* This is the OSX implementation.
* @author Colin Graf
* @author Thomas Röfer
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
  /**
  * The function is a helper.
  * It is called to collect information about all human interface devices.
  * @param value Address of the enumerated value.
  * @param context Address of the collection the value is added to.
  */
  static void copyCallBack(const void* value, void* context);

  static const void* hidManager; /**< Handle of the human interface manager. */
  static unsigned int nextDevice; /**< Index of the next device to checked for using as joystick. */
  static unsigned int usedJoysticks; /**< Counts the joysticks already used. */

  const void* deviceId; /**< Handle of the joystick. */
  const void* axisIds[numOfAxes]; /**< Handles of all axes. 0 if they do not exist. */
  int axisMin[numOfAxes]; /**< Minimum readings of axes. */
  int axisMax[numOfAxes]; /**< Maximum readings of axes. */
  const void* buttonIds[numOfButtons]; /**< Handles of all buttons. 0 if they do not exist. */
  const void* hatId; /**< Handles of the coolie hat. 0 if it does not exist. */
  int axisState[numOfAxes]; /**< The state of each axis. */
  unsigned buttonState[2]; /**< The button pressed states. One bit per button. */
  unsigned buttonEvents[2]; /**< Pending button changed events. One bit per button. */
};
