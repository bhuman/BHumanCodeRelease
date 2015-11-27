/**
* @file Controller/Platform/Windows/Joystick.cpp
* Implementation of the joystick interface class.
* This is the Windows implementation.
* @author Colin Graf
*/

#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>

#include "Joystick.h"
#include "Platform/BHAssert.h"

unsigned int Joystick::usedJoysticks = 0;

Joystick::Joystick() : joystickId(-1) {}

bool Joystick::init()
{
  ASSERT(joystickId == -1);
  for(int i = 0; i < 32; ++i)
    if(!(usedJoysticks & (1 << i)))
    {
      usedJoysticks |= 1 << i;
      joystickId = i;
      buttonEvents[0] = buttonEvents[1] = buttonState[0] = buttonState[1] = 0;
      for(int i = 0; i < numOfAxes; ++i)
        axisState[i] = 32767;
      return true;
    }
  return false;
}

Joystick::~Joystick()
{
  deactivate();
}

void Joystick::deactivate()
{
  if(joystickId != -1)
  {
    usedJoysticks &= ~(1 << joystickId);
    joystickId = -1;
  }
}

bool Joystick::update()
{
  if(joystickId == -1)
    return false;

  // poll for new events
  JOYINFOEX joy;
  joy.dwSize = sizeof(joy);
  joy.dwFlags = JOY_RETURNALL;
  int r = joyGetPosEx(joystickId, &joy);
  if(r != JOYERR_NOERROR)
  {
    deactivate();
    return false;
  }

  // normal buttons
  buttonEvents[0] |= buttonState[0] ^ joy.dwButtons;
  buttonState[0] = joy.dwButtons;

  // POV buttons
  unsigned int newButtonState1 = joy.dwPOV != 0xffff ? (1 << (joy.dwPOV / 4500)) : 0;
  buttonEvents[1] |= buttonState[1] ^ newButtonState1;
  buttonState[1] = newButtonState1;

  // update axes
  axisState[0] = joy.dwXpos;
  axisState[1] = joy.dwYpos;
  axisState[2] = joy.dwZpos;
  axisState[3] = joy.dwRpos;
  axisState[4] = joy.dwUpos ? joy.dwUpos : 32767;
  axisState[5] = joy.dwVpos ? joy.dwVpos : 32767;

  // map POV on axis 0 and 1
  if(buttonEvents[1] || joy.dwPOV != 0xffff)
  {
    axisState[6] = axisState[7] = 32767;
    if(joy.dwPOV != 0xffff)
    {
      int dir = joy.dwPOV / 4500;
      if(dir >= 7 || dir <= 1)
        axisState[7] = 0;
      if(dir >= 3 && dir <= 5)
        axisState[7] = 65534;
      if(dir >= 1 && dir <= 3)
        axisState[6] = 65534;
      if(dir >= 5 && dir <= 7)
        axisState[6] = 0;
    }
  }

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(joystickId != -1);
  if(buttonEvents[0] == 0 && buttonEvents[1] == 0)
    return false;

  for(int j = 0; j < 2; ++j)
    if(buttonEvents[j])
      for(unsigned int i = 0, events = buttonEvents[j]; i < 32; ++i)
        if(events & (1 << i))
        {
          unsigned int bit = 1 << i;
          buttonEvents[j] &= ~bit;
          pressed = buttonState[j] & bit ? true : false;
          buttonId = i + j * 32;
          if(buttonId < numOfButtons)
            return true;
        }

  ASSERT(false);
  return false;
}

float Joystick::getAxisState(unsigned int axisId) const
{
  ASSERT(joystickId != -1);
  ASSERT(axisId < numOfAxes);
  return (32767 - int(axisState[axisId])) / 32767.f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(joystickId != -1);
  ASSERT(buttonId < numOfButtons);
  return buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}
