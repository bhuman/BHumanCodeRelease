/**
 * @file Controller/Platform/Windows/Joystick.cpp
 * Implementation of the joystick interface class.
 * This is the Windows implementation.
 * @author Colin Graf
 */

#include <Windows.h>
#include <mmsystem.h>

#include "Controller/Platform/Joystick.h"
#include "Platform/BHAssert.h"

class Joystick::Private
{
public:
  static unsigned int usedJoysticks; /**< Joystick usage indicator. One bit per joystick. */

  unsigned int joystickId = -1; /**< The id of the joystick. */
  unsigned int buttonEvents[2]; /**< Pending button changed events. One bit per button. */
  unsigned int buttonState[2]; /**< The button pressed states. One bit per button. */
  unsigned int axisState[Joystick::numOfAxes]; /**< The state of each axis in the range [1...65534?]. */

  ~Private();

  void deactivate();
};

unsigned int Joystick::Private::usedJoysticks = 0;

Joystick::Private::~Private()
{
  deactivate();
}

void Joystick::Private::deactivate()
{
  if(joystickId != -1)
  {
    usedJoysticks &= ~(1 << joystickId);
    joystickId = -1;
  }
}

Joystick::Joystick() : p(new Private()) {}
Joystick::~Joystick() = default;

bool Joystick::init()
{
  ASSERT(p->joystickId == -1);
  for(int i = 0; i < 32; ++i)
  {
    if(!(Private::usedJoysticks & (1 << i)))
    {
      p->usedJoysticks |= 1 << i;
      p->joystickId = i;
      p->buttonEvents[0] = p->buttonEvents[1] = p->buttonState[0] = p->buttonState[1] = 0;
      for(int i = 0; i < numOfAxes; ++i)
        p->axisState[i] = 32767;
      return true;
    }
  }
  return false;
}

bool Joystick::update()
{
  if(p->joystickId == -1)
    return false;

  // poll for new events
  JOYINFOEX joy;
  joy.dwSize = sizeof(joy);
  joy.dwFlags = JOY_RETURNALL;
  int r = joyGetPosEx(p->joystickId, &joy);
  if(r != JOYERR_NOERROR)
  {
    p->deactivate();
    return false;
  }

  // normal buttons
  p->buttonEvents[0] |= p->buttonState[0] ^ joy.dwButtons;
  p->buttonState[0] = joy.dwButtons;

  // POV buttons
  unsigned int newButtonState1 = joy.dwPOV != 0xffff ? (1 << (joy.dwPOV / 4500)) : 0;
  p->buttonEvents[1] |= p->buttonState[1] ^ newButtonState1;
  p->buttonState[1] = newButtonState1;

  // update axes
  p->axisState[0] = joy.dwXpos;
  p->axisState[1] = joy.dwYpos;
  p->axisState[2] = joy.dwZpos;
  p->axisState[3] = joy.dwRpos;
  p->axisState[4] = joy.dwUpos ? joy.dwUpos : 32767;
  p->axisState[5] = joy.dwVpos ? joy.dwVpos : 32767;

  // map POV on axis 0 and 1
  if(p->buttonEvents[1] || joy.dwPOV != 0xffff)
  {
    p->axisState[6] = p->axisState[7] = 32767;
    if(joy.dwPOV != 0xffff)
    {
      int dir = joy.dwPOV / 4500;
      if(dir >= 7 || dir <= 1)
        p->axisState[7] = 0;
      if(dir >= 3 && dir <= 5)
        p->axisState[7] = 65534;
      if(dir >= 1 && dir <= 3)
        p->axisState[6] = 65534;
      if(dir >= 5 && dir <= 7)
        p->axisState[6] = 0;
    }
  }

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(p->joystickId != -1);
  if(p->buttonEvents[0] == 0 && p->buttonEvents[1] == 0)
    return false;

  for(int j = 0; j < 2; ++j)
  {
    if(p->buttonEvents[j])
    {
      for(unsigned int i = 0, events = p->buttonEvents[j]; i < 32; ++i)
      {
        if(events & (1 << i))
        {
          unsigned int bit = 1 << i;
          p->buttonEvents[j] &= ~bit;
          pressed = p->buttonState[j] & bit ? true : false;
          buttonId = i + j * 32;
          if(buttonId < numOfButtons)
            return true;
        }
      }
    }
  }

  ASSERT(false);
  return false;
}

float Joystick::getAxisState(unsigned int axisId) const
{
  ASSERT(p->joystickId != -1);
  ASSERT(axisId < numOfAxes);
  return (32767 - int(p->axisState[axisId])) / 32767.f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(p->joystickId != -1);
  ASSERT(buttonId < numOfButtons);
  return p->buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}
