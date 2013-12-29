/**
* @file Controller/Platform/Linux/Joystick.cpp
* Implementation of the joystick interface class.
* This is the Linux implementation.
* @author Colin Graf
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#include <linux/joystick.h>
#include <unistd.h>
#include <cstdio>

#include "Joystick.h"
#include "Platform/BHAssert.h"

unsigned int Joystick::usedJoysticks = 0;

Joystick::Joystick() : jd(-1) {}

bool Joystick::init()
{
  ASSERT(jd == -1);
  for(int i = 0; i < 32; ++i)
    if(!(usedJoysticks & (1 << i)))
    {
      char devname[16];
      snprintf(devname, 16, "/dev/input/js%i", i);

      // try to open device
      jd = open(devname, O_RDONLY | O_NONBLOCK);
      if(jd != -1)
      {
        usedJoysticks |= 1 << i;
        joystickId = i;
        buttonState[0] = buttonState[1] = 0;
        for(int i = 0; i < numOfAxes; ++i)
          axisState[i] = 0;
        return true;
      }
    }
  return false;
}

Joystick::~Joystick()
{
  if(jd != -1)
  {
    usedJoysticks &= ~(1 << joystickId);
    close(jd);
  }
}

bool Joystick::update()
{
  if(jd == -1)
    return false;

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(jd != -1);
  struct js_event e;
  ssize_t r;
  while((r = read(jd, &e, sizeof(struct js_event))) > 0)
  {
    //button press or release event
    if(e.type & JS_EVENT_BUTTON && e.number < numOfButtons)
    {
      buttonId = e.number;
      pressed = e.value ? true : false;
      if(pressed)
        buttonState[e.number / 32] |= 1 << (buttonId % 32);
      else
        buttonState[e.number / 32] &= ~(1 << (buttonId % 32));
      return true;
    }
    //axis position changed
    else if(e.type & JS_EVENT_AXIS && e.number < numOfAxes)
    {
      // map POV axes on axis 0 and 1
      if(e.number == 4)
        e.number = 0;
      else if(e.number == 5)
        e.number = 1;

      axisState[e.number] = e.value;
    }
  }

  return false;
}

float Joystick::getAxisState(unsigned int axisId) const
{
  ASSERT(jd != -1);
  ASSERT(axisId < numOfAxes);
  return float(-axisState[axisId]) / 32767.f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(jd != -1);
  ASSERT(buttonId < numOfButtons);
  return buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}
