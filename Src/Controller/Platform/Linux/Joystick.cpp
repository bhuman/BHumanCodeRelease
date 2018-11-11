/**
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

#include "Controller/Platform/Joystick.h"
#include "Platform/BHAssert.h"

class Joystick::Private
{
public:
  static unsigned int usedJoysticks; /**< Joystick usage indicator. One bit per joystick. */

  int jd = -1; /**< The joystick descriptor. */
  unsigned int joystickId = 0; /**< The id of the joystick. */
  int axisState[Joystick::numOfAxes]; /**< The state of each axis in the range [-32767...32767]. */
  unsigned int buttonState[2]; /**< The button pressed states. One bit per button. */

  ~Private();
};

unsigned int Joystick::Private::usedJoysticks = 0;

Joystick::Private::~Private()
{
  if(jd != -1)
  {
    usedJoysticks &= ~(1 << joystickId);
    close(jd);
  }
}

Joystick::Joystick() : p(new Private()) {}
Joystick::~Joystick() = default;

bool Joystick::init()
{
  ASSERT(p->jd == -1);
  for(int i = 0; i < 32; ++i)
    if(!(Joystick::Private::usedJoysticks & (1 << i)))
    {
      char devname[16];
      std::snprintf(devname, 16, "/dev/input/js%i", i);

      // try to open device
      p->jd = open(devname, O_RDONLY | O_NONBLOCK);
      if(p->jd != -1)
      {
        Joystick::Private::usedJoysticks |= 1 << i;
        p->joystickId = i;
        p->buttonState[0] = p->buttonState[1] = 0;
        for(int i = 0; i < numOfAxes; ++i)
          p->axisState[i] = 0;
        return true;
      }
    }
  return false;
}

bool Joystick::update()
{
  if(p->jd == -1)
    return false;

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(p->jd != -1);
  struct js_event e;
  ssize_t r;
  while((r = read(p->jd, &e, sizeof(struct js_event))) > 0)
  {
    //button press or release event
    if(e.type & JS_EVENT_BUTTON && e.number < numOfButtons)
    {
      buttonId = e.number;
      pressed = e.value ? true : false;
      if(pressed)
        p->buttonState[e.number / 32] |= 1 << (buttonId % 32);
      else
        p->buttonState[e.number / 32] &= ~(1 << (buttonId % 32));
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

      p->axisState[e.number] = e.value;
    }
  }

  return false;
}

float Joystick::getAxisState(unsigned int axisId) const
{
  ASSERT(p->jd != -1);
  ASSERT(axisId < numOfAxes);
  return float(-p->axisState[axisId]) / 32767.f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(p->jd != -1);
  ASSERT(buttonId < numOfButtons);
  return p->buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}
