/**
 * @file Controller/Platform/OSX/Joystick.cpp
 * Implementation of the joystick interface class.
 * This is the OSX implementation.
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include "Joystick.h"
#include "Platform/BHAssert.h"
#include "Platform/Thread.h"
#include <IOKit/hid/IOHIDLib.h>

static SyncObject _syncObject;
const void* Joystick::hidManager = 0;
unsigned Joystick::usedJoysticks = 0;
unsigned Joystick::nextDevice = 0;

Joystick::Joystick() : deviceId(0), hatId(0)
{
  for(int i = 0; i < numOfAxes; ++i)
    axisIds[i] = 0;
  for(int i = 0; i < numOfButtons; ++i)
    buttonIds[i] = 0;
  for(int i = 0; i < 2; ++i)
    buttonState[i] = buttonEvents[i] = 0;
}

Joystick::~Joystick()
{
  SYNC;
  if(deviceId)
  {
    for(int i = 0; i < numOfAxes; ++i)
      if(axisIds[i])
        CFRelease((IOHIDElementRef) axisIds[i]);

    for(int i = 0; i < numOfButtons; ++i)
      if(buttonIds[i])
        CFRelease((IOHIDElementRef) buttonIds[i]);

    if(hatId)
      CFRelease((IOHIDElementRef) hatId);

    CFRelease((IOHIDDeviceRef) deviceId);
    --usedJoysticks;
  }

  if(hidManager && !usedJoysticks)
  {
    IOHIDManagerClose((IOHIDManagerRef) hidManager, kIOHIDOptionsTypeNone);
    CFRelease((IOHIDManagerRef) hidManager);
    hidManager = 0;
  }
}

bool Joystick::init()
{
  SYNC;
  if(!hidManager)
  {
    VERIFY(hidManager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone));
    IOHIDManagerSetDeviceMatching((IOHIDManagerRef) hidManager, 0);
    IOHIDManagerOpen((IOHIDManagerRef) hidManager, kIOHIDOptionsTypeNone);
    nextDevice = 0;
  }

  CFSetRef copyOfDevices = IOHIDManagerCopyDevices((IOHIDManagerRef) hidManager);
  CFMutableArrayRef devArray = CFArrayCreateMutable(kCFAllocatorDefault, 0, &kCFTypeArrayCallBacks);
  CFSetApplyFunction(copyOfDevices, copyCallBack, (void*) devArray);
  CFRelease(copyOfDevices);

  while(!deviceId && nextDevice < (unsigned) CFArrayGetCount(devArray))
  {
    deviceId = CFArrayGetValueAtIndex(devArray, nextDevice++);
    if(deviceId)
    {
      CFArrayRef elemAry = IOHIDDeviceCopyMatchingElements((IOHIDDeviceRef) deviceId, 0, 0);
      bool isJoystick = false;
      for(int i = 0; !isJoystick && i < (int) CFArrayGetCount(elemAry); ++i)
      {
        IOHIDElementRef elem = (IOHIDElementRef) CFArrayGetValueAtIndex(elemAry, i);
        isJoystick = IOHIDElementGetUsagePage(elem) == kHIDPage_GenericDesktop &&
                     (IOHIDElementGetUsage(elem) == kHIDUsage_GD_Joystick ||
                      IOHIDElementGetUsage(elem) == kHIDUsage_GD_GamePad);
      }
      if(isJoystick)
      {
        CFRetain((IOHIDDeviceRef) deviceId);
        ++usedJoysticks;
        for(int i = 0; i < (int) CFArrayGetCount(elemAry); ++i)
        {
          IOHIDElementRef elem = (IOHIDElementRef) CFArrayGetValueAtIndex(elemAry, i);
          IOHIDElementType elemType = IOHIDElementGetType(elem);

          if(elemType == kIOHIDElementTypeInput_Misc ||
             elemType == kIOHIDElementTypeInput_Button ||
             elemType == kIOHIDElementTypeInput_Axis ||
             elemType == kIOHIDElementTypeInput_ScanCodes)
          {
            if(IOHIDElementGetUsagePage(elem) == kHIDPage_GenericDesktop)
              switch(IOHIDElementGetUsage(elem))
              {
                case kHIDUsage_GD_X:
                case kHIDUsage_GD_Y:
                case kHIDUsage_GD_Z:
                case kHIDUsage_GD_Rx:
                case kHIDUsage_GD_Ry:
                case kHIDUsage_GD_Rz:
                {
                  CFRetain(elem);
                  int axis = IOHIDElementGetUsage(elem) - kHIDUsage_GD_X;
                  axisIds[axis] = elem;
                  axisMin[axis] = (int) IOHIDElementGetLogicalMin(elem);
                  axisMax[axis] = (int) IOHIDElementGetLogicalMax(elem);
                  break;
                }
                case kHIDUsage_GD_Hatswitch:
                  CFRetain(elem);
                  hatId = elem;
                  axisMin[6] = axisMin[7] = -1;
                  axisMax[6] = axisMax[7] = 1;
                  break;
              }
            else if(IOHIDElementGetUsagePage(elem) == kHIDPage_Button)
            {
              CFRetain(elem);
              int button = IOHIDElementGetUsage(elem) - 1;
              if(button >= 0 && button < numOfButtons)
                buttonIds[button] = elem;
            }
          }
        }
      }
      else
        deviceId = 0;
      CFRelease(elemAry);
    }
  }

  CFRelease(devArray);

  return deviceId != 0;
}

bool Joystick::update()
{
  if(!deviceId)
    return false;

  IOHIDValueRef valueRef;

  for(int i = 0; i < numOfAxes; ++i)
    if(axisIds[i])
    {
      IOHIDDeviceGetValue((IOHIDDeviceRef) deviceId, (IOHIDElementRef) axisIds[i], &valueRef);
      axisState[i] = (int) IOHIDValueGetIntegerValue(valueRef);
    }

  unsigned newState = 0;
  for(int i = 0; i < numOfButtons; ++i)
    if(buttonIds[i])
    {
      IOHIDDeviceGetValue((IOHIDDeviceRef) deviceId, (IOHIDElementRef) buttonIds[i], &valueRef);
      newState |= IOHIDValueGetIntegerValue(valueRef) ? 1 << i : 0;
    }
  buttonEvents[0] |= buttonState[0] ^ newState;
  buttonState[0] = newState;

  if(hatId)
  {
    IOHIDDeviceGetValue((IOHIDDeviceRef) deviceId, (IOHIDElementRef) hatId, &valueRef);
    int dir = (int) IOHIDValueGetIntegerValue(valueRef);
    newState = !(dir & 8) ? 1 << dir : 0;
    buttonEvents[1] |= buttonState[1] ^ newState;
    buttonState[1] = newState;
    axisState[6] = axisState[7] = 0;
    if(!(dir & 8))
    {
      if(dir == 7 || dir <= 1)
        axisState[7] = axisMin[7];
      else if(dir >= 3 && dir <= 5)
        axisState[7] = axisMax[7];
      if(dir >= 1 && dir <= 3)
        axisState[6] = axisMax[6];
      else if(dir >= 5 && dir <= 7)
        axisState[6] = axisMin[6];
    }
  }

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(deviceId);
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
  ASSERT(deviceId);
  ASSERT(axisId < numOfAxes);
  if(axisIds[axisId] || (hatId && (0xc0 & 1 << axisId)))
  {
    int middle = (axisMin[axisId] + axisMax[axisId]) / 2;
    int diff = axisState[axisId] - middle;
    if(diff < 0)
      return float(diff) / (axisMin[axisId] - middle);
    else
      return float(diff) / (middle - axisMax[axisId]);
  }
  else
    return 0.0f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(deviceId);
  ASSERT(buttonId < numOfButtons);
  return buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}

void Joystick::copyCallBack(const void* value, void* context)
{
  CFArrayAppendValue((CFMutableArrayRef) context, value);
}
