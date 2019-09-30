/**
 * @file Controller/Platform/macOS/Joystick.cpp
 * Implementation of the joystick interface class.
 * This is the macOS implementation.
 * @author Colin Graf
 * @author Thomas Röfer
 */

#include "Controller/Platform/Joystick.h"
#include "Platform/BHAssert.h"
#include "Platform/Thread.h"
#include <IOKit/hid/IOHIDLib.h>

static DECLARE_SYNC;

class Joystick::Private
{
public:
  static const void* hidManager; /**< Handle of the human interface manager. */
  static unsigned int nextDevice; /**< Index of the next device to checked for using as joystick. */
  static unsigned int usedJoysticks; /**< Counts the joysticks already used. */

  const void* deviceId = nullptr; /**< Handle of the joystick. */
  const void* axisIds[Joystick::numOfAxes]; /**< Handles of all axes. 0 if they do not exist. */
  int axisMin[Joystick::numOfAxes]; /**< Minimum readings of axes. */
  int axisMax[Joystick::numOfAxes]; /**< Maximum readings of axes. */
  const void* buttonIds[numOfButtons]; /**< Handles of all buttons. 0 if they do not exist. */
  const void* hatId = nullptr; /**< Handles of the coolie hat. 0 if it does not exist. */
  int axisState[Joystick::numOfAxes]; /**< The state of each axis. */
  unsigned buttonState[2]; /**< The button pressed states. One bit per button. */
  unsigned buttonEvents[2]; /**< Pending button changed events. One bit per button. */

  Private();
  ~Private();

  /**
   * The function is a helper.
   * It is called to collect information about all human interface devices.
   * @param value Address of the enumerated value.
   * @param context Address of the collection the value is added to.
   */
  static void copyCallBack(const void* value, void* context);

  /**
   * The function is a helper.
   * It is called to create a dictionary to filter for certain devices.
   * @param page The page value.
   * @param usage The usage value.
   * @return The dictionary requiring that both input values are met.
   */
  static CFDictionaryRef copyDevicesMask(const unsigned& page, const unsigned& usage);
};

const void* Joystick::Private::hidManager = nullptr;
unsigned int Joystick::Private::nextDevice = 0;
unsigned int Joystick::Private::usedJoysticks = 0;

Joystick::Private::Private()
{
  for(int i = 0; i < numOfAxes; ++i)
    axisIds[i] = 0;
  for(int i = 0; i < numOfButtons; ++i)
    buttonIds[i] = 0;
  for(int i = 0; i < 2; ++i)
    buttonState[i] = buttonEvents[i] = 0;
}

Joystick::Private::~Private()
{
  SYNC;
  if(deviceId)
  {
    for(int i = 0; i < numOfAxes; ++i)
      if(axisIds[i])
        CFRelease(static_cast<IOHIDElementRef>(const_cast<void*>(axisIds[i])));

    for(int i = 0; i < numOfButtons; ++i)
      if(buttonIds[i])
        CFRelease(static_cast<IOHIDElementRef>(const_cast<void*>(buttonIds[i])));

    if(hatId)
      CFRelease(static_cast<IOHIDElementRef>(const_cast<void*>(hatId)));

    CFRelease(static_cast<IOHIDDeviceRef>(const_cast<void*>(deviceId)));
    --usedJoysticks;
  }

  if(hidManager && !usedJoysticks)
  {
    IOHIDManagerClose(static_cast<IOHIDManagerRef>(const_cast<void*>(hidManager)), kIOHIDOptionsTypeNone);
    CFRelease(static_cast<IOHIDManagerRef>(const_cast<void*>(hidManager)));
    hidManager = nullptr;
  }
}

Joystick::Joystick() : p(new Private()) {}
Joystick::~Joystick() = default;

bool Joystick::init()
{
  SYNC;
  if(!p->hidManager)
  {
    VERIFY(p->hidManager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone));
    CFDictionaryRef maskArray[2];
    maskArray[0] = Joystick::Private::copyDevicesMask(kHIDPage_GenericDesktop, kHIDUsage_GD_Joystick);
    maskArray[1] = Joystick::Private::copyDevicesMask(kHIDPage_GenericDesktop, kHIDUsage_GD_GamePad);
    CFArrayRef mask = CFArrayCreate(NULL, (const void**)maskArray, 2, NULL);
    IOHIDManagerSetDeviceMatchingMultiple(static_cast<IOHIDManagerRef>(const_cast<void*>(p->hidManager)), mask);
    CFRelease(mask);
    CFRelease(maskArray[0]);
    CFRelease(maskArray[1]);
    IOHIDManagerOpen(static_cast<IOHIDManagerRef>(const_cast<void*>(p->hidManager)), kIOHIDOptionsTypeNone);
    p->nextDevice = 0;
  }

  CFSetRef copyOfDevices = IOHIDManagerCopyDevices(static_cast<IOHIDManagerRef>(const_cast<void*>(p->hidManager)));
  if(copyOfDevices)
  {
    CFMutableArrayRef devArray = CFArrayCreateMutable(kCFAllocatorDefault, 0, &kCFTypeArrayCallBacks);
    CFSetApplyFunction(copyOfDevices, Joystick::Private::copyCallBack, static_cast<void*>(devArray));
    CFRelease(copyOfDevices);

    while(!p->deviceId && p->nextDevice < static_cast<unsigned>(CFArrayGetCount(devArray)))
    {
      p->deviceId = CFArrayGetValueAtIndex(devArray, p->nextDevice++);
      if(p->deviceId)
      {
        CFArrayRef elemAry = IOHIDDeviceCopyMatchingElements(static_cast<IOHIDDeviceRef>(const_cast<void*>(p->deviceId)), nullptr, kIOHIDOptionsTypeNone);
        if(elemAry)
        {
          if(CFArrayGetCount(elemAry))
          {
            CFRetain(static_cast<IOHIDDeviceRef>(const_cast<void*>(p->deviceId)));
            ++(p->usedJoysticks);
            for(int i = 0; i < static_cast<int>(CFArrayGetCount(elemAry)); ++i)
            {
              IOHIDElementRef elem = static_cast<IOHIDElementRef>(const_cast<void*>(CFArrayGetValueAtIndex(elemAry, i)));
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
                      p->axisIds[axis] = elem;
                      p->axisMin[axis] = static_cast<int>(IOHIDElementGetLogicalMin(elem));
                      p->axisMax[axis] = static_cast<int>(IOHIDElementGetLogicalMax(elem));
                      break;
                    }
                    case kHIDUsage_GD_Hatswitch:
                      CFRetain(elem);
                      p->hatId = elem;
                      p->axisMin[6] = p->axisMin[7] = -1;
                      p->axisMax[6] = p->axisMax[7] = 1;
                      break;
                  }
                else if(IOHIDElementGetUsagePage(elem) == kHIDPage_Button)
                {
                  int button = IOHIDElementGetUsage(elem) - 1;
                  if(button >= 0 && button < numOfButtons)
                  {
                    CFRetain(elem);
                    p->buttonIds[button] = elem;
                  }
                }
              }
            }
          }
          else
            p->deviceId = 0;
          CFRelease(elemAry);
        }
      }
    }

    CFRelease(devArray);
  }

  return p->deviceId != 0;
}

bool Joystick::update()
{
  if(!p->deviceId)
    return false;

  IOHIDValueRef valueRef;

  for(int i = 0; i < numOfAxes; ++i)
    if(p->axisIds[i])
    {
      IOHIDDeviceGetValue(static_cast<IOHIDDeviceRef>(const_cast<void*>(p->deviceId)),
                          static_cast<IOHIDElementRef>(const_cast<void*>(p->axisIds[i])), &valueRef);
      p->axisState[i] = static_cast<int>(IOHIDValueGetIntegerValue(valueRef));
    }

  unsigned newState = 0;
  for(int i = 0; i < numOfButtons; ++i)
    if(p->buttonIds[i])
    {
      IOHIDDeviceGetValue(static_cast<IOHIDDeviceRef>(const_cast<void*>(p->deviceId)),
                          static_cast<IOHIDElementRef>(const_cast<void*>(p->buttonIds[i])), &valueRef);
      newState |= IOHIDValueGetIntegerValue(valueRef) ? 1 << i : 0;
    }
  p->buttonEvents[0] |= p->buttonState[0] ^ newState;
  p->buttonState[0] = newState;

  if(p->hatId)
  {
    IOHIDDeviceGetValue(static_cast<IOHIDDeviceRef>(const_cast<void*>(p->deviceId)),
                        static_cast<IOHIDElementRef>(const_cast<void*>(p->hatId)), &valueRef);
    int dir = static_cast<int>(IOHIDValueGetIntegerValue(valueRef));
    newState = !(dir & 8) ? 1 << dir : 0;
    p->buttonEvents[1] |= p->buttonState[1] ^ newState;
    p->buttonState[1] = newState;
    p->axisState[6] = p->axisState[7] = 0;
    if(!(dir & 8))
    {
      if(dir == 7 || dir <= 1)
        p->axisState[7] = p->axisMin[7];
      else if(dir >= 3 && dir <= 5)
        p->axisState[7] = p->axisMax[7];
      if(dir >= 1 && dir <= 3)
        p->axisState[6] = p->axisMax[6];
      else if(dir >= 5 && dir <= 7)
        p->axisState[6] = p->axisMin[6];
    }
  }

  return true;
}

bool Joystick::getNextEvent(unsigned int& buttonId, bool& pressed)
{
  ASSERT(p->deviceId);
  if(p->buttonEvents[0] == 0 && p->buttonEvents[1] == 0)
    return false;

  for(int j = 0; j < 2; ++j)
    if(p->buttonEvents[j])
      for(unsigned int i = 0, events = p->buttonEvents[j]; i < 32; ++i)
        if(events & (1 << i))
        {
          unsigned int bit = 1 << i;
          p->buttonEvents[j] &= ~bit;
          pressed = p->buttonState[j] & bit ? true : false;
          buttonId = i + j * 32;
          if(buttonId < numOfButtons)
            return true;
        }

  ASSERT(false);
  return false;
}

float Joystick::getAxisState(unsigned int axisId) const
{
  ASSERT(p->deviceId);
  ASSERT(axisId < numOfAxes);
  if(p->axisIds[axisId] || (p->hatId && (0xc0 & 1 << axisId)))
  {
    int middle = (p->axisMin[axisId] + p->axisMax[axisId]) / 2;
    int diff = p->axisState[axisId] - middle;
    if(diff < 0)
      return float(diff) / (p->axisMin[axisId] - middle);
    else
      return float(diff) / (middle - p->axisMax[axisId]);
  }
  else
    return 0.0f;
}

bool Joystick::isButtonPressed(unsigned int buttonId) const
{
  ASSERT(p->deviceId);
  ASSERT(buttonId < numOfButtons);
  return p->buttonState[buttonId / 32] & (1 << (buttonId % 32)) ? true : false;
}

void Joystick::Private::copyCallBack(const void* value, void* context)
{
  CFArrayAppendValue(static_cast<CFMutableArrayRef>(context), value);
}

CFDictionaryRef Joystick::Private::copyDevicesMask(const unsigned& page, const unsigned& usage)
{
  CFMutableDictionaryRef dictionary = CFDictionaryCreateMutable(kCFAllocatorDefault, 2, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

  // Add the page value.
  CFNumberRef value = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &page);
  CFDictionarySetValue(dictionary, CFSTR(kIOHIDDeviceUsagePageKey), value);
  CFRelease(value);

  // Add the usage value (which is only valid if page value exists).
  value = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &usage);
  CFDictionarySetValue(dictionary, CFSTR(kIOHIDDeviceUsageKey), value);
  CFRelease(value);

  return dictionary;
}
