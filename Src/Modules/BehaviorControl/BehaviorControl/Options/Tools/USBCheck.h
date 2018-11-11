/**
 * An Option to Check the Mounting State of the USB Stick
 */
option(USBCheck)
{
  initial_state(waitForInput)
  {
    transition
    {
      if(theKeyStates.pressed[KeyStates::headMiddle])
        goto waitForRelease;
    }
  }

  state(waitForRelease)
  {
    transition
    {
      if(state_time > 1000)
        goto waitForInput;
      if(!theKeyStates.pressed[KeyStates::headMiddle])
        goto waitForSecondPress;
    }
  }

  state(waitForSecondPress)
  {
    transition
    {
      if(state_time > 200)
        goto waitForInput;
      if(theKeyStates.pressed[KeyStates::headMiddle])
        goto waitForSecondRelease;
    }
  }

  state(waitForSecondRelease)
  {
    transition
    {
      if(state_time > 1000)
        goto waitForInput;
      if(!theKeyStates.pressed[KeyStates::headMiddle])
        goto waitForThirdPress;
    }
  }

  state(waitForThirdPress)
  {
    transition
    {
      if(state_time > 200)
        goto waitForInput;
      if(theKeyStates.pressed[KeyStates::headMiddle])
        goto waitForThirdRelease;
    }
  }

  state(waitForThirdRelease)
  {
    transition
    {
      if(state_time > 1000)
        goto waitForInput;
      if(!theKeyStates.pressed[KeyStates::headMiddle])
        goto checkMount;
    }
  }

  state(checkMount)
  {
    transition
    {
      goto waitForInput;
    }
    action
    {
      if(SystemCall::usbIsMounted())
        SystemCall::playSound("usb_mounted.wav");
      else
        SystemCall::playSound("mounting_failed.wav");
    }
  }
}
