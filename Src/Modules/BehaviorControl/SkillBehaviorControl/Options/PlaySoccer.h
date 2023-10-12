/**
 * @file PlaySoccer.h
 *
 * This file defines the root option of the soccer behavior.
 *
 * @author Thomas Röfer
 */

option(PlaySoccer)
{
  initial_state(selectOption)
  {
    action
    {
      select_option(options); //@options
    }
  }
}
