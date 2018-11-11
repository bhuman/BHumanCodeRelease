/**
 * Sets the head control mode.
 * @param mode The head control mode that is executed by a different
 *             part of the behavior.
 */
option(HeadControlMode, (HeadControl::Mode) mode)
{
  initial_state(set)
  {
    action
    {
      theHeadControlMode = mode;
    }
  }
}
