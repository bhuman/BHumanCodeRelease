/** behavior for the ready state */
option(ReadyState)
{
  /* position has been reached -> stand and wait */
  initial_state(stand)
  {
    action
    {
      HeadControlMode(HeadControl::lookForward);
      Stand();
    }
  }
}
