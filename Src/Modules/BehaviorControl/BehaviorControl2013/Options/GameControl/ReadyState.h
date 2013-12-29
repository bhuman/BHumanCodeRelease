/** behavior for the ready state */
option(ReadyState)
{
  initial_state(stand)
  {
    transition
    {
    }
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }
}