/** behavior for the ready state */
option(ReadyState)
{
  initial_state(stand)
  {
    action
    {
      theHeadControlMode = HeadControl::lookForward;
      Stand();
    }
  }
}
