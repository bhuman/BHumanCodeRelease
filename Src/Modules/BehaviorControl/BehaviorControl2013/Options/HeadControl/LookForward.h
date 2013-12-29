/** Pun intended. Looks straight ahead in a way that the Nao V4's cameras cover the area in front of its feet as well as everything else in front of the robot.*/
option(LookForward)
{

  /** Simply sets the necessary angles */
  initial_state(lookForward)
  {
    action
    {
      SetHeadPanTilt(0.f, -0.38f, fromDegrees(150.f));
    }
  }

}
