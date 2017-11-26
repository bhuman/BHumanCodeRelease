/**
* request to point at the specific point with an arm
*     the arm is selected automatically
*
* @param localPoint, the point that should be pointed
*                    relative to the robot field position
*/

option(PointAt, (Vector3f) localPoint)
{
  initial_state(setRequest)
  {
    transition
    {
      if(localPoint.x() > 0)
        goto left;
      else
        goto right;
    }
  }

  state(left)
  {
    transition
    {
      if(localPoint.y() < -50)
        goto right;
    }
    action
    {
      PointAtWithArm(localPoint, Arms::left);
    }
  }

  state(right)
  {
    transition
    {
      if(localPoint.y() > 50)
      goto left;
    }
      action
    {
      PointAtWithArm(localPoint, Arms::right);
    }
  }
}

/** 
  * request an specific arm to point at the specific point
  * @param localPoint, the point that should be pointed
  *                    relative to the robot field position
  * @param arm, the arm that should be used
  */
option(PointAtWithArm, (Vector3f) localPoint, ((Arms) Arm) arm)
{
  initial_state(setRequest)
  {
    action
    {
      theArmMotionRequest.armMotion[arm] = ArmMotionRequest::pointAt;
      theArmMotionRequest.pointToPointAt = localPoint;
    }
  }
}
