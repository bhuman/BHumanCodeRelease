option(ArmContact)
{
  initial_state(main)
  {
    action
    {
      ArmContactLeft();
      ArmContactRight();
    }
  }
}

option(ArmContactLeft, (const int) (6000) actionDelay, (const int) (8000) targetTime)
{
  initial_state(readyForContact)
  {
    transition
    {
      if(theArmContactModel.contactLeft && SystemCall::getMode() != SystemCall::simulatedRobot)
      switch(theArmContactModel.pushDirectionLeft)
      {
        case ArmContactModel::S:
        case ArmContactModel::SW:
        case ArmContactModel::SE:
          goto handleContact;
        default:
          break;
      }
    }
  }

  state(handleContact)
  {
    transition
    {
      if(state_time > targetTime)
      goto recoveryTime;
    }
      action
    {
      KeyFrameLeftArm(ArmKeyFrameRequest::back);
    }
  }

  state(recoveryTime)
  {
    transition
    {
      if(state_time >= actionDelay)
      goto readyForContact;
    }
  }
}

option(ArmContactRight, (const int) (6000) actionDelay, (const int) (8000) targetTime)
{
  initial_state(readyForContact)
  {
    transition
    {
      if(theArmContactModel.contactRight && SystemCall::getMode() != SystemCall::simulatedRobot)
        switch(theArmContactModel.pushDirectionRight)
        {
          case ArmContactModel::S:
          case ArmContactModel::SW:
          case ArmContactModel::SE:
            goto handleContact;
          default:
            break;
        }
    }
  }

  state(handleContact)
  {
    transition
    {
      if(state_time > targetTime)
      goto recoveryTime;
    }
    action
    {
      KeyFrameRightArm(ArmKeyFrameRequest::back);
    }
  }
  
  state(recoveryTime)
  {
    transition
    {
      if(state_time >= actionDelay)
      goto readyForContact;
    }
  }
}