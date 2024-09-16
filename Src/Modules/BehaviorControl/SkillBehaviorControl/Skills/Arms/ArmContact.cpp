/**
 * @file ArmContact.cpp
 *
 * This file implements the ArmContact skills.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) ArmContact,
       args((Arms::Arm) arm))
{
  common_transition
  {
    if(arm == Arms::numOfArms)
      goto both;
    else if(arm == Arms::left)
      goto left;
    else
      goto right;
  }

  initial_state(both)
  {
    action
    {
      ArmContactLeftArm();
      ArmContactRightArm();
    }
  }

  state(left)
  {
    action
    {
      ArmContactLeftArm();
    }
  }

  state(right)
  {
    action
    {
      ArmContactRightArm();
    }
  }
}

option((SkillBehaviorControl) ArmContactLeftArm,
       defs((int)(6000) stayBackTime, /**< The duration for which an arm stays back after contact. */
            (int)(2000) deadTime)) /**< The duration for which new contacts are ignored after the arm has been back. */
{
  const bool hasContact = theArmContactModel.status[Arms::left].contact
                          && (theArmContactModel.status[Arms::left].pushDirection == ArmContactModel::backward
                              || theArmContactModel.status[Arms::left].pushDirection == ArmContactModel::left
                              || theArmContactModel.status[Arms::left].pushDirection == ArmContactModel::right);

  initial_state(armNormal)
  {
    transition
    {
      if(SystemCall::getMode() == SystemCall::simulatedRobot)
        goto ignoreInSimulation;
      else if(hasContact)
        goto armBack;
    }
  }

  state(armBack)
  {
    transition
    {
      if(!hasContact)
        goto keepBack;
    }
    action
    {
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(keepBack)
  {
    transition
    {
      if(hasContact)
        goto armBack;
      else if(state_time > stayBackTime)
        goto delay;
    }
    action
    {
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(delay)
  {
    transition
    {
      if(state_time > deadTime)
        goto armNormal;
    }
  }

  aborted_state(ignoreInSimulation) {}
}

option((SkillBehaviorControl) ArmContactRightArm,
       defs((int)(6000) stayBackTime, /**< The duration for which an arm stays back after contact. */
            (int)(2000) deadTime)) /**< The duration for which new contacts are ignored after the arm has been back. */
{
  const bool hasContact = theArmContactModel.status[Arms::right].contact
                          && (theArmContactModel.status[Arms::right].pushDirection == ArmContactModel::backward
                              || theArmContactModel.status[Arms::right].pushDirection == ArmContactModel::left
                              || theArmContactModel.status[Arms::right].pushDirection == ArmContactModel::right);

  initial_state(armNormal)
  {
    transition
    {
      if(SystemCall::getMode() == SystemCall::simulatedRobot)
        goto ignoreInSimulation;
      else if(hasContact)
        goto armBack;
    }
  }

  state(armBack)
  {
    transition
    {
      if(!hasContact)
        goto keepBack;
    }
    action
    {
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(keepBack)
  {
    transition
    {
      if(hasContact)
        goto armBack;
      else if(state_time > stayBackTime)
        goto delay;
    }
    action
    {
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(delay)
  {
    transition
    {
      if(state_time > deadTime)
        goto armNormal;
    }
  }

  aborted_state(ignoreInSimulation) {}
}
