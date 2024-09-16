/**
 * @file KeyFrameArms.cpp
 *
 * This file implements the KeyFrameArms and KeyFrameSingleArm skills.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) KeyFrameArms,
       args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
            (Arms::Arm) arm,
            (bool) fast))
{
  initial_state(decideArm)
  {
    transition
    {
      if(arm == Arms::numOfArms)
        goto both;
      else if(arm == Arms::left)
        goto left;
      else
        goto right;
    }
  }

  state(both)
  {
    transition
    {
      if(arm == Arms::left)
        goto left;
      else if(arm == Arms::right)
        goto right;
      else if(action_done)
        goto waitForLeft;
    }
    action
    {
      KeyFrameLeftArm({.motion = motion,
                       .fast = fast});
      KeyFrameRightArm({.motion = motion,
                        .fast = fast});
    }
  }

  state(waitForLeft)
  {
    transition
    {
      if(arm == Arms::left)
        goto left;
      else if(arm == Arms::right)
        goto rightDone;
      else if(action_done)
        goto bothDone;
    }
    action
    {
      KeyFrameRightArm({.motion = motion,
                        .fast = fast});
      KeyFrameLeftArm({.motion = motion,
                       .fast = fast});
    }
  }

  target_state(bothDone)
  {
    transition
    {
      if(arm == Arms::left)
        goto leftDone;
      else if(arm == Arms::right)
        goto rightDone;
    }
    action
    {
      KeyFrameLeftArm({.motion = motion,
                       .fast = fast});
      KeyFrameRightArm({.motion = motion,
                        .fast = fast});
    }
  }

  state(left)
  {
    transition
    {
      if(arm == Arms::numOfArms)
        goto both;
      else if(arm == Arms::right)
        goto right;
      else if(action_done)
        goto leftDone;
    }
    action
    {
      KeyFrameLeftArm({.motion = motion,
                       .fast = fast});
    }
  }

  target_state(leftDone)
  {
    transition
    {
      if(arm == Arms::numOfArms)
        goto both;
      else if(arm == Arms::right)
        goto right;
    }
    action
    {
      KeyFrameLeftArm({.motion = motion,
                       .fast = fast});
    }
  }

  state(right)
  {
    transition
    {
      if(arm == Arms::numOfArms)
        goto both;
      else if(arm == Arms::left)
        goto left;
      else if(action_done)
        goto rightDone;
    }
    action
    {
      KeyFrameRightArm({.motion = motion,
                        .fast = fast});
    }
  }

  target_state(rightDone)
  {
    transition
    {
      if(arm == Arms::numOfArms)
        goto both;
      else if(arm == Arms::left)
        goto left;
    }
    action
    {
      KeyFrameRightArm({.motion = motion,
                        .fast = fast});
    }
  }
}

option((SkillBehaviorControl) KeyFrameLeftArm,
       args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
            (bool) fast))
{
  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
  theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].motion = motion;
  theArmMotionRequest.armKeyFrameRequest.arms[Arms::left].fast = fast;

  initial_state(execute)
  {
    transition
    {
      if(theArmMotionInfo.isKeyframeMotion(Arms::left, motion))
        goto done;
    }
  }

  target_state(done) {}
}

option((SkillBehaviorControl) KeyFrameRightArm,
       args((ArmKeyFrameRequest::ArmKeyFrameId) motion,
            (bool) fast))
{
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
  theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].motion = motion;
  theArmMotionRequest.armKeyFrameRequest.arms[Arms::right].fast = fast;

  initial_state(execute)
  {
    transition
    {
      if(theArmMotionInfo.isKeyframeMotion(Arms::right, motion))
        goto done;
    }
  }

  target_state(done) {}
}
