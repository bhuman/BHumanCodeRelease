/**
 * @file DemoSoccer.cpp
 *
 * This file implements the DemoSoccer skill.
 *
 * @author Philip Reichenberg et al.
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoSoccer)
{
  initial_state(searching)
  {
    transition
    {
      if(theRobotHealth.maxJointTemperatureStatus != JointSensorData::TemperatureStatus::regular)
      {
        if(theMotionInfo.isMotion(MotionPhase::stand))
          goto waving;
        else
          goto walkToWaving;
      }
      // Only play the ball if we saw it some time before far away or very recently.
      else if((theFieldBall.ballWasSeen(4000) && theBallModel.estimate.position.squaredNorm() > sqr(500.f)) ||
              theFieldBall.ballWasSeen(2000))
        goto playing;
    }
    action
    {
      DemoSearchForBall();
    }
  }

  state(playing)
  {
    transition
    {
      // If the robot [<< At this point, the author seems to have died. - editor's note]
      if(theRobotHealth.maxJointTemperatureStatus != JointSensorData::TemperatureStatus::regular)
        goto walkToWaving;
      // Do not play the ball if we have not seen it for a while or we have lost it while it was close.
      else if(!theFieldBall.ballWasSeen(4000) ||
              (!theFieldBall.ballWasSeen(2000) && theBallModel.estimate.position.squaredNorm() <= sqr(500.f)))
        goto searching;
    }
    action
    {
      DemoGoToBallAndKick();
    }
  }

  state(walkToWaving)
  {
    transition
    {
      if(state_time > 2000)
        goto standBeforeWaving;
    }
    action
    {
      WalkAtRelativeSpeed({.speed = {0.f, 0.01f, 0.f}});
      LookForward();
    }
  }

  state(standBeforeWaving)
  {
    transition
    {
      if(state_time > 1000)
        goto waving;
    }
    action
    {
      Stand({.high = true});
      LookForward();
    }
  }

  state(waving)
  {
    // This state can't be left on purpose, because it wouldn't be nice if the robot randomly started walking again.
    action
    {
      DemoWave();
    }
  }
}
