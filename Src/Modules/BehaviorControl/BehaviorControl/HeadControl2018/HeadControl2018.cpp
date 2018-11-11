/**
 * BehaviorControl 2018 implementation of HeadControl
 */

#include "HeadControl2018.h"

MAKE_BEHAVIOR_OPTION(HeadControl2018);

HeadControl2018::HeadControl2018() :
  Cabsl<HeadControl2018>(BehaviorOptionRegistry::theInstance->theActivationGraph)
{
}

void HeadControl2018::execute()
{
  beginFrame(theFrameInfo.time, false);
  if(!theGroundContactState.contact && theGameInfo.state != STATE_INITIAL)
  {
    LookForward();
  }
  else
  {
    switch(theHeadControlMode)
    {
      case HeadControl::off:
        SetHeadPanTilt(JointAngles::off, JointAngles::off, 0.f);
        break;
      case HeadControl::lookForward:
        LookForward();
        break;
      default:
        break;
    }
  }
  endFrame();
}
