/**
 * @file InWalkKick.cpp
 *
 * This file implements the implementation of the InWalkKick skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(InWalkKickImpl,
{,
  IMPLEMENTS(InWalkKick),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class InWalkKickImpl : public InWalkKickImplBase
{
  option(InWalkKick)
  {
    initial_state(launch)
    {
      transition
      {
        if(theMotionInfo.motion == MotionRequest::walk && theMotionInfo.walkRequest.walkKickRequest == p.walkKick)
          goto execute;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
      }
    }

    state(execute)
    {
      transition
      {
        if(theMotionInfo.walkRequest.walkKickRequest.kickType == WalkKicks::none)
          goto finished;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, false);
      }
    }

    target_state(finished)
    {
      action
      {
        setRequest(p, false);
      }
    }

    aborted_state(fallen)
    {
      transition
      {
        if(theMotionInfo.motion != MotionRequest::fall && (theMotionInfo.motion != MotionRequest::getUp || theGetUpEngineOutput.isLeavingPossible))
          goto launch;
      }
      action
      {
        theMotionRequest.motion = MotionRequest::getUp;
        theLibCheck.inc(LibCheck::motionRequest);
      }
    }
  }

  void setRequest(const InWalkKick& p, bool requestWalkKick)
  {
    theMotionRequest.motion = MotionRequest::walk;
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = p.kickPose;
    theMotionRequest.walkRequest.speed = Pose2f(1.f, 1.f, 1.f);
    theMotionRequest.walkRequest.walkKickRequest = requestWalkKick ? p.walkKick : WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(InWalkKickImpl);
