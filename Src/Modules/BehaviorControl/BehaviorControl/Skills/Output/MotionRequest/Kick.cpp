/**
 * @file Kick.cpp
 *
 * This file implements the implementation of the Kick skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/KickRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include <algorithm>

SKILL_IMPLEMENTATION(KickImpl,
{,
  IMPLEMENTS(Kick),
  REQUIRES(GetUpEngineOutput),
  REQUIRES(KickEngineOutput),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class KickImpl : public KickImplBase
{
  option(Kick)
  {
    initial_state(launch)
    {
      transition
      {
        if(theMotionInfo.motion == MotionRequest::kick && theMotionInfo.kickRequest.kickMotionType == p.kickType)
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
        if(theKickEngineOutput.isLeavingPossible)
          goto finished;
        if(theMotionInfo.motion == MotionRequest::fall)
          goto fallen;
      }
      action
      {
        setRequest(p, true);
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

  void setRequest(const Kick& p, bool requestKick)
  {
    if(requestKick)
      theMotionRequest.motion = MotionRequest::kick;
    else
      theMotionRequest.motion = MotionRequest::stand;
    theMotionRequest.kickRequest.kickMotionType = p.kickType;
    theMotionRequest.kickRequest.mirror = p.mirror;
    theMotionRequest.kickRequest.armsBackFix = p.armsBackFix;
    theMotionRequest.kickRequest.dynPoints.clear();
    theLibCheck.inc(LibCheck::motionRequest);
  }
};

MAKE_SKILL_IMPLEMENTATION(KickImpl);
