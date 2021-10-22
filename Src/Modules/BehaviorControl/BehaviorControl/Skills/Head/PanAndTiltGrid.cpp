/**
 * @file PanAndTiltGrid.cpp
 *
 * This file implements a skill that moves the head in steps between the maximum pan and tilt, relative to the current orientation.
 *
 * @author Lukas Plecher
 */

#include "Modules/BehaviorControl/BehaviorControl/Skills/Head/HeadOrientation.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(PanAndTiltGridImpl,
{,
  IMPLEMENTS(PanAndTiltGrid),
  CALLS(LookAtAngles),
  REQUIRES(HeadLimits),
});

class PanAndTiltGridImpl : public PanAndTiltGridImplBase
{
  HeadOrientation currentHeadTarget;

  bool panHeadRight = false;
  bool tiltHeadDown = false;

  option(PanAndTiltGrid)
  {
    const auto getPanOffset = [&p, this]()
    {
      return currentHeadTarget.pan - p.original.pan;
    };

    const auto getTiltOffset = [&p, this]()
    {
      return currentHeadTarget.tilt - p.original.tilt;
    };

    const auto setPanOffset = [&p, this](const Angle& offset)
    {
      currentHeadTarget.pan = p.original.pan + offset;
    };

    const auto setTiltOffset = [&p, this](const Angle& offset)
    {
      currentHeadTarget.tilt = p.original.tilt + offset;
    };

    const auto moveHeadTo = [&p, this](const HeadOrientation& h)
    {
      theLookAtAnglesSkill(h.pan, h.tilt, p.speed, HeadMotionRequest::upperCamera);
    };

    initial_state(initial)
    {
      transition
      {
        if(state_time) goto panHead;
      }
      action
      {
        currentHeadTarget = p.original;
        moveHeadTo(currentHeadTarget);
      }
    }

    state(panHead)
    {
      transition
      {
        goto waitAfterPan;
      }
      action
      {
        if(getPanOffset() > p.maximum.pan || currentHeadTarget.pan > theHeadLimits.maxPan())
        {
          panHeadRight = true;
          setPanOffset(0_deg);
        }
        currentHeadTarget.pan += panHeadRight ? -p.panStep : p.panStep;

        moveHeadTo(currentHeadTarget);
      }
    }
    state(tiltHead)
    {
      transition
      {
        goto waitAfterTilt;
      }
      action
      {
        if(getTiltOffset() < -p.maximum.tilt || currentHeadTarget.tilt < theHeadLimits.getTiltBound(currentHeadTarget.pan).min)
        {
          tiltHeadDown = true;
          setTiltOffset(0_deg);
        }
        currentHeadTarget.tilt += tiltHeadDown ? p.tiltStep : -p.tiltStep;

        moveHeadTo(currentHeadTarget);
      }
    }
    state(waitAfterPan)
    {
      transition
      {
        if(state_time > p.waitInPosition)
        {
          if(getPanOffset() < -p.maximum.pan || currentHeadTarget.pan < theHeadLimits.minPan())
          {
            panHeadRight = false;
            setPanOffset(0_deg);
            goto done;
          }
          goto panHead; //for now, turn head only to the left and right. TODO: the states tiltHead and waitAfterTilt are now unused
        }
      }
      action {
        moveHeadTo(currentHeadTarget);
      }
    }
    state(waitAfterTilt) {
      transition
      {
        if(state_time > p.waitInPosition)
        {
          if(getTiltOffset() > p.maximum.tilt || currentHeadTarget.tilt > theHeadLimits.getTiltBound(currentHeadTarget.pan).max)
          {
            tiltHeadDown = false;
            setTiltOffset(0_deg);
            goto panHead;
          }
          goto tiltHead;
        }
      }
      action
      {
        moveHeadTo(currentHeadTarget);
      }
    }
    target_state(done)
    {
      transition
      {
      }
      action
      {
        moveHeadTo(currentHeadTarget);
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(PanAndTiltGridImpl);
