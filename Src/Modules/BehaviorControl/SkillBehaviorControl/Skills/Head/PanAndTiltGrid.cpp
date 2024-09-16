/**
 * @file PanAndTiltGrid.cpp
 *
 * This file implements a skill that moves the head in steps between the maximum pan and tilt, relative to the current orientation.
 *
 * @author Lukas Plecher
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PanAndTiltGrid,
       args((const HeadOrientation&) original,
            (const HeadOrientation&) maximum,
            (const Angle&) panStep,
            (const Angle&) tiltStep,
            (int) waitInPosition,
            (Angle) speed),
       vars((HeadOrientation)(original) currentHeadTarget,
            (bool)(false) panHeadRight,
            (bool)(false) tiltHeadDown))
{
  const auto getPanOffset = [&]
  {
    return currentHeadTarget.pan - original.pan;
  };

  const auto getTiltOffset = [&]
  {
    return currentHeadTarget.tilt - original.tilt;
  };

  const auto setPanOffset = [&](const Angle& offset)
  {
    currentHeadTarget.pan = original.pan + offset;
  };

  const auto setTiltOffset = [&](const Angle& offset)
  {
    currentHeadTarget.tilt = original.tilt + offset;
  };

  const auto moveHeadTo = [&](const HeadOrientation& h)
  {
    LookAtAngles({.pan = h.pan,
                  .tilt = h.tilt,
                  .speed = speed,
                  .camera = HeadMotionRequest::upperCamera});
  };

  initial_state(initial)
  {
    transition
    {
      if(state_time)
        goto panHead;
    }
    action
    {
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
      if(getPanOffset() > maximum.pan || currentHeadTarget.pan > theHeadLimits.maxPan())
      {
        panHeadRight = true;
        setPanOffset(0_deg);
      }
      currentHeadTarget.pan += panHeadRight ? -panStep : panStep;

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
      if(getTiltOffset() < -maximum.tilt || currentHeadTarget.tilt < theHeadLimits.getTiltBound(currentHeadTarget.pan).min)
      {
        tiltHeadDown = true;
        setTiltOffset(0_deg);
      }
      currentHeadTarget.tilt += tiltHeadDown ? tiltStep : -tiltStep;

      moveHeadTo(currentHeadTarget);
    }
  }

  state(waitAfterPan)
  {
    transition
    {
      if(state_time > waitInPosition)
      {
        if(getPanOffset() < -maximum.pan || currentHeadTarget.pan < theHeadLimits.minPan())
        {
          panHeadRight = false;
          setPanOffset(0_deg);
          goto done;
        }
        goto panHead; //for now, turn head only to the left and right. TODO: the states tiltHead and waitAfterTilt are now unused
      }
    }
    action
    {
      moveHeadTo(currentHeadTarget);
    }
  }

  state(waitAfterTilt)
  {
    transition
    {
      if(state_time > waitInPosition)
      {
        if(getTiltOffset() > maximum.tilt || currentHeadTarget.tilt > theHeadLimits.getTiltBound(currentHeadTarget.pan).max)
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
    action
    {
      moveHeadTo(currentHeadTarget);
    }
  }
}
