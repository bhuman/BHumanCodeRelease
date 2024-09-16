/**
 * @file CalibrateRobot.cpp
 *
 * @author Lukas Plecher
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) CalibrateRobot,
       args((const CalibrationRequest&) request))
{
  initial_state(execute)
  {
    action
    {
      theCalibrationRequest = request;
    }
  }
}
