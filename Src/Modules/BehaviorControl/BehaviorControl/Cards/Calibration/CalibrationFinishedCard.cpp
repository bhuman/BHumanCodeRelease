/**
 * @file CalibrationFinishedCard.cpp
 *
 * This file defines a card that defines the behavior of the robot when all calibration steps are done.
 *
 * @author Lukas Plecher
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(CalibrationFinishedCard,
{,
  CALLS(Activity),
  CALLS(KeyframeMotion),
  CALLS(LookForward),
  CALLS(Say),
});

class CalibrationFinishedCard : public CalibrationFinishedCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theSaySkill("Calibration finished.");
    theKeyframeMotionSkill(KeyframeMotionRequest::sitDown);
    theActivitySkill(BehaviorStatus::calibrationFinished);
    theLookForwardSkill();
  }

  void reset() override
  {
    if(SystemCall::getMode() == SystemCall::simulatedRobot)
      OUTPUT(idConsole, text, "gc autonomousCalibrationFinished");
  }
};

MAKE_CARD(CalibrationFinishedCard);
