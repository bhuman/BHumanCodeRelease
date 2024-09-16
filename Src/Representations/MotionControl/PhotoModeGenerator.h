/**
 * @file CalibrationGenerator.h
 *
 * This file declares a representation that can create phases to calibrate the robot.
 * The robot is meant to start from a standing position.
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Infrastructure/LEDRequest.h"

STREAMABLE_WITH_BASE(PhotoModeGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION;

  // Create enumeration of relevant body parts of the robot,
  // that will subsequently be made unstiff to position them by hand
  ENUM(BodyPart,
  {,
    head,
    leftArm,
    rightArm,
    leftLeg,
    rightLeg,
    bothLegs,
  });

  ENUM(LEDGroup,
  {,
    eyes,
    ears,
    chest,
    feet,
    all,
  });

  ENUM(LampColor,
  {,
    white,
    red,
    green,
    blue,
    yellow,
    magenta,
    cyan,
    off,
  }),

  (bool)(false) isActive,
  (PhotoModeGenerator::BodyPart)(PhotoModeGenerator::head) bodyPart, // the selected body part
  (bool)(false) isUnstiff, // Is the selected body part unstiff?
  (bool)(false) selectLEDs, // Are we currently changing LEDs?
  (LEDGroup)(PhotoModeGenerator::eyes) ledGroup, // currently changed LED Group
  (ENUM_INDEXED_ARRAY(LampColor, LEDGroup)) lamps, // Which lamps are turned on (each color separately)
//  (ENUM_INDEXED_ARRAY(LEDRequest::LEDState, LEDRequest::LED)) lamps, // Which lamps are turned on (each color separately)
});
