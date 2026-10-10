/**
 * @file JerseyClassifier.h
 *
 * This file declares a representation which contains functionality to detect
 * jerseys that players wear.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Streaming/Function.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Perception/ObstaclePercepts/RobotPercept.h"

STREAMABLE(JerseyClassifier,
{
  /**
   * Detect the jersey color.
   * @param robotOnField The detected robot, the fields type and confidence will be updated if a jersey color was detected.
   */
  FUNCTION(void(RobotPercept::Robot& robotOnField)) detectJersey,
});
