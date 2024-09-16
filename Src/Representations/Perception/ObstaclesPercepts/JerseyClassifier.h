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
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"

STREAMABLE(JerseyClassifier,
{
  /**
   * Detect the jersey color.
   * @param obstacleInImage The obstacle as it was detected in the image.
   * @param obstacleOnField The fields type and confidence will be updated if a jersey color was detected.
   */
  FUNCTION(void(const ObstaclesImagePercept::Obstacle& obstacleInImage, ObstaclesFieldPercept::Obstacle& obstacleOnField)) detectJersey,
});
