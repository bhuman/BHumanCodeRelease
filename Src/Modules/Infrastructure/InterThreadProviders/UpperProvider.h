/**
 * @file UpperProvider.h
 *
 * This file declares a module that provides representations from the upper
 * camera thread for the current frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/GoalPercepts/GoalPostsPercept.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Framework/Module.h"

STREAMABLE_WITH_BASE(UpperFieldBoundary, FieldBoundary, {,});
STREAMABLE_WITH_BASE(UpperObstaclesPerceptorData, ObstaclesPerceptorData, {,});
STREAMABLE_WITH_BASE(UpperOdometryData, OdometryData, {,});

MODULE(UpperProvider,
{,
  REQUIRES(UpperFieldBoundary),
  PROVIDES(OtherFieldBoundary),
  REQUIRES(UpperObstaclesPerceptorData),
  PROVIDES(OtherObstaclesPerceptorData),
  REQUIRES(UpperOdometryData),
  PROVIDES(OtherOdometryData),
});

class UpperProvider : public UpperProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherFieldBoundary The representation updated.
   */
  void update(OtherFieldBoundary& theOtherFieldBoundary) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherObstaclesPerceptorData The representation updated.
   */
  void update(OtherObstaclesPerceptorData& theOtherObstaclesPerceptorData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherOdometryData The representation updated.
   */
  void update(OtherOdometryData& theOtherOdometryData) override;
};
