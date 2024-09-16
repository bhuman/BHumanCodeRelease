/**
 * @file LowerProvider.h
 *
 * This file declares a module that provides representations from the lower
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

STREAMABLE_WITH_BASE(LowerFieldBoundary, FieldBoundary, {,});
STREAMABLE_WITH_BASE(LowerObstaclesPerceptorData, ObstaclesPerceptorData, {,});
STREAMABLE_WITH_BASE(LowerOdometryData, OdometryData, {,});

MODULE(LowerProvider,
{,
  REQUIRES(LowerFieldBoundary),
  PROVIDES(OtherFieldBoundary),
  REQUIRES(LowerObstaclesPerceptorData),
  PROVIDES(OtherObstaclesPerceptorData),
  REQUIRES(LowerOdometryData),
  PROVIDES(OtherOdometryData),
});

class LowerProvider : public LowerProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherFieldBoundary The representation updated.
   */
  void update(OtherFieldBoundary& theOtherFieldBoundary) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherObstaclesPerceptorData Since no data is sent from the lower
   *                                      to the upper image, nothing happens
   *                                      here.
   */
  void update(OtherObstaclesPerceptorData& theOtherObstaclesPerceptorData) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theOtherOdometryData The representation updated.
   */
  void update(OtherOdometryData& theOtherOdometryData) override;
};
