/**
 * @file LowerProvider.h
 *
 * This file declares a module that provides representations from the lower
 * camera thread for the current frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Tools/Module/Module.h"

STREAMABLE_WITH_BASE(LowerFieldBoundary, FieldBoundary, {,});
STREAMABLE_WITH_BASE(LowerObstaclesPerceptorData, ObstaclesPerceptorData, {,});

MODULE(LowerProvider,
{,
  REQUIRES(LowerFieldBoundary),
  PROVIDES(OtherFieldBoundary),
  REQUIRES(LowerObstaclesPerceptorData),
  PROVIDES(OtherObstaclesPerceptorData),
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
};
