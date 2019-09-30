/**
 * @file UpperProvider.h
 *
 * This file declares a module that provides representations from the upper
 * camera thread for the current frame.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesFieldPercept.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesPerceptorData.h"
#include "Tools/Module/Module.h"

STREAMABLE_WITH_BASE(UpperFieldBoundary, FieldBoundary, {,});
STREAMABLE_WITH_BASE(UpperObstaclesPerceptorData, ObstaclesPerceptorData, {,});

MODULE(UpperProvider,
{,
  REQUIRES(UpperFieldBoundary),
  PROVIDES(OtherFieldBoundary),
  REQUIRES(UpperObstaclesPerceptorData),
  PROVIDES(OtherObstaclesPerceptorData),
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
};
