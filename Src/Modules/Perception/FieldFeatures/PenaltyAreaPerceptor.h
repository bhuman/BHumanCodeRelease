/**
 * @file PenaltyAreaPerceptor.h
 * Provides PenaltyArea.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Tools/Math/BHMath.h"

MODULE(PenaltyAreaPerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(IntersectionRelations),
  PROVIDES(PenaltyArea),
  LOADS_PARAMETERS(
  {,
    (float) thresholdIntersections,
    (Angle) thresholdAngleDisForIntersections,
    (float) thresholdYVarianceIntersection,
  }),
});

class PenaltyAreaPerceptor : public PenaltyAreaPerceptorBase
{
  void update(PenaltyArea& penaltyArea) override;

private:
  bool searchWithIntersections(PenaltyArea& penaltyArea) const;
};
