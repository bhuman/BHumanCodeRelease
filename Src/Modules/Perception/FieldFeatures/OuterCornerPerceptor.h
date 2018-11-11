/**
 * @file OuterCornerPerceptor.h
 * profides OuterCorner
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Representations/Perception/FieldFeatures/OuterCorner.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Tools/Math/BHMath.h"

MODULE(OuterCornerPerceptor,
{,
  REQUIRES(FieldLineIntersections),
  REQUIRES(IntersectionRelations),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(PenaltyArea),
  PROVIDES(OuterCorner),
  DEFINES_PARAMETERS(
  {,
    (float)(400.f) thresholdLTIntersections,
    (float)(600.f) thresholdLLIntersections,
    (float)(sqr(300.f)) squaredThresholdDisOffsetToPA,
    (float)(100.f) allowedDisplacement,
    (Angle)(20_deg) allowedAngleDisplacement,
    (Angle)(20_deg) thresholdPostulatedCornerAngleOffset,
    (Angle)(20_deg) thesholdAngleDisForLTIntersections,

    (float)(700.f) distrustAreaXRadius,
    (float)(800.f) distrustAreaYRadius,
    (Vector2f)(Vector2f(200.f, 0.f)) distrustAreaOffset,
  }),
});

class OuterCornerPerceptor : public OuterCornerPerceptorBase
{
  void update(OuterCorner& outerCorner) override;
private:
  bool searchForLAndPA(OuterCorner& outerCorner) const;
  bool searchForBigLAndT(OuterCorner& outerCorner) const;
  bool searchForBigLAndTL(OuterCorner& outerCorner) const;
  bool searchForBigLAndSmallL(OuterCorner& outerCorner) const;
};
