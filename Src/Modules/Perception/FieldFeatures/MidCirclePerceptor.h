/**
 * @file MidCirclePerceptor.h
 * profides MidCircle
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Tools/Math/BHMath.h"


MODULE(MidCirclePerceptor,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FieldLineIntersections),
  REQUIRES(IntersectionRelations),
  REQUIRES(FieldLines),
  REQUIRES(CirclePercept),
  REQUIRES(Odometer),
  REQUIRES(FieldDimensions),
  PROVIDES(MidCircle),
  DEFINES_PARAMETERS(
  {,
    (int)(30) maxTimeOffset,
    (float)(150) maxLineDistanceToCircleCenter,
    (float)(1000) allowedOffsetOfMidLineEndToCircleCenter,
    (float)(300) allowedTsXVariance,
    (float)(sqr(200)) squaredMinLineLength,
  }),
});

class MidCirclePerceptor : public MidCirclePerceptorBase
{
  void update(MidCircle& midCircle);
private:
  int lastFrameTime = 1;
  bool searchCircleWithLine(MidCircle& midCircle) const;
  bool searchWithSXAndT(MidCircle& midCircle) const;
};
