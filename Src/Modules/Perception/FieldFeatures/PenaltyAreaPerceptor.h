/**
 * @file PenaltyAreaPerceptor.h
 * Provides PenaltyArea.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Tools/Math/BHMath.h"

MODULE(PenaltyAreaPerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(IntersectionRelations),
  REQUIRES(Odometer),
  REQUIRES(PenaltyMarkPercept),
  PROVIDES(PenaltyArea),
  LOADS_PARAMETERS(
  {,
    (bool)(true) usePenaltyMark,
    (float)(110.f) thresholdDisVaranzToPenaltyMark,
    (float)(750.f) maxDistVarianceOfLineEnds,
    (float)(70.f) thresholdIntersections,
    (Angle)(20_deg) thesholdAngleDisForIntersections,
    (int)(30) maxTimeOffset,
    (float)(100.f) thresholdYVarianceIntersection,
  }),
});

class PenaltyAreaPerceptor : public PenaltyAreaPerceptorBase
{
  void update(PenaltyArea& penaltyArea) override;

private:
  unsigned lastFrameTime = 1;
  PenaltyMarkPercept theLastPenaltyMarkPercept;

  bool searchWithPMarkAndLine(PenaltyArea& penaltyArea) const;
  bool searchWithIntersections(PenaltyArea& penaltyArea) const;

  bool checkLineDistanceToNearesPoint(const Pose2f& posePA, const Vector2f& p1, const Vector2f& p2) const;
};
