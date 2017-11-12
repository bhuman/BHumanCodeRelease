/**
 * @file GoalFeaturePerceptor.h
 * Provides GoalFeature.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/FieldPercepts/GoalPostPercept.h"
#include "Representations/Perception/FieldFeatures/GoalFeature.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Angle.h"

MODULE(GoalFeaturePerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(FrameInfo),
  REQUIRES(GoalPostPercept),
  REQUIRES(IntersectionRelations),
  REQUIRES(Odometer),

  PROVIDES(GoalFeature),
  DEFINES_PARAMETERS(
  {,
    (float)(70.f) thresholdMaxDisVaranzGoalPostT,
    (float)(50.f) thresholdMaxXVaranzGoalPostT,
  }),
});

class GoalFeaturePerceptor : public GoalFeaturePerceptorBase
{
  void update(GoalFeature& goalFeature);
private:
  GoalPostPercept theLastGoalPostPercept;
  bool searchByGoalPostAndT(GoalFeature& goalFeature) const;
};
