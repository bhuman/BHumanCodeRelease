/**
 * @file GoalFramePerceptor.h
 * profides GoalFrame
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldFeatures/GoalFrame.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Angle.h"

MODULE(GoalFramePerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(LinesPercept),
  REQUIRES(IntersectionsPercept),
  REQUIRES(FieldBoundary),

  PROVIDES(GoalFrame),
  DEFINES_PARAMETERS(
  {,
    (float)(sqr(70.f)) sqrAllowedBXDivergence,
    (float)(100.f) allowedFieldBoundaryDivergence,
    (int)(2) neededConvexBoundaryPoints,
    (float)(sqr(850.f)) squaredBigIntersectionThreshold, /**< the square of the threshold for each linesegment of a big intersection */
    (float)(sqr(200.f)) tTDistanceThreshold,
    (float)(100.f) allowedGoalPostToLineDistance,
    (Angle)(20_deg) allowedTTAngleDivergence,
  }),
});

class GoalFramePerceptor : public GoalFramePerceptorBase
{
  void update(GoalFrame& goalFrame);
private:
  bool searchByBigT(GoalFrame& goalFrame) const;
  bool searchByBigX(GoalFrame& goalFrame) const;
  bool searchByTT(GoalFrame& goalFrame) const;

  bool calcGoalFrame(const Pose2f& prePose, const float yTranslation, GoalFrame& goalFrame) const;
};
