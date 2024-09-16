/**
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Math/Eigen.h"
#include "Framework/Module.h"

MODULE(FieldLinesProvider,
{,
  REQUIRES(RobotPose),

  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),

  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),

  REQUIRES(OdometryData),

  REQUIRES(CirclePercept),
  REQUIRES(IntersectionsPercept),
  REQUIRES(LinesPercept),
  REQUIRES(MeasurementCovariance),

  PROVIDES(FieldLines),
  REQUIRES(FieldLines),

  PROVIDES(FieldLineIntersections),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) bigLineThreshold,                        /**< Internal definition for a long line. TODO: Rethink this! */
    (int)(30) maxTimeOffset,
    (float)(200.f) maxLineDeviationFromAssumedCenterCircle,  /**< If the distance of a short line to the center circle is larger than this, it is not considered to be on the circle. */
    (float)(0.4f) centerWeighting,                           /**< Used for computing the covariance of a line by determining the actual point for this computation, must be between 0 (-> closest point on line) and 1 (-> center of line).*/
  }),
});

class FieldLinesProvider : public FieldLinesProviderBase
{
private:
  using SpotLine = LinesPercept::Line;
  using PerceptLine = FieldLines::Line;

  ENUM(SpotLineStatus,
  {,
    thrown,
    stayed,
  });
  std::vector<SpotLineStatus> spotLineUsage;
  CirclePercept lastCirclePercept;
  OdometryData lastOdometryData;
  unsigned int lastFrameTime = 1;
  std::vector<FieldLines::Line> internalListOfLines;  /**< Unsorted list of computed field lines. */

public:
  const SpotLine* midLine;
  bool isPointInSegment(const SpotLine& line, const Vector2f& point) const;
  void update(FieldLines& fieldLines) override;
  void update(FieldLineIntersections& fieldLineIntersections) override;
};
