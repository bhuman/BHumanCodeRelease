/**
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"

MODULE(FieldLinesProvider,
{,
  REQUIRES(RobotPose),

  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),

  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),

  REQUIRES(Odometer),

  REQUIRES(CirclePercept),
  REQUIRES(IntersectionsPercept),
  REQUIRES(LinesPercept),

  PROVIDES(FieldLines),
  REQUIRES(FieldLines),

  PROVIDES(FieldLineIntersections),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) bigLineThreshold, /**< Internal definition for a long line. TODO: Rethink this! */
    (int)(30) maxTimeOffset,
    (float)(200.f) maxLineDeviationFromAssumedCenterCircle, /**< If the distance of a short line to the center circle is larger than this, it is not considered to be on the circle. */
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
  static const int lostIndex = 5555;
  std::vector<unsigned> lineIndexTable;
  bool lastCircleWasSeen = false;
  unsigned int lastFrameTime = 1;
  std::vector<FieldLines::Line> internalListOfLines;  /**< Unsorted list of computed field lines. */

public:
  SpotLine* midLine;
  bool isPointInSegment(const SpotLine& line, const Vector2f& point) const;
  void update(FieldLines& fieldLines) override;
  void update(FieldLineIntersections& fieldLineIntersections) override;
};
