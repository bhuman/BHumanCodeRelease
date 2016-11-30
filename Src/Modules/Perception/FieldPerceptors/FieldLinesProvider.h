/**
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/PlayersPercepts/PlayersPercept.h"
#include "Representations/Perception/FieldFeatures/GoalFrame.h"
#include <vector>
#include <deque>
#include <functional>

MODULE(FieldLinesProvider,
{,
  REQUIRES(Image),
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(CirclePercept),
  REQUIRES(FieldDimensions),
  REQUIRES(PlayersPercept),
  REQUIRES(LinesPercept),
  REQUIRES(IntersectionsPercept),
  REQUIRES(GoalFrame),

  PROVIDES(FieldLines),
  REQUIRES(FieldLines),

  PROVIDES(FieldLineIntersections),
  DEFINES_PARAMETERS(
  {,
    (float)(sqr(850.f)) squaredBigLineThreshold, /**< the square of the threshold for each linesegment of a long line; This should be a good way longer then a penaltySideLine */
    (float)(20.f) goalFrameThreshold, /**< the threshold for a perception to be in field according to the goalFrame */
    (float)(sqr(20.f)) squaredMinLenghOfACuttedLine, /**< minmal length of a cutted line */
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
    cutted,
    stayed,
  });
  std::vector<SpotLineStatus> spotLineUsage;
  static const int lostIndex = 5555;
  std::vector<unsigned> lineIndexTable;

public:
  SpotLine* midLine;
  bool isPointInSegment(const SpotLine& line, const Vector2f& point) const;
  void update(FieldLines& fieldLines);
  void update(FieldLineIntersections& fieldLineIntersections);
};
