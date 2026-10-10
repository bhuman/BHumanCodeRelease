/**
 * @file FieldDimensions.cpp
 *
 * Some useful functions regarding field dimensions.
 *
 * @author Max Risler
 */

#include "FieldDimensions.h"
#include "Debugging/Modify.h"
#include "Streaming/InStreams.h"
#include "Platform/BHAssert.h"
#include "Math/Geometry.h"
#include "Math/Eigen.h"
#include "Representations/Infrastructure/GameState.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Modify.h"
#include "Framework/Blackboard.h"
#include <algorithm>

/**
 * Helper struct that supports the use of symbolic values in float fields.
 */
struct InSymbolicMapFile : public InMapFile
{
private:
  std::unordered_map<std::string, float> values; /**< All symbolic values known. */
  const char* entry = nullptr; /**< The name of the current entry processed. */

public:
  InSymbolicMapFile(const std::string& name, const std::unordered_map<std::string, float>& values = {})
  : InMapFile(name), values(values) {}

  void select(const char* name, int type, const char* enumType) override
  {
    Streaming::trimName(name);
    InMapFile::select(name, type, enumType);
    entry = name;
  }

protected:
  /**
   * When reading a float, read a string instead. Try to replace symbolic value.
   * Symbolic value can be preceded by a minus sign (without whitespace in between).
   */
  void inFloat(float& value) override
  {
    std::string buf;
    inString(buf);
    float sign = 1.f;
    if(buf[0] == '-')
    {
      sign = -1.f;
      buf = buf.substr(1);
    }

    const auto i = values.find(buf);
    if(i != values.end())
      value = i->second * sign;
    else if(!buf.empty() && (std::isdigit(buf[0]) || buf[0] == '.'))
      value = static_cast<float>(std::strtod(buf.c_str(), nullptr)) * sign;
    else
      OUTPUT_ERROR("fieldDimensions.cfg: Unknown symbol '" << buf << "'");

    if(entry)
      values[entry] = value;
  }
};

void FieldDimensions::load()
{
  InSymbolicMapFile stream("fieldDimensions.cfg");
  ASSERT(stream.exists());
  stream >> *this;
  boundary.add(Vector2f(xPosOpponentFieldBorder, yPosLeftFieldBorder));
  boundary.add(Vector2f(xPosOwnFieldBorder, yPosRightFieldBorder));
}

Pose2f FieldDimensions::randomPoseOnField() const
{
  Pose2f pose;
  do
    pose = Pose2f::random(Rangef(-pi, pi), boundary.x, boundary.y);
  while(!isInsideField(pose.translation));
  return pose;
}

Pose2f FieldDimensions::randomPoseOnCarpet() const
{
  Pose2f pose;
  do
    pose = Pose2f::random(Rangef(-pi, pi), boundary.x, boundary.y);
  while(!isInsideCarpet(pose.translation));
  return pose;
}

void FieldDimensions::draw() const
{
  drawPolygons();
  drawLines();
  drawGoalFrame();
  drawCorners();
  bool showValues = true;
  DEBUG_RESPONSE("representation:FieldDimensions:hideValues") showValues = false;
  DEBUG_DRAWING("field dimensions", "drawingOnField") drawDimensions(showValues);
}

void FieldDimensions::drawGoalFrame() const
{
  DEBUG_DRAWING("goal frame", "drawingOnField")
  {
    const ColorRGBA lineColor(224, 224, 224);
    for(const LinesTable::Line& l : goalFrameLines.lines)
    {
      LINE("goal frame", l.from.x(), l.from.y(), l.to.x(), l.to.y(), fieldLinesWidth * 0.7, Drawings::solidPen, lineColor);
    }

    CIRCLE("goal frame", xPosOpponentGoalPost, yPosLeftGoal, goalPostRadius, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("goal frame", xPosOpponentGoalPost, yPosRightGoal, goalPostRadius, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);

    CIRCLE("goal frame", xPosOwnGoalPost, yPosLeftGoal, goalPostRadius, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("goal frame", xPosOwnGoalPost, yPosRightGoal, goalPostRadius, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
  }
}

void FieldDimensions::drawLines() const
{
  DEBUG_DRAWING("field lines", "drawingOnField")
  {
    const ColorRGBA lineColor(224, 224, 224);
    for(const LinesTable::Line& line : fieldLines.lines)
    {
      if(line.partOf != LinesTable::Line::circle)
        LINE("field lines", line.from.x(), line.from.y(), line.to.x(), line.to.y(), fieldLinesWidth, Drawings::solidPen, lineColor);
    }

    CIRCLE("field lines", centerCircle.center.x(), centerCircle.center.y(), centerCircle.radius, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);

    if(!marksAreCrosses)
      for(const Vector2f& mark : marks)
        CIRCLE("field lines", mark.x(), mark.y(), markRadius, 0.f, Drawings::noPen, lineColor, Drawings::solidBrush, lineColor);

    if(hasCornerArcs)
    {
      const float angle = std::atan2(fieldLinesWidth, cornerArcRadius);
      ARC("field lines", xPosOwnGoalLine - fieldLinesWidth / 2.f, yPosLeftTouchline + fieldLinesWidth / 2.f, cornerArcRadius, -angle, -pi_2 + 2.f * angle, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);
      ARC("field lines", xPosOwnGoalLine - fieldLinesWidth / 2.f, yPosRightTouchline - fieldLinesWidth / 2.f, cornerArcRadius, angle, pi_2 - 2.f * angle, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);
      ARC("field lines", xPosOpponentGoalLine + fieldLinesWidth / 2.f, yPosLeftTouchline + fieldLinesWidth / 2.f, cornerArcRadius, pi + angle, pi_2 - 2.f * angle, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);
      ARC("field lines", xPosOpponentGoalLine + fieldLinesWidth / 2.f, yPosRightTouchline - fieldLinesWidth / 2.f, cornerArcRadius, pi - angle, -pi_2 + 2.f * angle, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);
    }
  }

  DEBUG_DRAWING("half meter grid", "drawingOnField")
  {
    for(int y = 0; y < yPosLeftFieldBorder; y += 500)
    {
      LINE("half meter grid", xPosOwnFieldBorder, y, xPosOpponentFieldBorder, y, 10, Drawings::dottedPen, ColorRGBA::black);
      LINE("half meter grid", xPosOwnFieldBorder, -y, xPosOpponentFieldBorder, -y, 10, Drawings::dottedPen, ColorRGBA::black);
    }

    for(int x = 0; x < xPosOpponentFieldBorder; x += 500)
    {
      LINE("half meter grid", x, yPosLeftFieldBorder, x, yPosRightFieldBorder, 10, Drawings::dottedPen, ColorRGBA::black);
      LINE("half meter grid", -x, yPosLeftFieldBorder, -x, yPosRightFieldBorder, 10, Drawings::dottedPen, ColorRGBA::black);
    }
  }
}

void FieldDimensions::drawPolygons() const
{
  if(Blackboard::getInstance().exists("GameState"))
  {
    const GameState& gameState = static_cast<const GameState&>(Blackboard::getInstance()["GameState"]);

    DEBUG_DRAWING("field polygons", "drawingOnField")
    {
      Vector2f points[4];
      points[0].x() = points[1].x() = xPosOpponentFieldBorder;
      points[2].x() = points[3].x() = xPosOwnFieldBorder;
      points[0].y() = points[3].y() = yPosLeftFieldBorder;
      points[1].y() = points[2].y() = yPosRightFieldBorder;
      POLYGON("field polygons", 4, points, 0, Drawings::solidPen, ColorRGBA(0, 140, 0), Drawings::solidBrush, ColorRGBA(0, 140, 0));

      static const ColorRGBA colors[] =
      {
        ColorRGBA(50, 120, 127),
        ColorRGBA(127, 80, 80),
        ColorRGBA(127, 120, 50),
        ColorRGBA(40, 40, 40),
        ColorRGBA(180, 180, 180),
        ColorRGBA(40, 80, 40),
        ColorRGBA(127, 120, 0),
        ColorRGBA(120, 20, 127),
        ColorRGBA(127, 80, 20),
        ColorRGBA(120, 120, 120)
      };
      static_assert(sizeof(colors) / sizeof(colors[0]) == GameState::Team::Color::numOfTeamColors);
      const ColorRGBA& own = colors[gameState.ownTeam.fieldPlayerColor];
      const ColorRGBA& opp = colors[gameState.opponentTeam.fieldPlayerColor];

      Vector2f goal[4];
      goal[0] = Vector2f(xPosOwnGoalLine - fieldLinesWidth / 2.f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOwnGoalLine - fieldLinesWidth / 2.f, yPosRightGoal);
      goal[2] = Vector2f(xPosOwnGoal, yPosRightGoal);
      goal[3] = Vector2f(xPosOwnGoal, yPosLeftGoal);
      POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, own, Drawings::solidBrush, own);

      goal[0] = Vector2f(xPosOpponentGoalLine + fieldLinesWidth / 2.f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOpponentGoalLine + fieldLinesWidth / 2.f, yPosRightGoal);
      goal[2] = Vector2f(xPosOpponentGoal, yPosRightGoal);
      goal[3] = Vector2f(xPosOpponentGoal, yPosLeftGoal);
      POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, opp, Drawings::solidBrush, opp);
    }
  }
}

void FieldDimensions::drawCorners() const
{
  DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");
  CornerClass c = xCorner;
  MODIFY("representation:FieldDimensions:cornerClass", c);
  COMPLEX_DRAWING("field corners")
  {
    for(auto i = corners[c].begin(); i != corners[c].end(); ++i)
      LARGE_DOT("field corners", i->x(), i->y(), ColorRGBA(255, 0, 0), ColorRGBA(255, 0, 0));
  }
}

void FieldDimensions::drawDimensions(bool showValues) const
{
  const float thickness = 10.f;
  const float ends = fieldLinesWidth;
  const float fontSize = ends * 5;
  const float xText = centerCircleRadius + fontSize;

  auto drawX = [showValues, thickness, ends, fontSize]([[maybe_unused]] float x, [[maybe_unused]] float y,
                                                       [[maybe_unused]] float length, [[maybe_unused]] float xText,
                                                       [[maybe_unused]] const std::string& name)
  {
    LINE("field dimensions", x, y, x + length, y, thickness, Drawings::solidPen, ColorRGBA::black);
    LINE("field dimensions", x, y - ends, x, y + ends, thickness, Drawings::solidPen, ColorRGBA::black);
    LINE("field dimensions", x + length, y - ends, x + length, y + ends, thickness, Drawings::solidPen, ColorRGBA::black);
    DRAW_TEXT("field dimensions", xText, y - fontSize, fontSize, ColorRGBA::black, name
              + (showValues ? std::string(" = ") + std::to_string(static_cast<int>(length)) + " mm": ""));
  };
  auto drawY = [showValues, thickness, ends, fontSize]([[maybe_unused]] float x, [[maybe_unused]] float y,
                                                       [[maybe_unused]] float length, [[maybe_unused]] float xOffset,
                                                       [[maybe_unused]] const std::string& name)
  {
    LINE("field dimensions", x, y, x, y + length, thickness, Drawings::solidPen, ColorRGBA::black);
    LINE("field dimensions", x - ends, y, x + ends, y, thickness, Drawings::solidPen, ColorRGBA::black);
    LINE("field dimensions", x - ends, y + length, x + ends, y + length, thickness, Drawings::solidPen, ColorRGBA::black);
    DRAW_TEXT("field dimensions", x + ends / 2 + xOffset, y + length / 2 - fontSize / 4.f, fontSize, ColorRGBA::black, name
              + (showValues ? std::string(" = ") + std::to_string(static_cast<int>(length)) + " mm": ""));
  };

  drawX(0, (yPosLeftFieldBorder + yPosLeftTouchline) / 2.f, xPosOpponentFieldBorder, xText, "xPosOpponentFieldBorder");
  drawX(0, (yPosLeftTouchline + yPosLeftPenaltyArea) / 2.f, xPosOpponentGoalLine, xText, "xPosOpponentGoalLine");
  drawX(0, (yPosLeftPenaltyArea + yPosLeftGoalArea) / 2.f, xPosOpponentPenaltyArea, xText, "xPosOpponentPenaltyArea");
  if(xPosOpponentGoalArea != xPosOpponentPenaltyArea)
    drawX(0, (yPosLeftGoalArea + yPosLeftGoal) / 2.f, xPosOpponentGoalArea, xText, "xPosOpponentGoalArea");
  drawX(0, yPosLeftGoal / 2.f, xPosOpponentGoal, xText, "xPosOpponentGoal");
  drawX(0, 0, xPosOpponentPenaltyMark, xText, "xPosOpponentPenaltyMark");
  drawX(0, yPosRightGoal, xPosOpponentGoalPost, xText, "xPosOpponentGoalPost");
  drawY((xPosOwnFieldBorder + xPosOwnGoal) / 2.f, 0, yPosLeftFieldBorder, 0, "yPosLeftFieldBorder");
  drawY((xPosOwnGoal + xPosOwnGoalLine) / 2.f, 0, yPosLeftGoal, 0, "yPosLeftGoal");
  if(yPosLeftGoalArea != yPosLeftPenaltyArea)
    drawY((xPosOwnGoalLine + xPosOwnGoalArea) / 2.f, 0, yPosLeftGoalArea, 0, "yPosLeftGoalArea");
  drawY((xPosOwnGoalArea + xPosOwnPenaltyArea) / 2.f, 0, yPosLeftPenaltyArea, 0, "yPosLeftPenaltyArea");
  drawY((xPosOwnPenaltyArea - centerCircleRadius) / 2.f, 0, yPosLeftTouchline, 0, "yPosLeftTouchline");
  drawY(-centerCircleRadius, -centerCircleRadius, centerCircleRadius, 0, "centerCircleRadius");
  drawY(xPosOwnGoalPost + goalPostRadius + ends, yPosRightGoal - goalPostRadius, goalPostRadius, ends, "goalPostRadius");
  drawY(xPosOwnPenaltyMark + fieldLinesWidth / 2.f + ends, -markRadius, markRadius, ends, "markRadius");
  drawY(xPosOwnPenaltyArea + fieldLinesWidth / 2.f + ends, yPosRightPenaltyArea - fieldLinesWidth / 2.f, fieldLinesWidth, ends, "fieldLinesWidth");
  drawY(xPosOwnGoalLine + cornerArcRadius - fieldLinesWidth / 2.f, yPosLeftTouchline - cornerArcRadius + fieldLinesWidth / 2.f, cornerArcRadius, 0, "cornerArcRadius");
}

void FieldDimensions::draw3D() const
{
  DEBUG_DRAWING3D("representation:FieldDimensions", "robot")
  {
    RENDER_OPTIONS3D("representation:FieldDimensions", Drawings3D::disableTransparency);

    const ColorRGBA fieldColor(0, 70, 0);
    const ColorRGBA lineColor(224, 224, 224);

    // Carpet
    {
      const Vector3f points[4] =
      {
        {xPosOpponentFieldBorder, yPosRightFieldBorder, -1.f},
        {xPosOpponentFieldBorder, yPosLeftFieldBorder, -1.f},
        {xPosOwnFieldBorder, yPosLeftFieldBorder, -1.f},
        {xPosOwnFieldBorder, yPosRightFieldBorder, -1.f}
      };
      QUAD3D("representation:FieldDimensions", points[0], points[1], points[2], points[3], fieldColor);
    }

    // Lines
    for(const LinesTable::Line& line : fieldLines.lines)
    {
      const float lengthOffset = line.partOf == LinesTable::Line::mark ? 0.f : (line.partOf == LinesTable::Line::circle ? 0.1f : 0.5f) * fieldLinesWidth;
      const float length = (line.to - line.from).norm();
      const Pose2f turn((line.to - line.from).angle(), line.from);
      const Vector2f relative[4] =
      {
        {-lengthOffset, fieldLinesWidth / 2.f},
        {-lengthOffset, -fieldLinesWidth / 2.f},
        {length + lengthOffset, -fieldLinesWidth / 2.f},
        {length + lengthOffset, fieldLinesWidth / 2.f}
      };
      const Vector2f points[4] =
      {
        turn * relative[0],
        turn * relative[1],
        turn * relative[2],
        turn * relative[3]
      };
      QUAD3D("representation:FieldDimensions",
             Vector3f(points[0].x(), points[0].y(), 0.f),
             Vector3f(points[1].x(), points[1].y(), 0.f),
             Vector3f(points[2].x(), points[2].y(), 0.f),
             Vector3f(points[3].x(), points[3].y(), 0.f),
             lineColor);
    }

    if(!marksAreCrosses)
      for(const Vector2f& mark : marks)
        CYLINDER3D("representation:FieldDimensions", mark.x(), mark.y(), 0.f, 0.f, 0.f, 0.f, markRadius, 1.f, lineColor);

    // Goals
    {
      const Vector3f points[6] =
      {
        {xPosOwnGoalPost, yPosLeftGoal, goalHeight / 2.f},
        {xPosOwnGoalPost, yPosRightGoal, goalHeight / 2.f},
        {xPosOpponentGoalPost, yPosLeftGoal, goalHeight / 2.f},
        {xPosOpponentGoalPost, yPosRightGoal, goalHeight / 2.f},
        {xPosOwnGoalPost, 0.f, goalHeight - goalPostRadius},
        {xPosOpponentGoalPost, 0.f, goalHeight - goalPostRadius}
      };
      for(int i = 0; i < 4; ++i)
        CYLINDER3D("representation:FieldDimensions", points[i].x(), points[i].y(), points[i].z(), 0, 0, 0, goalPostRadius, goalHeight, lineColor);
      for(int i = 4; i < 6; ++i)
        CYLINDER3D("representation:FieldDimensions", points[i].x(), points[i].y(), points[i].z(), 90_deg, 0, 0, goalPostRadius,
                   yPosLeftGoal - yPosRightGoal, lineColor);
    }
  }
}

void FieldDimensions::LinesTable::push(const Vector2f& from, const Vector2f& to, Line::PartOf partOf)
{
  LinesTable::Line line;
  line.from = from;
  line.to = to;
  line.partOf = partOf;
  lines.push_back(line);
}

void FieldDimensions::LinesTable::pushCircle(const Vector2f& center, float radius, int numOfSegments)
{
  Vector2f p1, p2;
  for(float a = 0; a <= pi_4; a += pi2 / numOfSegments)
  {
    p1 = Vector2f(std::sin(a), std::cos(a)) * radius;
    if(a > 0)
    {
      push(center + p1, center + p2, Line::circle);
      push(center + Vector2f(p1.x(), -p1.y()), center + Vector2f(p2.x(), -p2.y()), Line::circle);
      push(center + Vector2f(-p1.x(), p1.y()), center + Vector2f(-p2.x(), p2.y()), Line::circle);
      push(center - p1, center - p2, Line::circle);
      push(center + Vector2f(p1.y(), p1.x()), center + Vector2f(p2.y(), p2.x()), Line::circle);
      push(center + Vector2f(p1.y(), -p1.x()), center + Vector2f(p2.y(), -p2.x()), Line::circle);
      push(center + Vector2f(-p1.y(), p1.x()), center + Vector2f(-p2.y(), p2.x()), Line::circle);
      push(center + Vector2f(-p1.y(), -p1.x()), center + Vector2f(-p2.y(), -p2.x()), Line::circle);
    }
    p2 = p1;
  }
}

void FieldDimensions::LinesTable::pushCross(const Vector2f& center, float radius)
{
  push(center - Vector2f(0.f, radius), center + Vector2f(0.f, radius), Line::mark);
  push(center - Vector2f(radius, 0.f), center + Vector2f(radius, 0.f), Line::mark);
}

void FieldDimensions::read(In& stream)
{
  STREAM_BASE(SimpleFieldDimensions);

  std::vector<LinesTable::Line>& fieldLines(straightFieldLines.lines);
  std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);

  STREAM(goalFrameLines);
  STREAM(fieldLines);
  STREAM(centerCircle);
  STREAM(marksAreCrosses);
  STREAM(marks);
  STREAM(corners);

  // merge straightFieldLines, centerCircle and cross-marks to fieldLines
  this->fieldLines = straightFieldLines;
  this->fieldLines.pushCircle(centerCircle.center, centerCircle.radius, centerCircle.numOfSegments);
  if(marksAreCrosses)
    for(const Vector2f& mark : marks)
      this->fieldLines.pushCross(mark, markRadius - fieldLinesWidth / 2.f);

  // merge fieldLines and goalFrameLines into fieldLinesWithGoalFrame
  fieldLinesWithGoalFrame.lines.clear();
  for(LinesTable::Line& line : this->fieldLines.lines)
    fieldLinesWithGoalFrame.lines.push_back(line);
  for(LinesTable::Line& line : goalFrameLines)
    fieldLinesWithGoalFrame.lines.push_back(line);
}

void FieldDimensions::write(Out& stream) const
{
  STREAM_BASE(SimpleFieldDimensions);

  const std::vector<LinesTable::Line>& fieldLines(straightFieldLines.lines);
  const std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);

  STREAM(goalFrameLines);
  STREAM(fieldLines);
  STREAM(centerCircle);
  STREAM(marksAreCrosses);
  STREAM(marks);
  STREAM(corners);
}

void FieldDimensions::reg()
{
  PUBLISH(reg);
  REG_CLASS_WITH_BASE(FieldDimensions, SimpleFieldDimensions);
  REG(std::vector<LinesTable::Line>, goalFrameLines);
  REG(std::vector<LinesTable::Line>, fieldLines);
  REG(centerCircle);
  REG(marksAreCrosses);
  REG(marks);
  REG(corners);
}
