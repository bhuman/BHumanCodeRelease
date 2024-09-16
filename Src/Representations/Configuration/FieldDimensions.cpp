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

    CIRCLE("goal frame", xPosOpponentGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("goal frame", xPosOpponentGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);

    CIRCLE("goal frame", xPosOwnGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen,
           ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    CIRCLE("goal frame", xPosOwnGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen,
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
      if(!line.isPartOfCircle)
        LINE("field lines", line.from.x(), line.from.y(), line.to.x(), line.to.y(), fieldLinesWidth, Drawings::solidPen, lineColor);
    }

    CIRCLE("field lines", centerCircle.center.x(), centerCircle.center.y(), centerCircle.radius, fieldLinesWidth, Drawings::solidPen, lineColor, Drawings::noBrush, ColorRGBA::black);
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
      goal[0] = Vector2f(xPosOwnGoalLine - fieldLinesWidth * 0.5f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOwnGoalLine - fieldLinesWidth * 0.5f, yPosRightGoal);
      goal[2] = Vector2f(xPosOwnGoal, yPosRightGoal);
      goal[3] = Vector2f(xPosOwnGoal, yPosLeftGoal);
      POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, own, Drawings::solidBrush, own);

      goal[0] = Vector2f(xPosOpponentGoalLine + fieldLinesWidth * 0.5f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOpponentGoalLine + fieldLinesWidth * 0.5f, yPosRightGoal);
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
  const float fontSize = ends * 3;
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
  drawY(xPosOwnPenaltyMark + fieldLinesWidth / 2.f + ends, -penaltyMarkSize / 2.f, penaltyMarkSize, ends, "penaltyMarkSize");
  drawY(xPosOwnPenaltyArea + fieldLinesWidth / 2.f + ends, yPosRightPenaltyArea - fieldLinesWidth / 2.f, fieldLinesWidth, ends, "fieldLinesWidth");
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
      const float length = (line.to - line.from).norm();
      const Pose2f turn((line.to - line.from).angle(), line.from);
      const Vector2f relative[4] =
      {
        {(line.isPartOfCircle ? 0.1f : 0.5f) * -fieldLinesWidth, fieldLinesWidth / 2.f},
        {(line.isPartOfCircle ? 0.1f : 0.5f) * -fieldLinesWidth, -fieldLinesWidth / 2.f},
        {length + (line.isPartOfCircle ? 0.1f : 0.5f) * fieldLinesWidth, -fieldLinesWidth / 2.f},
        {length + (line.isPartOfCircle ? 0.1f : 0.5f) * fieldLinesWidth, fieldLinesWidth / 2.f}
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

void FieldDimensions::LinesTable::push(const Vector2f& from, const Vector2f& to, bool isPartOfCircle)
{
  LinesTable::Line line;
  line.from = from;
  line.to = to;
  line.isPartOfCircle = isPartOfCircle;
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
      push(center + p1, center + p2, true);
      push(center + Vector2f(p1.x(), -p1.y()), center + Vector2f(p2.x(), -p2.y()), true);
      push(center + Vector2f(-p1.x(), p1.y()), center + Vector2f(-p2.x(), p2.y()), true);
      push(center - p1, center - p2, true);
      push(center + Vector2f(p1.y(), p1.x()), center + Vector2f(p2.y(), p2.x()), true);
      push(center + Vector2f(p1.y(), -p1.x()), center + Vector2f(p2.y(), -p2.x()), true);
      push(center + Vector2f(-p1.y(), p1.x()), center + Vector2f(-p2.y(), p2.x()), true);
      push(center + Vector2f(-p1.y(), -p1.x()), center + Vector2f(-p2.y(), -p2.x()), true);
    }
    p2 = p1;
  }
}

void FieldDimensions::read(In& stream)
{
  In* theStream = &stream;

  // read from a file?
  if(dynamic_cast<InMapFile*>(theStream))
  {
    // if yes, try to load JSON file instead
#ifdef TARGET_ROBOT
    const char* filename = "/media/usb/field_dimensions.json";
#else
    const char* filename = "field_dimensions.json";
#endif
    InMapFile jsonStream(filename, ~bit(InMap::unusedAttribute));
    if(jsonStream.exists())
    {
      STREAMABLE(Dimensions,
      {
        STREAMABLE(Field,
        {,
          (float) length,
          (float) width,
          (float) penaltyCrossSize,
          (float) penaltyAreaLength,
          (float) penaltyAreaWidth,
          (float) penaltyCrossDistance,
          (float) centerCircleDiameter,
          (float) borderStripWidth,
        });

        STREAMABLE(Goal,
        {,
          (float) postDiameter,
          (float) height,
          (float) innerWidth,
          (float) depth,
        }),

        (Field) field,
        (Goal) goal,
      }) dims;

      STREAMABLE(DimensionsOpt,
      {
        STREAMABLE(Field,
        {,
          (float)(-1.f) goalBoxAreaLength,
          (float)(-1.f) goalBoxAreaWidth,
        }),

        (Field) field,
      }) dimsOpt;

      jsonStream >> dims;
      InMapFile(filename, ~(bit(InMap::missingAttribute) | bit(InMap::unusedAttribute))) >> dimsOpt;

      const float lineWidth = 0.05f;
      // The rulebook says this offset from the touchline should be 0.5m.
      // However, it should be at least 0.2m inwards from the carpet border, unless the border strip is too narrow.
      const float returnFromPenaltyOffsetY = std::clamp(dims.field.borderStripWidth - 0.2f, 0.f, 0.5f);

      // pre-define some values and use a template for field dimensions to generate everything else
      theStream = new InSymbolicMapFile(dimsOpt.field.goalBoxAreaLength != -1.f
                                        && (std::abs(dimsOpt.field.goalBoxAreaLength - dims.field.penaltyAreaLength) > 0.001f
                                            || std::abs(dimsOpt.field.goalBoxAreaWidth - dims.field.penaltyAreaWidth) > 0.001f)
                                        ? "fieldDimensions2020.cfg" : "fieldDimensions2015.cfg",
      {
        {"xPosOpponentFieldBorder", dims.field.length * 500.f + dims.field.borderStripWidth * 1000.f},
        {"xPosOpponentGoal", dims.field.length * 500.f - lineWidth * 500.f + dims.goal.depth * 1000.f},
        {"xPosOpponentGoalPost", dims.field.length * 500.f + lineWidth * 500.f},
        {"xPosOpponentGoalLine", dims.field.length * 500.f},
        {"xPosOpponentPenaltyArea", dims.field.length * 500.f - dims.field.penaltyAreaLength * 1000.f},
        {"xPosOpponentPenaltyMark", dims.field.length * 500.f - dims.field.penaltyCrossDistance * 1000.f},
        {"xPosOpponentGoalArea", dims.field.length * 500.f - dimsOpt.field.goalBoxAreaLength * 1000.f},
        {"yPosLeftFieldBorder", dims.field.width * 500.f + dims.field.borderStripWidth * 1000.f},
        {"yPosLeftReturnFromPenalty", dims.field.width * 500.f + returnFromPenaltyOffsetY * 1000.f},
        {"yPosLeftTouchline", dims.field.width * 500.f},
        {"yPosLeftPenaltyArea", dims.field.penaltyAreaWidth * 500.f},
        {"yPosLeftGoal", dims.goal.innerWidth * 500.f + dims.goal.postDiameter * 500.f},
        {"yPosLeftGoalArea", dimsOpt.field.goalBoxAreaWidth * 500.f},
        {"fieldLinesWidth", lineWidth * 1000.f},
        {"centerCircleRadius", dims.field.centerCircleDiameter * 500.f},
        {"goalPostRadius", dims.goal.postDiameter * 500.f},
        {"goalHeight", dims.goal.height * 1000.f},
        {"penaltyMarkSize", dims.field.penaltyCrossSize * 1000.f},
        {"xPenaltyMarkClose", dims.field.length * 500.f - dims.field.penaltyCrossDistance * 1000.f - dims.field.penaltyCrossSize * 500.f},
        {"xPenaltyMarkFar", dims.field.length * 500.f - dims.field.penaltyCrossDistance * 1000.f + dims.field.penaltyCrossSize * 500.f},
        {"penaltyMarkRadius", dims.field.penaltyCrossSize * 500.f}
      });
    }
  }

  {
    In& stream = *theStream;

    STREAM_BASE(SimpleFieldDimensions);

    std::vector<LinesTable::Line>& fieldLines(straightFieldLines.lines);
    std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);

    STREAM(goalFrameLines);
    STREAM(fieldLines);
    STREAM(centerCircle);
    STREAM(corners);

    // merge straightFieldLines and centerCircle to fieldLines
    this->fieldLines = straightFieldLines;
    this->fieldLines.pushCircle(centerCircle.center, centerCircle.radius, centerCircle.numOfSegments);

    // merge fieldLines and goalFrameLines into fieldLinesWithGoalFrame
    fieldLinesWithGoalFrame.lines.clear();
    for(LinesTable::Line& line : this->fieldLines.lines)
      fieldLinesWithGoalFrame.lines.push_back(line);
    for(LinesTable::Line& line : goalFrameLines)
      fieldLinesWithGoalFrame.lines.push_back(line);
  }

  if(theStream != &stream)
    delete theStream;
}

void FieldDimensions::write(Out& stream) const
{
  STREAM_BASE(SimpleFieldDimensions);

  const std::vector<LinesTable::Line>& fieldLines(straightFieldLines.lines);
  const std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);

  STREAM(goalFrameLines);
  STREAM(fieldLines);
  STREAM(centerCircle);
  STREAM(corners);
}

void FieldDimensions::reg()
{
  PUBLISH(reg);
  REG_CLASS_WITH_BASE(FieldDimensions, SimpleFieldDimensions);
  REG(std::vector<LinesTable::Line>, goalFrameLines);
  REG(std::vector<LinesTable::Line>, fieldLines);
  REG(centerCircle);
  REG(corners);
}
