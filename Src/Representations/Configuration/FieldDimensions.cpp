/**
 * @file FieldDimensions.cpp
 *
 * Some useful functions regarding field dimensions.
 *
 * @author Max Risler
 */

#include "FieldDimensions.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Eigen.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"
#include <algorithm>

using namespace std;

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
    else if(!buf.empty() && (isdigit(buf[0]) || buf[0] == '.'))
      value = static_cast<float>(strtod(buf.c_str(), nullptr)) * sign;
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
  }
}

void FieldDimensions::drawLines() const
{
  DEBUG_DRAWING("field lines", "drawingOnField")
  {
    Vector2f points[4];
    points[0].x() = points[1].x() = xPosOpponentFieldBorder;
    points[2].x() = points[3].x() = xPosOwnFieldBorder;
    points[0].y() = points[3].y() = yPosLeftFieldBorder;
    points[1].y() = points[2].y() = yPosRightFieldBorder;
    POLYGON("field lines", 4, points, 0, Drawings::solidPen, ColorRGBA(0, 140, 0), Drawings::solidBrush, ColorRGBA(0, 140, 0));

    const ColorRGBA lineColor(224, 224, 224);
    for(vector<LinesTable::Line>::const_iterator i = fieldLines.lines.begin(); i != fieldLines.lines.end(); ++i)
    {
      if(!i->isPartOfCircle)
        LINE("field lines", i->from.x(), i->from.y(), i->to.x(), i->to.y(), fieldLinesWidth, Drawings::solidPen, lineColor);
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
  if(Blackboard::getInstance().exists("OwnTeamInfo") && Blackboard::getInstance().exists("OpponentTeamInfo"))
  {
    const OwnTeamInfo& ownTeamInfo = static_cast<const OwnTeamInfo&>(Blackboard::getInstance()["OwnTeamInfo"]);
    const OpponentTeamInfo& opponentTeamInfo = static_cast<const OpponentTeamInfo&>(Blackboard::getInstance()["OpponentTeamInfo"]);

    DEBUG_DRAWING("field polygons", "drawingOnField")
    {
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
      const ColorRGBA& own = colors[ownTeamInfo.teamColor];
      const ColorRGBA& opp = colors[opponentTeamInfo.teamColor];

      Vector2f goal[4];
      goal[0] = Vector2f(xPosOwnGroundLine - fieldLinesWidth * 0.5f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOwnGroundLine - fieldLinesWidth * 0.5f, yPosRightGoal);
      goal[2] = Vector2f(xPosOwnGoal, yPosRightGoal);
      goal[3] = Vector2f(xPosOwnGoal, yPosLeftGoal);
      POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, own, Drawings::solidBrush, own);

      goal[0] = Vector2f(xPosOpponentGroundLine + fieldLinesWidth * 0.5f, yPosLeftGoal);
      goal[1] = Vector2f(xPosOpponentGroundLine + fieldLinesWidth * 0.5f, yPosRightGoal);
      goal[2] = Vector2f(xPosOpponentGoal, yPosRightGoal);
      goal[3] = Vector2f(xPosOpponentGoal, yPosLeftGoal);
      POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, opp, Drawings::solidBrush, opp);

      CIRCLE("field polygons", xPosOpponentGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen,
             ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
      CIRCLE("field polygons", xPosOpponentGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen,
             ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);

      CIRCLE("field polygons", xPosOwnGoalPost, yPosLeftGoal, 50, 0, Drawings::solidPen,
             ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
      CIRCLE("field polygons", xPosOwnGoalPost, yPosRightGoal, 50, 0, Drawings::solidPen,
             ColorRGBA::white, Drawings::solidBrush, ColorRGBA::white);
    }
  }
}

void FieldDimensions::drawCorners() const
{
  DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");
  CornerClass c = xCorner;
  MODIFY("fieldDimensions:cornerClass", c);
  COMPLEX_DRAWING("field corners")
  {
    for(auto i = corners[c].begin(); i != corners[c].end(); ++i)
      LARGE_DOT("field corners", i->x(), i->y(), ColorRGBA(255, 0, 0), ColorRGBA(255, 0, 0));
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
    p1 = Vector2f(sin(a), cos(a)) * radius;
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
    InMapFile jsonStream("/media/usb/field_dimensions.json", ~bit(InMap::missingAttribute));
#else
    InMapFile jsonStream("field_dimensions.json", ~0);
#endif
    if(jsonStream.exists())
    {
      STREAMABLE(Dimensions,
      {
        STREAMABLE(Field,
        {,
          (float) length,
          (float) width,
          (float) penaltyCrossSize,
          (float)(-1.f) goalBoxAreaLength,
          (float)(-1.f) goalBoxAreaWidth,
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

      jsonStream >> dims;

      const float lineWidth = 0.05f;

      // pre-define some values and use a template for field dimensions to generate everything else
      theStream = new InSymbolicMapFile(dims.field.goalBoxAreaLength != -1.f
                                        && (std::abs(dims.field.goalBoxAreaLength - dims.field.penaltyAreaLength) > 0.001f
                                            || std::abs(dims.field.goalBoxAreaWidth - dims.field.penaltyAreaWidth) > 0.001f)
                                        ? "fieldDimensions2020.cfg" : "fieldDimensions2015.cfg",
      {
        {"xPosOpponentFieldBorder", dims.field.length * 500.f + dims.field.borderStripWidth * 1000.f},
        {"xPosOpponentGoal", dims.field.length * 500.f - lineWidth * 500.f + dims.goal.depth * 1000.f},
        {"xPosOpponentGoalPost", dims.field.length * 500.f + lineWidth * 500.f},
        {"xPosOpponentGroundLine", dims.field.length * 500.f},
        {"xPosOpponentPenaltyArea", dims.field.length * 500.f - dims.field.penaltyAreaLength * 1000.f},
        {"xPosOpponentPenaltyMark", dims.field.length * 500.f - dims.field.penaltyCrossDistance * 1000.f},
        {"xPosOpponentGoalArea", dims.field.length * 500.f - dims.field.goalBoxAreaLength * 1000.f},
        {"yPosLeftFieldBorder", dims.field.width * 500.f + dims.field.borderStripWidth * 1000},
        {"yPosLeftSideline", dims.field.width * 500.f},
        {"yPosLeftPenaltyArea", dims.field.penaltyAreaWidth * 500.f},
        {"yPosLeftGoal", dims.goal.innerWidth * 500.f + dims.goal.postDiameter * 500.f},
        {"yPosLeftGoalArea", dims.field.goalBoxAreaWidth * 500.f},
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
