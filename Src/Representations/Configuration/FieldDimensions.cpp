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
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Debugging/DebugDrawings.h"
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
  InSymbolicMapFile(const std::string& name) : InMapFile(name) {}

  virtual void select(const char* name, int type, const char* (*enumToString)(int))
  {
    Streaming::trimName(name);
    InMapFile::select(name, type, enumToString);
    entry = name;
  }

protected:
  /**
   * When reading a float, read a string instead. Try to replace symbolic value.
   * Symbolic value can be preceeded by a minus sign (without whitespace in between).
   */
  virtual void inFloat(float& value)
  {
    std::string buf;
    inString(buf);
    float sign = 1.f;
    if(buf[0] == '-')
    {
      sign = -1.f;
      buf = buf.substr(1);
    }

    std::unordered_map<std::string, float>::const_iterator i = values.find(buf);
    if(i != values.end())
      value = i->second * sign;
    else if(!buf.empty() && (isdigit(buf[0]) || buf[0] == '.'))
      value = static_cast<float>(strtod(buf.c_str(), 0)) * sign;
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
  drawLines();
  drawGoalFrame();
  drawCorners();
}

void FieldDimensions::drawGoalFrame() const
{
  DECLARE_DEBUG_DRAWING("goal frame", "drawingOnField");
  COMPLEX_DRAWING("goal frame")
  {
    for(const LinesTable::Line& l : goalFrameLines.lines)
    {
      ColorRGBA lineColor(192, 192, 192);
      LINE("goal frame", l.from.x(), l.from.y(), l.to.x(), l.to.y(), fieldLinesWidth * 0.7, Drawings::solidPen, lineColor);
    }
  }
}

void FieldDimensions::drawLines() const
{
  DECLARE_DEBUG_DRAWING("field lines", "drawingOnField");
  COMPLEX_DRAWING("field lines")
  {
    ASSERT(carpetBorder.lines.size() <= 4);
    Vector2f points[4];
    for(unsigned i = 0; i < carpetBorder.lines.size(); ++i)
      points[i] = carpetBorder.lines[i].from;
    POLYGON("field lines", static_cast<int>(carpetBorder.lines.size()), points, 0, Drawings::solidPen, ColorRGBA(0, 180, 0), Drawings::solidBrush, ColorRGBA(0, 140, 0));

    ColorRGBA lineColor(192, 192, 192);
    for(vector<LinesTable::Line>::const_iterator i = fieldLines.lines.begin(); i != fieldLines.lines.end(); ++i)
    {
      LINE("field lines", i->from.x(), i->from.y(), i->to.x(), i->to.y(), fieldLinesWidth, Drawings::solidPen, lineColor);
    }
  }
}

void FieldDimensions::drawPolygons(int ownColor) const
{
  DECLARE_DEBUG_DRAWING("field polygons", "drawingOnField");
  COMPLEX_DRAWING("field polygons")
  {
    static const ColorRGBA colors[] =
    {
      ColorRGBA(50, 120, 127),
      ColorRGBA(127, 80, 80),
      ColorRGBA(127, 120, 50),
      ColorRGBA(80, 80, 80)
    };
    const ColorRGBA& own = colors[ownColor];
    const ColorRGBA& opp = colors[1 ^ ownColor];

    Vector2f goal[4];
    goal[0] = Vector2f(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2f(xPosOwnGroundline - fieldLinesWidth * 0.5f, yPosRightGoal);
    goal[2] = Vector2f(xPosOwnGoal, yPosRightGoal);
    goal[3] = Vector2f(xPosOwnGoal, yPosLeftGoal);
    POLYGON("field polygons", 4, goal, 0, Drawings::solidPen, own, Drawings::solidBrush, own);

    goal[0] = Vector2f(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosLeftGoal);
    goal[1] = Vector2f(xPosOpponentGroundline + fieldLinesWidth * 0.5f, yPosRightGoal);
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

void FieldDimensions::drawCorners() const
{
  DECLARE_DEBUG_DRAWING("field corners", "drawingOnField");
  CornerClass c = xCorner;
  MODIFY_ENUM("fieldDimensions:cornerClass", c);
  COMPLEX_DRAWING("field corners")
  {
    for(auto i = corners[c].begin(); i != corners[c].end(); ++i)
      LARGE_DOT("field corners", i->x(), i->y(), ColorRGBA(255, 255, 255), ColorRGBA(255, 255, 255));
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

bool FieldDimensions::LinesTable::getClosestIntersection(const Geometry::Line& l, Vector2f& outIntersection) const
{
  int wayne;
  return getClosestIntersection(l, wayne, outIntersection);
}

bool FieldDimensions::LinesTable::getClosestIntersection(const Geometry::Line& l, int& outLineIndex, Vector2f& outIntersection) const
{
  float currentMinimumDistance = std::numeric_limits<float>::max(); //square distance
  Vector2f closestPoint(-100000.f, -1000000.f);
  bool found = false;
  for(int i = 0; i < (int)lines.size(); ++i)
  {
    const Line& fieldLine = lines[i];
    Vector2f intersection;
    Geometry::Line line(fieldLine.from, (fieldLine.to - fieldLine.from).normalized());
    if(Geometry::getIntersectionOfLines(l, line, intersection))
    {
      //we already know that the intersection is on the line,
      //just need to know if it is inside the rectangle defined by the field line coordinates
      Rangef xRange(std::min(fieldLine.from.x(), fieldLine.to.x()), std::max(fieldLine.from.x(), fieldLine.to.x()));
      Rangef yRange(std::min(fieldLine.from.y(), fieldLine.to.y()), std::max(fieldLine.from.y(), fieldLine.to.y()));
      if(xRange.isInside(intersection.x()) &&
         yRange.isInside(intersection.y()))
      {
        const float squareDist = (l.base - intersection).squaredNorm();
        if(squareDist < currentMinimumDistance)
        {
          found = true;
          outLineIndex = i;
          currentMinimumDistance = squareDist;
          closestPoint = intersection;
        }
      }
    }
  }
  if(found)
    outIntersection = closestPoint;
  return found;
}

Vector2f FieldDimensions::LinesTable::getClosestPoint(const Vector2f& point) const
{
  float currentMinimumDistance = std::numeric_limits<float>::max();
  Vector2f closestPoint(-1.f, -1.f);
  Vector2f tempClosestPoint(-1.f, -1.f);

  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    Geometry::Line line(i->from, (i->to - i->from).normalized());
    //calculate orthogonal projection of point onto line (see http://de.wikipedia.org/wiki/Orthogonalprojektion)
    const float numerator = (point - line.base).dot(line.direction);
    const Vector2f projection = line.base + (line.direction * numerator);

    //check if projected point is on the line segment
    //We already know that it is on the line, therefore we just need to check
    //if it is inside the rectangle created by start and end point of the line
    const Vector2f& a = line.base; //a, b and c only exist to make the following if readable
    const Vector2f& b = i->to;
    const Vector2f& c = projection;

    float distance = -1;
    //not optimized expression
//    if(c.x >= b.x && c.x <= a.x && c.y >= b.y && c.y <= a.y ||
//       c.x >= a.x && c.x <= b.x && c.y >= a.y && c.y <= b.y ||
//       c.x >= a.x && c.x <= b.x && c.y >= b.y && c.y <= a.y ||
//       c.x >= b.x && c.x <= a.x && c.y >= a.y && c.y <= b.y)
    //optimized expression
    if(((c.y() >= b.y() && c.y() <= a.y()) || (c.y() >= a.y() && c.y() <= b.y())) &&
       ((c.x() >= b.x() && c.x() <= a.x()) || (c.x() >= a.x() && c.x() <= b.x())))
    {
      //If projection is on the line segment just calculate the distance between the point
      //and it's projection.
      distance = (projection - point).norm();
      tempClosestPoint = projection;
    }
    else
    {
      //If the projection is not on the line segment it is either left or
      //right of the line segment. Therefore use distance to edge
      const float distBase = (line.base - point).norm();
      const float distEnd = (i->to - point).norm();
      if(distBase <= distEnd)
      {
        distance = distBase;
        tempClosestPoint = line.base;
      }
      else
      {
        distance = distEnd;
        tempClosestPoint = i->to;
      }
    }

    if(distance < currentMinimumDistance)
    {
      currentMinimumDistance = distance;
      closestPoint = tempClosestPoint;
    }
  }
  return closestPoint;
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

bool FieldDimensions::LinesTable::isInside(const Vector2f& v) const
{
  //note:
  //This function assumes that the point (0,0) is inside and
  //that for any point inside the area the line to (0,0) belongs to the area too.

  Geometry::Line testLine(v, -v);
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
  {
    float factor;
    Geometry::Line border(i->from, i->to - i->from);
    if(Geometry::getIntersectionOfRaysFactor(border, testLine, factor))
      return false;
  }
  return true;
}

float FieldDimensions::LinesTable::clip(Vector2f& v) const
{
  if(isInside(v))
    return 0;
  else
  {
    const Vector2f old = v;
    Vector2f v2;
    float minDist = 100000;
    for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    {
      if(old == i->from)
        v2 = i->from;
      else
      {
        float a = (old - i->from).dot(i->to - i->from) / (i->to - i->from).dot(i->to - i->from);
        if(a <= 0)
          v2 = i->from;
        else if(a >= 1.f)
          v2 = i->to;
        else
          v2 = i->from + a * (i->to - i->from);
      }
      float dist = (old - v2).norm();
      if(minDist > dist)
      {
        minDist = dist;
        v = v2;
      }
    }
    return (v - old).norm();
  }
}

bool FieldDimensions::LinesTable::getClosestPoint(Vector2f& vMin, const Pose2f& p, int numberOfRotations, float minLength) const
{
  int trueNumberOfRotations = numberOfRotations;
  if(numberOfRotations == 2)
    numberOfRotations = 4;

  // target angle -> target index
  float r = p.rotation / pi2 * numberOfRotations + 0.5f;
  if(r < 0)
    r += numberOfRotations;
  int targetRot = int(r);
  ASSERT(targetRot >= 0 && targetRot < numberOfRotations);
  targetRot %= trueNumberOfRotations;
  Vector2f v2;
  float minDist = 100000;
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if((i->to - i->from).squaredNorm() >= sqr(minLength))
    {
      // angle -> index
      float r = ((i->to - i->from).angle() + pi_2) / pi2 * numberOfRotations + 0.5f;
      if(r < 0)
        r += numberOfRotations;
      else if(r >= numberOfRotations)
        r -= numberOfRotations;
      int rot = int(r);
      ASSERT(rot >= 0 && rot < numberOfRotations);
      rot %= trueNumberOfRotations;

      // index must be target index
      if(rot == targetRot)
      {
        if(p.translation == i->from)
          v2 = i->from;
        else
        {
          float a = (p.translation - i->from).dot(i->to - i->from) / ((p.translation - i->from).norm() * (i->to - i->from).norm());
          if(a <= 0)
            v2 = i->from;
          else if(a >= 1.f)
            v2 = i->to;
          else
            v2 = i->from + a * (i->to - i->from);
        }
        const Vector2f vDiff = v2 - p.translation;
        float dist = vDiff.norm();
        if(minDist > dist)
        {
          minDist = dist;
          vMin = v2;
        }
      }
    }
  return (minDist < 100000);
}

float FieldDimensions::LinesTable::getDistance(const Pose2f& pose) const
{
  float minDist = 100000;
  for(vector<Line>::const_iterator i = lines.begin(); i != lines.end(); ++i)
    if(i->from.y() < 0 && i->to.y() > 0)
    {
      const float dist = i->from.x() + (i->to.x() - i->from.x()) * -i->from.y() / (i->to.y() - i->from.y());
      if(dist >= 0 && dist < minDist)
        minDist = dist;
    }
  return minDist == 100000 ? -1 : minDist;
}

void FieldDimensions::serialize(In* in, Out* out)
{
  ASSERT(in); // handling center circle only works when reading

  std::vector<LinesTable::Line>& carpetBorder(this->carpetBorder.lines);
  std::vector<LinesTable::Line>& fieldBorder(this->fieldBorder.lines);
  std::vector<LinesTable::Line>& fieldLines(this->fieldLines.lines);
  std::vector<LinesTable::Line>& goalFrameLines(this->goalFrameLines.lines);
  LinesTable::Circle centerCircle;

  STREAM_REGISTER_BEGIN;
  STREAM_BASE(SimpleFieldDimensions)
  STREAM(carpetBorder);
  STREAM(goalFrameLines);
  STREAM(fieldBorder);
  STREAM(fieldLines);
  STREAM(centerCircle);
  this->fieldLines.pushCircle(centerCircle.center, centerCircle.radius, centerCircle.numOfSegments);

  STREAM(corners);
  STREAM_REGISTER_FINISH;

  //merge fieldLines and goalFrameLines into fieldLinesWithGoalFrame
  for(LinesTable::Line& line : fieldLines)
    fieldLinesWithGoalFrame.lines.push_back(line);
  for(LinesTable::Line& line : goalFrameLines)
    fieldLinesWithGoalFrame.lines.push_back(line);
}
