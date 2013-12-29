#include "FieldCoverageProvider.h"

#include "Tools/Team.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include <algorithm>
#include <list>
#include <sstream>

MAKE_MODULE(FieldCoverageProvider, Modeling);

FieldCoverageProvider::FieldCoverageProvider() :
range(99999),
lastWorstIdx(-1),
lastWorstNoTurnIdx(-1),
lastWorstNoTurnRangeIdx(-1),
lastWorstNoTurnHalfRangeIdx(-1),
visibleAngle(fromDegrees(20.0f)),
throwInLineDistanceSL(400.0f),
throwInLineDistanceGL(1000.f),
throwInLineOffset(0),
lastThrowInTimestamp(0),
throwInPenaltyDistance(1000.f),
throwInCellDiff(10),
minThrowInCellCoverage((xSteps - 1) * throwInCellDiff),
lastGameState(STATE_INITIAL)
{
  const float halfFieldLength = theFieldDimensions.xPosOpponentGroundline;
  const float halfFieldWidth = theFieldDimensions.yPosLeftSideline;
  cellLength.x = 2.0f * halfFieldLength / (float) xSteps;
  cellLength.y = 2.0f * halfFieldWidth / (float) ySteps;
  throwInLineOffset = static_cast<int>(throwInLineDistanceSL / cellLength.y);
  yPosLeftThrowInLine = halfFieldWidth - throwInLineDistanceSL;
  yPosRightThrowInLine = -yPosLeftThrowInLine;
  xPosOpponentThrowInPoint = halfFieldLength - throwInLineDistanceGL;
  xPosOwnThrowInPoint = -xPosOpponentThrowInPoint;
  lastGameState = theGameInfo.state;

  for(size_t x = 0; x < xSteps; x++)
    for(size_t y = 0; y < ySteps; y++)
    {
      const float xMin = cellLength.x * (float) x - halfFieldLength, yMin = cellLength.y * (float) y - halfFieldWidth;
      Cell* c = new Cell(xMin, xMin + cellLength.x, yMin, yMin + cellLength.y);
      fieldCoverageGrid[x][y] = c;
    }

  std::vector<FieldDimensions::LinesTable::Line>::const_iterator it = theFieldDimensions.fieldBorder.lines.begin();
  std::vector<FieldDimensions::LinesTable::Line>::const_iterator end = theFieldDimensions.fieldBorder.lines.end();
  for(; it != end; ++it)
    fieldBorder.push_back(Geometry::Line(it->corner.translation, Vector2<>(it->length, 0.f).rotate(it->corner.rotation)));
}

FieldCoverageProvider::~FieldCoverageProvider()
{
}

void FieldCoverageProvider::update(FieldCoverage& fieldCoverage)
{
  drawFieldView();

  DECLARE_PLOT("module:FieldCoverageProvider:mean");
  DECLARE_PLOT("module:FieldCoverageProvider:stddev");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:cameraRange", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:shadows", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:fieldCoverage", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:shadowsOnImage", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:worstTarget", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:worstNoTurnTarget", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:worstNoTurnRangeTarget", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:worstNoTurnHalfRangeTarget", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:worstNoTurnTargetImage", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:throwInCell", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:outPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:throwInPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("module:FieldCoverageProvider:cameraRange", "origin");

  if(theCombinedWorldModel.ballIsValid)
  {
    if(theFieldDimensions.isInsideField(theCombinedWorldModel.ballState.position))
    {
      lastValidInsideBall.position = theCombinedWorldModel.ballState.position;
      lastValidInsideBall.time = theFrameInfo.time;
      if(theCombinedWorldModel.ballState.velocity.abs() < 1.f)
        lastValidInsideLyingBall = lastValidInsideBall;
    }
    else
    {
      lastValidOutsideBall.position = theCombinedWorldModel.ballState.position;
      lastValidOutsideBall.time = theFrameInfo.time;
    }
  }

  // Precompute relative targets and distances.
  const Pose2D robotPoseInv = theRobotPose.invert();
  for(size_t x = 0; x < xSteps; ++x)
    for(size_t y = 0; y < ySteps; ++y)
    {
      Cell* cell = fieldCoverageGrid[x][y];
      cell->relativeTargetOnField = robotPoseInv * cell->absoluteCameraTargetOnField;
      cell->distance = cell->relativeTargetOnField.abs();
    }

  if(theCameraMatrix.isValid && theRobotPose.deviation < 50.0f)
  {

    const int xMax = theCameraInfo.width - 1;
    const int yMin = (int)std::max(theImageCoordinateSystem.origin.y + theImageCoordinateSystem.rotation.c[1].y * 25, 0.f);
    const int yMax = theCameraInfo.height - 1;

    // absolute positions of the corners of the camera image on the field
    Vector2<> cameraRange[4];
    cameraRange[0] = projectOnField(0, yMax);
    cameraRange[1] = projectOnField(0, yMin);
    cameraRange[2] = projectOnField(xMax, yMin);
    cameraRange[3] = projectOnField(xMax, yMax);

    POLYGON("module:FieldCoverageProvider:cameraRange", 4, cameraRange, 20,
            Drawings::ps_dash, ColorClasses::green, Drawings::bs_solid, ColorRGBA(0, 255, 0, 50));

    COMPLEX_DRAWING3D("module:FieldCoverageProvider:cameraRange",
    {
      TRANSLATE3D("module:FieldCoverageProvider:cameraRange", 0, 0, -210);
      for(int i = 0; i < 4; ++i)
      {
        Vector2<> p = (Pose2D(cameraRange[i]) - theRobotPose).translation;
        Vector2<> p2 = (Pose2D(cameraRange[(i + 1) & 3]) - theRobotPose).translation;
        LINE3D("module:FieldCoverageProvider:cameraRange",
               theCameraMatrix.translation.x, theCameraMatrix.translation.y, theCameraMatrix.translation.z,
               p.x, p.y, 0, 2, ColorClasses::blue);
        LINE3D("module:FieldCoverageProvider:cameraRange", p.x, p.y, 0, p2.x, p2.y, 0, 2, ColorClasses::blue);
      }
    });

    // precompute shadows
    std::vector<RobotShadow> shadows;
    computeShadows(shadows, theRobotPose, theRobotsModel);

    // Draw shadows
    COMPLEX_DRAWING("module:FieldCoverageProvider:shadows",
    {
      std::vector<RobotShadow>::const_iterator it = shadows.begin();
      const std::vector<RobotShadow>::const_iterator end = shadows.end();
      for(; it != end; ++it)
        it->draw();
    });
    COMPLEX_DRAWING("module:FieldCoverageProvider:shadowsOnImage",
    {
      std::vector<RobotShadow>::const_iterator it = shadows.begin();
      const std::vector<RobotShadow>::const_iterator end = shadows.end();
      for(; it != end; ++it)
        it->drawOnImage(theCameraMatrix, theCameraInfo, theImageCoordinateSystem);
    });

    // refresh every cell that is visible, i. e. the center is visible
    for(size_t x = 0; x < xSteps; x++)
    {
      for(size_t y = 0; y < ySteps; y++)
      {
        Cell& c = *fieldCoverageGrid[x][y];
        if(Geometry::isPointInsideConvexPolygon(cameraRange, 4, c.absoluteCameraTargetOnField))
        {
          // Cell is visibile if it's not shadowed.
          if(isCellShadowed(shadows, c))
          {
            c.shadowed = true;
          }
          else
          {
            c.shadowed = false;
            c.lastseenScanPattern = theFrameInfo.time;
            if(c.distance < range)
            {
              c.lastseen = theFrameInfo.time;
              fieldCoverage.cells[x * ySteps + y] = theFrameInfo.time;
            }
          }
        }
      }
    }
  }

  // Set cells which are cut by the throw-in lines to be uncovered if the ball is thrown in.
  ballThrowIn(fieldCoverage);
  COMPLEX_DRAWING("module:FieldCoverageProvider:fieldCoverage",
  {
    for(size_t x = 0; x < xSteps; x++)
      for(size_t y = 0; y < ySteps; y++)
      {
        const Cell& c = *fieldCoverageGrid[x][y];
        ColorRGBA color = ColorRGBA(255, 0, 0, 255 - c.coverage(theFrameInfo.time));
        QUADRANGLE("module:FieldCoverageProvider:fieldCoverage",
                   c.polygon[0].x, c.polygon[0].y,
                   c.polygon[1].x, c.polygon[1].y,
                   c.polygon[2].x, c.polygon[2].y,
                   c.polygon[3].x, c.polygon[3].y,
                   20, Drawings::ps_solid, color);
      }
  });
  // Calculate worst-covered cells.
  Cell** cells = (Cell**) fieldCoverageGrid;
  unsigned short worstCoverage = Cell::maxCoverage + 1;
  unsigned int worstNoTurnScanTimestamp = theFrameInfo.time + 1;
  unsigned short worstNoTurnRangeCoverage = worstCoverage;
  unsigned short worstNoTurnHalfRangeCoverage = worstCoverage;
  int worstNoTurnHalfRangeIdx = 0;
  int worstNoTurnRangeIdx = 0;
  int worstNoTurnIdx = 0;
  int worstIdx = 0;
  ASSERT(worstCoverage > Cell::maxCoverage);
  for(size_t i = 0; i < xSteps * ySteps; i++)
  {
    Cell* cell = cells[i];
    if(!cell->shadowed)
    {
      const unsigned char coverage = cell->coverage(theFrameInfo.time);
      if(coverage < worstCoverage)
      {
        worstCoverage = coverage;
        worstIdx = i;
      }
      if(std::abs(cell->relativeTargetOnField.angle()) < visibleAngle)
      {
        if(cell->lastseenScanPattern < worstNoTurnScanTimestamp)
        {
          worstNoTurnScanTimestamp = cell->lastseenScanPattern;
          worstNoTurnIdx = i;
        }
        if(coverage < worstNoTurnRangeCoverage && cell->distance < range)
        {
          worstNoTurnRangeCoverage = coverage;
          worstNoTurnRangeIdx = i;
        }
        if(coverage < worstNoTurnHalfRangeCoverage && cell->distance < range / 2.0f)
        {
          worstNoTurnHalfRangeCoverage = coverage;
          worstNoTurnHalfRangeIdx = i;
        }
      }
    }
  }

  // Update targets.
  if(lastWorstIdx == -1 || cells[worstIdx]->coverage(theFrameInfo.time) < cells[lastWorstIdx]->coverage(theFrameInfo.time))
  {
    fieldCoverage.worstTarget = FieldCoverage::Target(cells[worstIdx]->relativeTargetOnField, worstCoverage);
  }
  else
  {
    fieldCoverage.worstTarget = FieldCoverage::Target(cells[lastWorstIdx]->relativeTargetOnField,
                                cells[lastWorstIdx]->coverage(theFrameInfo.time));
  }

  if(lastWorstNoTurnIdx == -1 || std::abs(cells[lastWorstNoTurnIdx]->relativeTargetOnField.angle()) > visibleAngle + 0.1
     || worstNoTurnScanTimestamp < cells[lastWorstNoTurnIdx]->lastseenScanPattern)
  {
    fieldCoverage.worstNoTurnTarget = FieldCoverage::Target(cells[worstNoTurnIdx]->relativeTargetOnField,
                                      cells[worstNoTurnIdx]->coverage(theFrameInfo.time, worstNoTurnScanTimestamp));
    lastWorstNoTurnIdx = worstNoTurnIdx;
  }
  else
  {
    fieldCoverage.worstNoTurnTarget = FieldCoverage::Target(cells[lastWorstNoTurnIdx]->relativeTargetOnField,
                                      cells[lastWorstNoTurnIdx]->coverage(theFrameInfo.time));
  }

  if(lastWorstNoTurnRangeIdx == -1 || worstNoTurnRangeCoverage < cells[lastWorstNoTurnRangeIdx]->coverage(theFrameInfo.time)
     || std::abs(cells[lastWorstNoTurnRangeIdx]->relativeTargetOnField.angle()) > visibleAngle + 0.1
     || cells[lastWorstNoTurnRangeIdx]->distance > range)
  {
    fieldCoverage.worstNoTurnRangeTarget = FieldCoverage::Target(cells[worstNoTurnRangeIdx]->relativeTargetOnField, worstNoTurnRangeCoverage);
    lastWorstNoTurnRangeIdx = worstNoTurnRangeIdx;
  }
  else
  {
    fieldCoverage.worstNoTurnRangeTarget = FieldCoverage::Target(cells[lastWorstNoTurnRangeIdx]->relativeTargetOnField,
                                           cells[lastWorstNoTurnRangeIdx]->coverage(theFrameInfo.time));
  }

  if(lastWorstNoTurnHalfRangeIdx == -1 || worstNoTurnHalfRangeCoverage < cells[lastWorstNoTurnHalfRangeIdx]->coverage(theFrameInfo.time)
     || std::abs(cells[lastWorstNoTurnHalfRangeIdx]->relativeTargetOnField.angle()) > visibleAngle + 0.1
     || cells[lastWorstNoTurnHalfRangeIdx]->distance < range / 2.f)
  {
    fieldCoverage.worstNoTurnHalfRangeTarget = FieldCoverage::Target(cells[worstNoTurnHalfRangeIdx]->relativeTargetOnField,
        worstNoTurnHalfRangeCoverage);
    lastWorstNoTurnHalfRangeIdx = worstNoTurnHalfRangeIdx;
  }
  else
  {
    fieldCoverage.worstNoTurnHalfRangeTarget = FieldCoverage::Target(cells[lastWorstNoTurnHalfRangeIdx]->relativeTargetOnField,
        cells[lastWorstNoTurnHalfRangeIdx]->coverage(theFrameInfo.time));
  }

  // Drawing concerning the state of the field coverage and targets of worst-covered cells.
  COMPLEX_DRAWING("module:FieldCoverageProvider:worstTarget",
  {
    if(fieldCoverage.worstTarget.isValid)
    {
      const Vector2<> abs = theRobotPose * fieldCoverage.worstTarget.target;
      CROSS("module:FieldCoverageProvider:worstTarget",
            abs.x, abs.y, 100, 20, Drawings::ps_solid, ColorClasses::blue);
    }
  });

  COMPLEX_DRAWING("module:FieldCoverageProvider:worstNoTurnTarget",
  {
    if(fieldCoverage.worstNoTurnTarget.isValid)
    {
      const Vector2<> abs = theRobotPose * fieldCoverage.worstNoTurnTarget.target;
      CROSS("module:FieldCoverageProvider:worstNoTurnTarget",
            abs.x, abs.y, 100, 20, Drawings::ps_solid, ColorClasses::white);
    }
  });

  COMPLEX_DRAWING("module:FieldCoverageProvider:worstNoTurnRangeTarget",
  {
    if(fieldCoverage.worstNoTurnRangeTarget.isValid)
    {
      const Vector2<> abs = theRobotPose * fieldCoverage.worstNoTurnRangeTarget.target;
      CROSS("module:FieldCoverageProvider:worstNoTurnRangeTarget",
            abs.x, abs.y, 100, 20, Drawings::ps_solid, ColorClasses::yellow);
    }
  });

  COMPLEX_DRAWING("module:FieldCoverageProvider:worstNoTurnHalfRangeTarget",
  {
    if(fieldCoverage.worstNoTurnHalfRangeTarget.isValid)
    {
      const Vector2<> abs = theRobotPose * fieldCoverage.worstNoTurnHalfRangeTarget.target;
      CROSS("module:FieldCoverageProvider:worstNoTurnHalfRangeTarget",
            abs.x, abs.y, 100, 20, Drawings::ps_solid, ColorRGBA(255, 192, 203)); // Pink
    }
  });

  COMPLEX_DRAWING("module:FieldCoverageProvider:worstNoTurnTargetImage",
  {
    if(fieldCoverage.worstNoTurnTarget.isValid)
    {
      Vector2<int> img;
      Geometry::calculatePointInImage(fieldCoverage.worstNoTurnTarget.target, theCameraMatrix, theCameraInfo, img);
      CROSS("module:FieldCoverageProvider:worstNoTurnTargetImage",
            img.x, img.y, 100, 20, Drawings::ps_solid, ColorClasses::blue);
    }
  });

  // Calculate statistics of the coverage;
  calculateMean(fieldCoverage);
  calculateStdDev(fieldCoverage);
  PLOT("module:FieldCoverageProvider:mean", fieldCoverage.mean);
  PLOT("module:FieldCoverageProvider:stddev", fieldCoverage.stddev);

  // Assemble Team Comm
  if(theTeamMateData.sendThisFrame)
  {
    teamData.nextInterval();
    unsigned intervalOffset = teamData.interval * teamData.intervalSize;
    ASSERT(intervalOffset + teamData.intervalSize <= xSteps * ySteps);
    for(int i = 0; i < teamData.intervalSize; ++i)
      teamData.cells[i] = cells[intervalOffset + i]->coverage(theFrameInfo.time);
    teamData.timestamp = theFrameInfo.time;
    TEAM_OUTPUT(idTeamMateFieldCoverage, bin, teamData);
  }
}

Vector2<> FieldCoverageProvider::projectOnField(int x, int y)
{
  Vector2<> pointOnField;
  Geometry::calculatePointOnField(x, y, theCameraMatrix, theCameraInfo, pointOnField);
  pointOnField = Geometry::relative2FieldCoord(theRobotPose, pointOnField);
  return pointOnField;
}

void FieldCoverageProvider::calculateMean(FieldCoverage& fieldCoverage)
{
  const int size = xSteps * ySteps;
  int sum = 0;
  for(int i = 0; i < size; ++i)
    sum += fieldCoverage.coverage(i, theFrameInfo.time);
  fieldCoverage.mean = static_cast<float>(sum) / static_cast<float>(size);
}

void FieldCoverageProvider::calculateStdDev(FieldCoverage& fieldCoverage)
{
  const int size = xSteps * ySteps;
  fieldCoverage.stddev = 0.0f;
  for(int i = 0; i < size; ++i)
    fieldCoverage.stddev += sqr(fieldCoverage.mean - static_cast<float>(fieldCoverage.coverage(i, theFrameInfo.time)));
  fieldCoverage.stddev /= static_cast<float>(size);
  fieldCoverage.stddev = std::sqrt(fieldCoverage.stddev);
}

bool FieldCoverageProvider::isBallThrownIn()
{
  if(theFrameInfo.getTimeSince(theGameInfo.timeLastPackageReceived) > 1000
     || theGameInfo.state != STATE_PLAYING)
  {
    lastGameState = theGameInfo.state;
    return false;
  }

  int gameInfoLastThrowIn = theFrameInfo.time / 1000 - theGameInfo.dropInTime;
  if(gameInfoLastThrowIn < 0)
    gameInfoLastThrowIn = 0;

  if(lastGameState != STATE_PLAYING)
  {
    lastThrowInTimestamp = gameInfoLastThrowIn;
    lastGameState = STATE_PLAYING;
    return false;
  }

  if(gameInfoLastThrowIn < lastThrowInTimestamp - 2
     || gameInfoLastThrowIn > lastThrowInTimestamp + 2)
  {
    lastThrowInTimestamp = gameInfoLastThrowIn;
    // Don't care about a throw-in if it was more than half a minute ago.
    return theGameInfo.dropInTime < 30;
  }
  return false;
}

void FieldCoverageProvider::ballThrowIn(FieldCoverage& fieldCoverage)
{
  // Set cells which are cut by the throw-in lines to be uncovered if the ball is thrown in.
  calculateBallOutPosition();
  CROSS("module:FieldCoverageProvider:outPosition",
        ballOut.position.x, ballOut.position.y, 75, 30,
        Drawings::ps_solid, ColorRGBA(255, 0, 0));

  fieldCoverage.throwIn = isBallThrownIn();
  DEBUG_RESPONSE("module:FieldCoverageProvider:throwIn", fieldCoverage.throwIn = true;);
  if(fieldCoverage.throwIn)
  {
    const bool ownTeamKicked = theGameInfo.dropInTeam == theOwnTeamInfo.teamColour;

    Vector2<> throwInPosition(lastValidInsideLyingBall.position.x + (ownTeamKicked ? -throwInPenaltyDistance : throwInPenaltyDistance),
                              lastValidInsideLyingBall.position.y < 0 ? yPosRightThrowInLine : yPosLeftThrowInLine);
    if(theFrameInfo.getTimeSince(ballOut.time) < 10 * 1000) // We should know where the ball went out.
    {
      if(ballOut.position.x > theFieldDimensions.xPosOpponentGroundline - 10.0f && ballOut.position.x < theFieldDimensions.xPosOpponentGroundline + 10.0f)
        throwInPosition.x = ownTeamKicked ? std::min(throwInPosition.x, 0.f) : xPosOpponentThrowInPoint;
      else if(ballOut.position.x > theFieldDimensions.xPosOwnGroundline - 10.0f && ballOut.position.x < theFieldDimensions.xPosOwnGroundline + 10.0f)
        throwInPosition.x = ownTeamKicked ? xPosOwnThrowInPoint : std::max(throwInPosition.x, 0.f);
      else if((ballOut.position.y > theFieldDimensions.yPosLeftSideline - 10.0f && ballOut.position.y < theFieldDimensions.yPosLeftSideline + 10.0f)
              || (ballOut.position.y > theFieldDimensions.yPosRightSideline - 10.0f && ballOut.position.y < theFieldDimensions.yPosRightSideline + 10.0f))
        throwInPosition.x = ownTeamKicked ? std::min(throwInPosition.x, ballOut.position.x - throwInPenaltyDistance)
                            : std::max(throwInPosition.x, ballOut.position.x + throwInPenaltyDistance);
      else
        OUTPUT_WARNING("Ball is out but I don't know where.");

      throwInPosition.y = ballOut.position.y < 0.f ? yPosRightThrowInLine : yPosLeftThrowInLine;
    }

    // Clip throw in position to be on the throw in line.
    if(throwInPosition.x > theFieldDimensions.xPosOpponentGroundline - throwInLineDistanceGL)
      throwInPosition.x = theFieldDimensions.xPosOpponentGroundline - throwInLineDistanceGL;
    if(throwInPosition.x < theFieldDimensions.xPosOwnGroundline + throwInLineDistanceGL)
      throwInPosition.x = theFieldDimensions.xPosOwnGroundline + throwInLineDistanceGL;

    CROSS("module:FieldCoverageProvider:throwInPosition",
          throwInPosition.x, throwInPosition.y, 75, 30,
          Drawings::ps_solid, ColorRGBA(255, 192, 203)); // That's pink.

    Vector2<int> throwInCell(static_cast<int>((throwInPosition.x - theFieldDimensions.xPosOwnGroundline) / cellLength.x),
                             static_cast<int>((throwInPosition.y - theFieldDimensions.yPosRightSideline) / cellLength.y));
    CROSS("module:FieldCoverageProvider:throwInCell",
          throwInCell.x * cellLength.x + theFieldDimensions.xPosOwnGroundline,
          throwInCell.y * cellLength.y + theFieldDimensions.yPosRightSideline,
          75, 30, Drawings::ps_solid, ColorRGBA(255, 255, 0));

    const int coverageOtherThrowInLine = (xSteps - throwInCell.x) * throwInCellDiff; // Coverage of the throw-in line which wasn't crossed by the ball.
    const int yOtherThrowInLine = ySteps - 1 - throwInCell.y;
    for(int x = 0; x < static_cast<int>(xSteps); ++x)
    {
      for(int y = 0; y < static_cast<int>(ySteps); ++y)
      {
        int coverage = minThrowInCellCoverage;
        if(y == throwInCell.y)
          coverage = abs(x - throwInCell.x) * throwInCellDiff;
        else if(y == yOtherThrowInLine)
          coverage = coverageOtherThrowInLine;
        fieldCoverageGrid[x][y]->setCoverage(theFrameInfo.time, (unsigned char) coverage);
        fieldCoverage.cells[x * ySteps + y] = fieldCoverageGrid[x][y]->lastseen;
      }
    }
  }
}

void FieldCoverageProvider::computeShadows(std::vector<RobotShadow>& shadows, const RobotPose& robotPose,
    const RobotsModel& robotsModel)
{
  for(RobotsModel::RCIt r = theRobotsModel.robots.begin(); r != theRobotsModel.robots.end(); ++r)
    if(r->standing)
      shadows.push_back(RobotShadow(robotPose, r->relPosOnField));
}

bool FieldCoverageProvider::isCellShadowed(const std::vector<RobotShadow>& shadows, const Cell& cell)
{
  std::vector<RobotShadow>::const_iterator it = shadows.begin();
  const std::vector<RobotShadow>::const_iterator end = shadows.end();
  for(; it != end; ++it)
    if(it->isPointShadowed(cell))
      return true;
  return false;
}

bool FieldCoverageProvider::calculateBallOutPosition()
{
  if(lastValidInsideBall.time > lastValidOutsideBall.time)
    return false; // Calculating the way the ball came back into the field doesn't make sense.
  else if(ballOut.time >= lastValidInsideBall.time)
    return false; // The ball did not move inside the field since the last call.

  // Make line in ball direction
  Geometry::Line ballOutLine(lastValidInsideBall.position,
                             lastValidOutsideBall.position - lastValidInsideBall.position);

  // Intersect line with field border lines and select most plausible intersection.
  const float opponentGl = theFieldDimensions.xPosOpponentGroundline + 10.f;
  const float ownGl = theFieldDimensions.xPosOwnGroundline - 10.f;
  const float leftSl = theFieldDimensions.yPosLeftSideline + 10.f;
  const float rightSl = theFieldDimensions.yPosRightSideline - 10.f;
  float minDistance = -1.f;
  for(std::vector<Geometry::Line>::const_iterator it = fieldBorder.begin(), end = fieldBorder.end();
      it != end; ++it)
  {
    Vector2<> intersection;
    if(!Geometry::getIntersectionOfLines(*it, ballOutLine, intersection))
      continue;
    if(intersection.x < opponentGl && intersection.x > ownGl && intersection.y < leftSl && intersection.y > rightSl)
    {
      const float distance = (lastValidInsideBall.position - intersection).abs();
      if(minDistance < 0.f || distance < minDistance)
      {
        ballOut.position = intersection;
        minDistance = distance;
      }
    }
  }

  if(minDistance >= 0.f)
  {
    ballOut.time = lastValidInsideBall.time;
    return true;
  }
  return false;
}

FieldCoverageProvider::RobotShadow::RobotShadow(const RobotShadow& other)
  : robotPose(other.robotPose), distance(other.distance)
{
  for(int i = 0; i < 4; ++i)
    vertices[i] = other.vertices[i];
}

FieldCoverageProvider::RobotShadow::RobotShadow(const RobotPose& robotPose, const Vector2<>& otherRobotRelativePosition)
  : robotPose(robotPose), distance(otherRobotRelativePosition.abs())
{
  // Calculate shadow vertices.
  const Vector2<> absPosOnField = Geometry::relative2FieldCoord(robotPose, otherRobotRelativePosition);
  Vector2<> directionToRobot = absPosOnField - robotPose.translation;
  const Vector2<> perpendicular = directionToRobot.rotateRight().normalize();

  vertices[0] = absPosOnField - perpendicular * 250.0f;
  vertices[3] = absPosOnField + perpendicular * 250.0f;
  vertices[1] = vertices[0] + (vertices[0] - robotPose.translation).normalize(7000.0f);
  vertices[2] = vertices[3] + (vertices[3] - robotPose.translation).normalize(7000.0f);
}

FieldCoverageProvider::RobotShadow& FieldCoverageProvider::RobotShadow::operator=(const RobotShadow& other)
{
  distance = other.distance;
  for(int i = 0; i < 4; ++i)
    vertices[i] = other.vertices[i];
  return *this;
}

bool FieldCoverageProvider::RobotShadow::isPointShadowed(const Cell& cell) const
{
  if(cell.distance < distance)
    return false;
  return Geometry::isPointInsideConvexPolygon(vertices, 4, cell.absoluteCameraTargetOnField);
}

void FieldCoverageProvider::RobotShadow::draw() const
{
  QUADRANGLE("module:FieldCoverageProvider:shadows",
             vertices[0].x, vertices[0].y,
             vertices[1].x, vertices[1].y,
             vertices[2].x, vertices[2].y,
             vertices[3].x, vertices[3].y,
             20, Drawings::ps_solid, ColorClasses::yellow);
}

void FieldCoverageProvider::RobotShadow::drawOnImage(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo,
                                                     const ImageCoordinateSystem& imageCoordinateSystem) const
{
  COMPLEX_DRAWING("module:FieldCoverageProvider:shadowsOnImage",
  {
    Vector2<> pixel1 = projectOnImage(vertices[0], cameraMatrix, cameraInfo, imageCoordinateSystem);
    Vector2<> pixel2 = projectOnImage(vertices[1], cameraMatrix, cameraInfo, imageCoordinateSystem);
    Vector2<> pixel3 = projectOnImage(vertices[2], cameraMatrix, cameraInfo, imageCoordinateSystem);
    Vector2<> pixel4 = projectOnImage(vertices[3], cameraMatrix, cameraInfo, imageCoordinateSystem);
    QUADRANGLE("module:FieldCoverageProvider:shadowsOnImage",
               pixel1.x, pixel1.y,
               pixel2.x, pixel2.y,
               pixel3.x, pixel3.y,
               pixel4.x, pixel4.y,
               1, Drawings::ps_solid, ColorClasses::red);
  });
}

Vector2<> FieldCoverageProvider::RobotShadow::projectOnImage(const Vector2<>& absPosOnField, const CameraMatrix& cameraMatrix,
    const CameraInfo& cameraInfo, const ImageCoordinateSystem& imageCoordinateSystem) const
{
  Vector2<> relPosOnField = Geometry::fieldCoord2Relative(robotPose, absPosOnField);
  Vector2<int> pointInImage;
  Geometry::calculatePointInImage(relPosOnField, cameraMatrix, cameraInfo, pointInImage);
  return imageCoordinateSystem.fromCorrectedApprox(pointInImage);
}

void FieldCoverageProvider::drawFieldView()
{
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:FieldView", "drawingOnField");

  Vector2<> p[4];
  Vector3<> vectorToCenter(1, 0, 0);

  RotationMatrix r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  Vector3<> vectorToCenterWorld = r * vectorToCenter;

  float a1 = theCameraMatrix.translation.x,
  a2 = theCameraMatrix.translation.y,
  a3 = theCameraMatrix.translation.z ,
  b1 = vectorToCenterWorld.x,
  b2 = vectorToCenterWorld.y,
  b3 = vectorToCenterWorld.z,
  f = a3 / b3;
  Vector2<> pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[0] = theRobotPose.translation;
  else
    p[0] = (theRobotPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(theCameraInfo.openingAngleHeight / 2);
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[1] = theRobotPose.translation;
  else
    p[1] = (theRobotPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(-(theCameraInfo.openingAngleWidth / 2));
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[2] = theRobotPose.translation + Vector2<>(12000, 0).rotate(theRobotPose.rotation + (-theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    p[2] = (theRobotPose + pof).translation;

  r = theCameraMatrix.rotation;
  r.rotateY(-(theCameraInfo.openingAngleHeight / 2));
  r.rotateZ(theCameraInfo.openingAngleWidth / 2);
  vectorToCenterWorld = r * vectorToCenter;

  b1 = vectorToCenterWorld.x;
  b2 = vectorToCenterWorld.y;
  b3 = vectorToCenterWorld.z;
  f = a3 / b3;
  pof = Vector2<>(a1 - f * b1, a2 - f * b2);

  if(f > 0.f)
    p[3] = theRobotPose.translation + Vector2<>(12000, 0).rotate(theRobotPose.rotation + (theCameraInfo.openingAngleWidth / 2) + theCameraMatrix.rotation.getZAngle());
  else
    p[3] = (theRobotPose + pof).translation;

  POLYGON("module:FieldCoverageProvider:FieldView", 4, p, 20, Drawings::ps_null, ColorClasses::none, Drawings::bs_solid, ColorRGBA(255, 255, 255, 25));
}
