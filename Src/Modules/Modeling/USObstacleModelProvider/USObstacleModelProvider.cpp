/**
 * This file implements a module that determines the position of obstacles using sonar
 * measurements. The measurements are entered into a grid that keeps a limited history
 * of previous measurements per cell. The module clusters all cells that have accumulated
 * enough evidence for the presence of an obstacle. It estimates the positions of the
 * obstacles from these clusters.
 * @author Thomas Röfer
 * @author Christian Mandel
 * @author Tim Laue
 */

#include "USObstacleModelProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Approx.h"
#include <algorithm>

static const EvidenceGrid::Cell initCell = {0, 0, 0}; // For VS2012

USObstacleModelProvider::USObstacleModelProvider()
: gridSize(2.f * (maxDistance + obstacleThickness +
                  std::max(std::max(std::max(leftToLeft.translation.abs(), leftToRight.translation.abs()),
                                    rightToLeft.translation.abs()), rightToRight.translation.abs()))),
  grid(gridSize, gridSizeInCells, gridSizeInCells, initCell),
  lastUSTimeStamp(0), lastDecay(0), lastTimePenalized(0),
  lastGameState(STATE_INITIAL), lastPenaltyState(PENALTY_NONE)
{
  prealloc();
}

void USObstacleModelProvider::prealloc()
{
  points.reserve(grid.widthInCells); // should be enough
  inter.reserve(grid.widthInCells); // 2 might also be enough
  cellStack.reserve(grid.widthInCells * grid.heightInCells);
}

inline void USObstacleModelProvider::updatePolygon(const std::function<void(EvidenceGrid::Cell*)>& fn)
{
  // generate boundary of polygon
  std::list<Line> lines;
  int j;
  bool lastPeak = false;
  for(j = 1; j < (int) points.size() - 1; ++j)
  {
    bool peak = points[j - 1].y < points[j].y && points[j + 1].y < points[j].y;
    lines.push_back(Line(points[j - 1], points[j], peak || lastPeak));
    lastPeak = peak;
  }
  lines.push_back(Line(points[j - 1], points[j], lastPeak));

  // sort the lines by their y coordinates
  lines.sort();

  // run through lines in increasing y order
  for(int y = (int) lines.front().a.y; !lines.empty(); ++y)
  {
    inter.clear();
    for(std::list<Line>::iterator it = lines.begin(); it != lines.end() && it->a.y <= y;)
      if(it->b.y > y || (it->peak && it->b.y == y))
      {
        // line is not finished yet
        inter.push_back((int) it->a.x);
        it->a.x += it->ils;
        ++it;
      }
      else if(it->b.y == y && it->a.y == y && y != 0)
      {
        // horizontal line, is not drawn when y == 0, because overlapping lines clutter the map
        inter.push_back((int) it->a.x);
        inter.push_back((int) it->b.x);
        ++it;
      }
      else
        // line is finished -> remove it
        it = lines.erase(it);

    if(!inter.empty())
    {
      // fill the line on an even/odd basis
      bool paint = false;
      int goalX = -1;
      std::sort(inter.begin(), inter.end());
      std::vector<int>::iterator iIt = inter.begin();
      for(;;)
      {
        int startX = *iIt;
        if(++iIt == inter.end())
          break;
        paint ^= true;
        if(paint)
        {
          if(startX == goalX)
            ++startX;
          goalX = *iIt;
          for(EvidenceGrid::Cell* cell = &grid[y][startX],
              * endCell = cell + goalX - startX; cell <= endCell; ++cell)
            fn(cell);
        }
      }
    }
  }
}

void USObstacleModelProvider::update(USObstacleModel& usObstacleModel)
{
  if(theCameraInfo.camera == CameraInfo::upper) // skip sonar for upper camera
    return;

  DEBUG_RESPONSE_ONCE("module:USObstacleModelProvider:clear", grid.clear(););

  gridSize = 2.f * (maxDistance + obstacleThickness +
                    std::max(std::max(std::max(leftToLeft.translation.abs(), leftToRight.translation.abs()),
                                      rightToLeft.translation.abs()), rightToRight.translation.abs()));
  
  // Check whether the grid size was MODIFYed
  if(gridSize != grid.width || gridSizeInCells != grid.widthInCells)
  {
    grid = EvidenceGrid(gridSize, gridSizeInCells, gridSizeInCells, initCell);
    prealloc();
  }

  // Clear grid when penalized.
  // Note that until a certain time after being unpenalized, the grid will
  // stay empty, because no measurements are entered.
  if(lastPenaltyState == PENALTY_NONE && theRobotInfo.penalty != PENALTY_NONE)
    grid.clear();
  else // otherwise center the grid around the current odometry position
    grid.move(theOdometryData.translation);

  if(theRobotInfo.penalty != PENALTY_NONE)
    lastTimePenalized = theFrameInfo.time;

  // Enter new measurements if there are any, unless we are penalized or have
  // been shortly or want to execute or are executing a special action.
  if(theFilteredSensorData.usTimeStamp != lastUSTimeStamp &&
     theFrameInfo.getTimeSince(lastTimePenalized) >= ignoreAfterPenaltyDelay &&
     theMotionRequest.motion != MotionRequest::specialAction &&
     theMotionInfo.motion != MotionRequest::specialAction)
  {
    // Define functions for updating the grid cells

    // Lower evidence of an obstacle
    auto freeCell = [=](EvidenceGrid::Cell* c) -> void
    {
      c->count -= (c->history >> (historySize - 1)) & 1;
      c->history <<= 1;
    };

    // Increase evidence of an obstacle
    auto occupyCell = [=](EvidenceGrid::Cell* c) -> void
    {
      c->count += (c->history & 1) ^ 1;
      c->history |= 1;
    };

    // If this cell is consided occupied, strengthen that belief,
    // otherwise lower the evidence of an obstacle
    auto supportCell = [=](EvidenceGrid::Cell* c) -> void
    {
      if(c->count >= occupiedThreshold)
      {
        c->count += ((c->history >> (historySize - 1)) & 1) ^ 1;
        c->history <<= 1;
        c->history |= 1;
      }
      else
        freeCell(c);
    };

    // Make the grid forget old measurments after a while
    if(theFrameInfo.getTimeSince(lastDecay) >= decayDelay)
    {
      lastDecay = theFrameInfo.time;
      for(int y = 0; y < grid.heightInCells; ++y)
        for(EvidenceGrid::Cell* c = grid[y], *cEnd = c + grid.widthInCells; c < cEnd; ++c)
          freeCell(c);
    }

    // Pick the two sensors configurations that were used and enter the measurements to the grid
    bool same = theFilteredSensorData.usActuatorMode == SensorData::bothToSame;
    const SonarInfo* infos[2] = {same ? &leftToLeft : &rightToLeft, same ? &rightToRight : &leftToRight};
    const float* distances[2] = {&theFilteredSensorData.data[SensorData::usL], &theFilteredSensorData.data[SensorData::usR]};
    for(int i = 0; i < 2; ++i)
    {
      float freeDistance = std::min(distances[i][0], maxDistance) + obstacleThickness;

      // If there is a second measurement, free grid until this distance or the maximum distance,
      // because the area between the first obstacle and the second should be empty.
      if(freeDistance < maxDistance && distances[i][1] < 2550.f && distances[i][1] > freeDistance)
        freeDistance = std::min(distances[i][1], maxDistance);

      // If the measurement is small, strengthen all obstacles present until that
      // distance and do a normal update for everything beyond
      if(distances[i][0] <= minDistance)
      {
        setupDiskSegment(*infos[i], distances[i][0] - grid.cellSize);
        updatePolygon(supportCell);
        setupRingSegment(*infos[i], distances[i][0], freeDistance);
        updatePolygon(freeCell);
      }
      else
      { // Otherwise lower the evidence in the whole area
        setupDiskSegment(*infos[i], freeDistance);
        updatePolygon(freeCell);
      }

      // If an obstalce was detected within the measurin range, enter it
      if(distances[i][0] < maxDistance)
      {
        setupRingSegment(*infos[i], distances[i][0], distances[i][0] + obstacleThickness);
        updatePolygon(occupyCell);
      }
    }
  }

  // Cluster the obstacles and fill the model
  clusterObstacles(usObstacleModel);

  draw();

  lastUSTimeStamp = theFilteredSensorData.usTimeStamp;
  lastGameState = theGameInfo.state;
  lastPenaltyState = theRobotInfo.penalty;
}

void USObstacleModelProvider::setupDiskSegment(const SonarInfo& info, float distance)
{
  int numOfPoints = std::max(2, std::min(int(distance * pi2 / maxDistanceStep), int(info.openingAngle / fromDegrees(maxAngleStep))));
  float angleStep = info.openingAngle / (numOfPoints - 1);
  Pose2D sensorPose = theOdometryData + info + Pose2D(-info.openingAngle / 2.f);

  points.clear();
  Vector2<int> sensorPos = grid.worldToGrid(sensorPose.translation);
  points.push_back(sensorPos);
  for(int j = 0; j < numOfPoints; ++j)
  {
    Vector2<int> point = grid.worldToGrid(sensorPose * Vector2<>(distance, 0));
    clipPointP2(sensorPos, point);
    points.push_back(point);
    sensorPose.rotation += angleStep;
  }
  points.push_back(sensorPos);
}

void USObstacleModelProvider::setupRingSegment(const SonarInfo& info, float minDistance, float maxDistance)
{
  int numOfPoints = std::max(2, std::min(int(maxDistance * pi2 / maxDistanceStep), int(info.openingAngle / fromDegrees(maxAngleStep))));
  float angleStep = info.openingAngle / (numOfPoints - 1);
  Pose2D sensorPose = theOdometryData + info + Pose2D(-info.openingAngle / 2.f);

  points.resize(numOfPoints * 2 + 1);
  Vector2<int> sensorPos = grid.worldToGrid(sensorPose.translation);
  for(int j = 0; j < numOfPoints; ++j)
  {
    Vector2<int> obstacleFront = grid.worldToGrid(sensorPose * Vector2<>(minDistance, 0));
    Vector2<int> obstacleBack = grid.worldToGrid(sensorPose * Vector2<>(maxDistance, 0));
    clipPointP2(sensorPos, obstacleFront);
    clipPointP2(sensorPos, obstacleBack);
    points[j] = obstacleBack;
    points[points.size() - j - 2] = obstacleFront;
    sensorPose.rotation += angleStep;
  }
  points[points.size() - 1] = points[0];
}

void USObstacleModelProvider::clipPointP2(const Vector2<int>& p1, Vector2<int>& p2) const
{
  if(p2.x < 0)
  {
    p2.y = p1.y + (p2.y - p1.y) * -p1.x / (p2.x - p1.x);
    p2.x = 0;
  }
  else if(p2.x >= grid.widthInCells)
  {
    p2.y = p1.y + (p2.y - p1.y) * (grid.widthInCells - 1 - p1.x) / (p2.x - p1.x);
    p2.x = grid.widthInCells - 1;
  }

  if(p2.y < 0)
  {
    p2.x = p1.x + (p2.x - p1.x) * -p1.y / (p2.y - p1.y);
    p2.y = 0;
  }
  else if(p2.y >= grid.heightInCells)
  {
    p2.x = p1.x + (p2.x - p1.x) * (grid.heightInCells - 1 - p1.y) / (p2.y - p1.y);
    p2.y = grid.heightInCells - 1;
  }
}

void USObstacleModelProvider::clusterObstacles(ObstacleModel& obstacleModel)
{
  DECLARE_DEBUG_DRAWING("module:USObstacleModelProvider:covariance", "drawingOnField");

  // Mark all cells as unclustered
  for(int y = 0; y < grid.heightInCells; ++y)
    for(EvidenceGrid::Cell* c = grid[y], *cEnd = c + grid.widthInCells; c < cEnd; ++c)
      c->clustered = 0;

  // Cluster cell occupied and fill the model
  obstacleModel.obstacles.clear();
  for(int y = 0; y < grid.heightInCells; ++y)
    for(EvidenceGrid::Cell* c = grid[y], *cEnd = c + grid.widthInCells; c < cEnd; ++c)
      if(!c->clustered && c->count >= occupiedThreshold)
      {
        c->clustered = 1;
        clusterObstacle(Vector2<int>(c - grid[y], y), obstacleModel);
      }
}

void USObstacleModelProvider::clusterObstacle(const Vector2<int>& start, ObstacleModel& obstacleModel)
{
  float distanceSum = 0.f;
  float directionSum = 0.f;
  float distanceSum2 = 0.f; // sum of squares
  float directionSum2 = 0.f; // sum of squares
  float divisor = 0.f;
  int count = 0;
  float minDirection = std::numeric_limits<float>::max();
  float maxDirection = -std::numeric_limits<float>::max();
  float minDistance = std::numeric_limits<float>::max();
  const float cosine = std::cos(theOdometryData.rotation);
  const float sine = std::sin(theOdometryData.rotation);
  const Matrix2x2<> rotation(cosine, sine, -sine, cosine);
  const Vector2<> offset = (Pose2D(grid.gridToWorld(grid.worldToGrid(theOdometryData.translation))) - theOdometryData).translation;
  ObstacleModel::Obstacle obstacle;

  cellStack.push_back(start);
  while(!cellStack.empty())
  {
    Vector2<int> coords = cellStack.back();
    cellStack.pop_back();
    const float weight = (float) grid[coords.y][coords.x].count;

    // Calculate distance and direction relative to robot
    const Vector2<> cellInWorld((coords.x - float(grid.widthInCells >> 1)) * grid.cellSize,
                                (coords.y - float(grid.heightInCells >> 1)) * grid.cellSize);
    const Vector2<> pos = offset + rotation * cellInWorld;
    const float sqrDistance = pos.squareAbs();
    const float distance = std::sqrt(sqrDistance);
    float direction = approxAtan2(pos.y, pos.x);

    if(count)
    {
      // Keep range of obstacles behind back connected
      const float center = (maxDirection + minDirection) / 2.0f;
      direction = center + normalize(direction - center);
    }

    distanceSum += weight * distance;
    directionSum += weight * direction;
    distanceSum2 += weight * sqrDistance;
    directionSum2 += weight * sqr(direction);
    divisor += weight;
    ++count;

    if(distance < minDistance)
    {
      minDistance = distance;
      obstacle.closestPoint = pos;
    }
    if(direction > maxDirection)
    {
      maxDirection = direction;
      obstacle.leftCorner = pos;
    }
    if(direction < minDirection)
    {
      minDirection = direction;
      obstacle.rightCorner = pos;
    }

    // enumerate neighbors
    for(int y = std::max(coords.y - 1, 0); y <= std::min(coords.y + 1, grid.heightInCells - 1); ++y)
      for(int x = std::max(coords.x - 1, 0); x <= std::min(coords.x + 1, grid.widthInCells - 1); ++x)
      {
        EvidenceGrid::Cell& neighbor = grid[y][x];
        if(!neighbor.clustered && neighbor.count >= occupiedThreshold)
        {
          neighbor.clustered = 1;
          cellStack.push_back(Vector2<int>(x, y));
        }
      }
  }

  if(count >= minObstacleCells)
  {
    const float avgDistance = distanceSum / divisor;
    const float avgDirection = directionSum / divisor;
    obstacle.center = Pose2D(avgDirection) * Vector2<>(avgDistance, 0.f);

    // Choose closestPoint in the direction of the center because everything else is nonsense
    const float closest = obstacle.closestPoint.abs();
    obstacle.closestPoint = obstacle.center;
    obstacle.closestPoint.normalize(closest);

    // Hack: move the center half a robot thickness behind the closest point
    // This might be bad for team communication, but good for the local
    // combined world model, because obstacle avoidance works much better.
    obstacle.center.normalize(obstacle.closestPoint.abs() + obstacleThickness / 2.f);

    // Calculate covariance: Technically, this is wrong, because this covariance is on an arc,
    // but it should be in 2D Euclidean space.
    const float stdDevDistance = distanceSum2 / divisor - sqr(avgDistance);
    const float stdDevDirection = directionSum2 / divisor - sqr(avgDirection);
    obstacle.covariance = Matrix2x2<>(stdDevDistance, 0.f, 0.f, stdDevDirection * sqr(avgDistance));
    rotateMatrix(obstacle.covariance, obstacle.center.angle());
    COVARIANCE2D("module:USObstacleModelProvider:covariance", obstacle.covariance, obstacle.center);

    obstacle.type = ObstacleModel::Obstacle::US;
    obstacleModel.obstacles.push_back(obstacle);
  }
}

void USObstacleModelProvider::rotateMatrix(Matrix2x2<>& matrix, const float angle)
{
  const float cosine = std::cos(angle);
  const float sine = std::sin(angle);
  const Matrix2x2<> rotationMatrix(cosine, -sine, sine, cosine);
  matrix = (rotationMatrix * matrix) * rotationMatrix.transpose();
}

void USObstacleModelProvider::draw() const
{
  DECLARE_DEBUG_DRAWING("module:USObstacleModelProvider:us", "drawingOnField",
  {
    bool same = theFilteredSensorData.usActuatorMode == SensorData::bothToSame;
    const SonarInfo* infos[2] = {same ? &leftToLeft : &rightToLeft, same ? &rightToRight : &leftToRight};
    const float distances[2] = {theFilteredSensorData.data[SensorData::usL], theFilteredSensorData.data[SensorData::usR]};
    for(int i  = 0; i < 2; ++i)
    {
      int numOfPoints = std::max(2, std::min(int(distances[i] * pi2 / maxDistanceStep), int(infos[i]->openingAngle / fromDegrees(maxAngleStep))));
      float angleStep = infos[i]->openingAngle / (numOfPoints - 1);
      Pose2D sensorPose = *infos[i] + Pose2D(-infos[i]->openingAngle / 2.f);

      std::vector<Vector2<> > points;
      points.push_back(sensorPose.translation);
      for(int j = 0; j < numOfPoints; ++j)
      {
        points.push_back(sensorPose * Vector2<>(distances[i], 0));
        sensorPose.rotation += angleStep;
      }
      POLYGON("module:USObstacleModelProvider:us", (int) points.size(), points, 0, Drawings::ps_null, ColorRGBA(), Drawings::bs_solid, ColorRGBA(255,255,255,64));
      if(distances[i] < maxDistance)
        for(unsigned j = 2; j < points.size(); ++j)
          LINE("module:USObstacleModelProvider:us", points[j - 1].x, points[j - 1].y, points[j].x, points[j].y, 1, Drawings::ps_solid, ColorRGBA(0, 0, 0));
    }
  });

  DECLARE_DEBUG_DRAWING("module:USObstacleModelProvider:grid", "drawingOnField",
  {
    ColorRGBA baseColor(200, 200, 255, 128);
    std::vector<unsigned char> cells;
    cells.reserve(grid.widthInCells * grid.heightInCells);
    for(int y = 0; y < grid.heightInCells; ++y)
      for(const EvidenceGrid::Cell* c = grid[y], *cEnd = c + grid.widthInCells; c < cEnd; ++c)
        cells.push_back((unsigned char) (c->count * 255 / (historySize - 1)));
    GRID_MONO("module:USObstacleModelProvider:grid", grid.center.x, grid.center.y, grid.cellSize, grid.widthInCells, grid.heightInCells, baseColor, cells.data());
    RECTANGLE("module:USObstacleModelProvider:grid", grid.center.x - grid.width / 2, grid.center.y - grid.height / 2,
              grid.center.x + grid.width / 2, grid.center.y + grid.height / 2,
              20, Drawings::ps_solid, ColorRGBA(0, 0, 100));
  });

  DECLARE_DEBUG_DRAWING("module:USObstacleModelProvider:occupied", "drawingOnField",
  {
    ColorRGBA baseColor(255, 0, 0, 128);
    std::vector<unsigned char> cells;
    cells.reserve(grid.widthInCells * grid.heightInCells);
    for(int y = 0; y < grid.heightInCells; ++y)
      for(const EvidenceGrid::Cell* c = grid[y], *cEnd = c + grid.widthInCells; c < cEnd; ++c)
        cells.push_back((unsigned char) (c->count >= occupiedThreshold ? 0 : 255));
    GRID_MONO("module:USObstacleModelProvider:occupied", grid.center.x, grid.center.y, grid.cellSize, grid.widthInCells, grid.heightInCells, baseColor, cells.data());
  });
}

MAKE_MODULE(USObstacleModelProvider, Modeling)
