/**
 * This file declares a module that determines the position of obstacles using sonar
 * measurements. The measurements are entered into a grid that keeps a limited history
 * of previous measurements per cell. The module clusters all cells that have accumulated
 * enough evidence for the presence of an obstacle. It estimates the positions of the
 * obstacles from these clusters.
 * @author Thomas Röfer
 * @author Christian Mandel
 * @author Tim Laue
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "EvidenceGrid.h"

#include <functional>

/**
 * The class represents information about a single sonar sensor,
 * i.e. its 2D pose in the robot's torso and its opening angle.
 */
class SonarInfo : public Pose2D
{
private:
  /**
   * The method makes the object streamable. In the streams,
   * angles are represented in degree, while in this object
   * they are represented in radian.
   * @param in The stream from which the object is read
   * @param out The stream to which the object is written
   */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    float rotation = toDegrees(this->rotation);
    float openingAngle = toDegrees(this->openingAngle);
    STREAM(rotation);
    STREAM(translation);
    STREAM(openingAngle);
    if(in)
    {
      this->rotation = fromDegrees(rotation);
      this->openingAngle = fromDegrees(openingAngle);
    }
    STREAM_REGISTER_FINISH;
  }

public:
  float openingAngle; /**< The opening angle in radian. */
};

MODULE(USObstacleModelProvider)
  REQUIRES(OdometryData)
  REQUIRES(FrameInfo)
  REQUIRES(FilteredSensorData)
  REQUIRES(GameInfo)
  REQUIRES(RobotInfo)
  USES(MotionRequest)
  REQUIRES(MotionInfo)
  REQUIRES(CameraInfo)
  PROVIDES_WITH_DRAW(USObstacleModel)
  LOADS_PARAMETER(SonarInfo, leftToLeft)
  LOADS_PARAMETER(SonarInfo, rightToRight)
  LOADS_PARAMETER(SonarInfo, leftToRight)
  LOADS_PARAMETER(SonarInfo, rightToLeft)
  LOADS_PARAMETER(float, maxDistance)
  LOADS_PARAMETER(float, minDistance)
  LOADS_PARAMETER(int, gridSizeInCells)
  LOADS_PARAMETER(unsigned char, occupiedThreshold)
  LOADS_PARAMETER(float, maxAngleStep)
  LOADS_PARAMETER(int, maxDistanceStep)
  LOADS_PARAMETER(float, obstacleThickness)
  LOADS_PARAMETER(int, decayDelay)
  LOADS_PARAMETER(int, historySize)
  LOADS_PARAMETER(int, minObstacleCells)
  LOADS_PARAMETER(int, ignoreAfterPenaltyDelay);
END_MODULE

class USObstacleModelProvider: public USObstacleModelProviderBase
{
private:
  /**
   * @class Line
   * This class represents a single line segment of a polyline.
   */
  class Line
  {
  public:
    /**
     * Constructor.
     * @param ai The first point of the line segment.
     * @param bi The last point of the line segment.
     * @param peak In one of the two points a peak?
     */
    Line(const Vector2<int>& ai, const Vector2<int>& bi, bool peak)
    : peak(peak)
    {
      if(ai.y <= bi.y)
      {
        a = Vector2<>(ai);
        b = Vector2<>(bi);
      }
      else
      {
        b = Vector2<>(ai);
        a = Vector2<>(bi);
      }
      ils = b.y != a.y ? (b.x - a.x) / (b.y - a.y) : 0;
    }

    /**
     * Operator that compares a given line with this line.
     * @param other This line is compared to other.
     * @return Is the y coordinate of this line smaller than the one of the other line?
     */
    bool operator<(const Line& other) const
    {
      return (a.y < other.a.y);
    }

    Vector2<> a; /**< First point of this line. */
    Vector2<> b; /**< Last point of this line. */
    float ils; /**< Inverse slope of this line. */
    bool peak; /**< Is this line part of an upper peak? */
  };

  float gridSize; /**< The width and height of the grid in mm. */
  EvidenceGrid grid; /**< The grid containing the measurement histories. */
  unsigned lastUSTimeStamp; /**< Timestamp of the last sonar measurement processed. */
  unsigned lastDecay; /**< Last time when all cells were updated with an empty measurement. */
  unsigned lastTimePenalized; /**< Last time the robot was penalized. */
  int lastGameState; /**< The game state in the previous frame. */
  int lastPenaltyState; /**< The penalty state in the previous frame. */
  std::vector<Vector2<int>> points; /**< Polygon vertices for filling (prevents reallocation). */
  std::vector<int> inter; /**< The intersections per line (prevents reallocation). */
  std::vector<Vector2<int>> cellStack; /**< A stack for the floodfill algorithm (prevents reallocation). */

  /**
   * The only provider method of this module.
   */
  void update(USObstacleModel& usObstacleModel);

  /**
   * Reserves memory of suffient size for central data structures.
   */
  void prealloc();

  /**
   * Sets up a polygon in 'points' that has the shape of a disk segment.
   * @param info The center of the disk and the opening angle of the segment.
   * @param distance The radius of the disk.
   */
  void setupDiskSegment(const SonarInfo& info, float distance);

  /**
   * Sets up a polygon in 'points' that has the shape of a ring segment.
   * @param info The center of the ring and the opening angle of the segment.
   * @param minDistance The inner radius of the ring.
   * @param maxDistance The outer radius of the ring.
   */
  void setupRingSegment(const SonarInfo& info, float minDistance, float maxDistance);

  /**
   * Updates all cells inside the polygon described in 'points'.
   * @param fn The kind of update.
   */
  void updatePolygon(const std::function<void(EvidenceGrid::Cell*)>& fn);

  /**
   * Clips the endpoint of a line to the boundary of the grid.
   * @param p1 This point must be inside the grid and will not be clipped.
   * @param p2 This point will be clipped to the boundary of the grid.
   */
  void clipPointP2(const Vector2<int>& p1, Vector2<int>& p2) const;

  /**
   * Cluster all obstacles in the grid and add them to the model.
   * @param obstacleModel The model the obstacles will be added to.
   */
  void clusterObstacles(ObstacleModel& obstacleModel);

  /**
   * Cluster a single obstacle starting at a single grid cell.
   * If it is big enough, it will be added to the model.
   * @param start The coordinates of the starting cell. Its count must be at
   *              least 'occupiedThreshold' and it must be marked as 'clustered'.
   * @param obstacleModel The model the obstacles might be added to.
   */
  void clusterObstacle(const Vector2<int>& start, ObstacleModel& obstacleModel);

  /**
   * Rotate the covariance by a given angle.
   * @param matrix A matrix representing the covariance.
   * @param angle The angle it is rotated by.
   */
  static void rotateMatrix(Matrix2x2<>& matrix, const float angle);

  /**
   * Draw the sonar measurments and the grid.
   */
  void draw() const;

public:
  /**
   * Default constructor. Initializes the module with the values from the configuration file.
   */
  USObstacleModelProvider();
};
