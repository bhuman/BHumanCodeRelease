/**
* @file ObstacleModel.h
*
* Declaration of class ObstacleModel
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#pragma once

#include "Tools/Math/Vector2.h"
#include "Tools/Math/Matrix2x2.h"
#include "Tools/Math/Random.h"
#include "Tools/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Platform/BHAssert.h"
#include <vector>

class Pose3D;

/**
* @class ObstacleModel
*
* A class that represents the current state of the robot's environment
*/
STREAMABLE(ObstacleModel,
{
public:
  /** A single obstacle */
  STREAMABLE(Obstacle,
  {
  public:
    ENUM(Type, US, ROBOT, ARM, FOOT); /**< Different obstacle type */

    /** Constructor */
    Obstacle(const Vector2<>& leftCorner, const Vector2<>& rightCorner,
             const Vector2<>& center, const Vector2<>& closestPoint, const Matrix2x2<>& covariance, Type type = US),

    (Vector2<>) leftCorner,      /**< Leftmost point of the obstacle */
    (Vector2<>) rightCorner,     /**< Rightmost point of the obstacle */
    (Vector2<>) center,          /**< Center of mass of obstacle */
    (Vector2<>) closestPoint,    /**< Point of obstacle that is closest to the robot */
    (Matrix2x2<>) covariance,
    (Type)(US) type,             /**< The type of an obstacle */
  });

  /** Function for drawing */
  void draw() const;

  void draw3D(const Pose3D& torsoMatrix) const,

  /** A list of obstacles */
  (std::vector<Obstacle>) obstacles,
});

class USObstacleModel : public ObstacleModel {};

/**
 * A compressed version of the obstacleModel.
 */
STREAMABLE(ObstacleModelCompressed,
{
public:
  /**
   * private obstacle with compressed streaming.
   */
  STREAMABLE(Obstacle,
  {
  public:
    Obstacle(const ObstacleModel::Obstacle& other),

    (Vector2<short>) leftCorner,   /**< Leftmost point of the obstacle */
    (Vector2<short>) rightCorner,  /**< Rightmost point of the obstacle */
    (Vector2<short>) center,       /**< Center of mass of obstacle */
    (Vector2<short>) closestPoint, /**< Point of obstacle that is closest to the robot */

    /**
    * The covariance is a 2x2 matrix. however x12 and x21 are always identical.
    * Therefore we only store 3 floats instead of 4: x11, x12 and x22
    * |x11  x12|
    * |        |
    * |x21  x22|
    */
    (float) x11,
    (float) x12,
    (float) x22,

    (ObstacleModel::Obstacle, Type)(US) type, /**< The type of an obstacle */
  });

  ObstacleModelCompressed(const ObstacleModel& other, unsigned int maxNumberOfObstacles);

  operator ObstacleModel () const,

  (std::vector<Obstacle>) obstacles,
});

