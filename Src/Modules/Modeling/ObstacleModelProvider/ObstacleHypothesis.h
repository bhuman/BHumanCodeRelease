/**
 * @file ObstacleHypothesis.h
 *
 * Declaration of a class that represents a possible obstacle.
 *
 * @author Florian Maaß
 * @author Jan Fiedler
 * @author Nicole Schrader
 */
#pragma once

#include "Tools/Modeling/Obstacle.h"
#include "Math/RingBuffer.h"

struct CameraInfo;
struct CameraMatrix;
struct FieldBoundary;

/**
 * @class ObstacleHypothesis
 *
 * Represents a possible obstacle with some functions.
 */
class ObstacleHypothesis : public Obstacle
{
  friend class ObstacleModelProvider; // Only the obstacle model provider can create instances.

  int team = 0;                            /**< Negative value is for teammates. */
  int upright = 0;                         /**< Negative value is for fallen robots. */
  unsigned seenCount = 0u;                 /**< Somewhat validity (between minPercepts and maxPercepts in ObstacleModelProvider). */
  unsigned notSeenButShouldSeenCount = 0u; /**< How many times the obstacle was not seen but could be measured. */

  /** The constructor. */
  ObstacleHypothesis(const Type type);
  /**
   * The constructor.
   * @param The Obstacle.
   * @param seenCount The amount of percepts.
   */
  ObstacleHypothesis(const Matrix2f& covariance,
                     const Vector2f& center,
                     const Vector2f& left,
                     const Vector2f& right,
                     const unsigned lastSeen,
                     const Type type,
                     const unsigned seenCount);

  /** Is the left and the right vector between the two given parameters left and right angle (radian)? */
  bool isBetween(const float leftAngle, const float rightAngle) const
  {
    return leftAngle > left.angle() && right.angle() > rightAngle;
  }

  /**
   * Whether this obstacle is behind the other one.
   * @param other The other obstacle.
   */
  bool isBehind(const ObstacleHypothesis& other) const;

  /**
   * Whether an obstacle is in the image.
   * @param centerInImage The center of the obstacle in Image coordinates.
   */
  bool isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const;

  /** EKF dynamic step. */
  void dynamic(const float odometryRotation, const Vector2f& odometryTranslation, const Matrix2f& odometryJacobian,
               const float odometryNoiseX, const float odometryNoiseY);
  /**
   * EKF measurement step.
   * @param weightedSum Factor how much the measured width affects the actual width.
   * @param minWidth The minimum width of an obstacle.
   */
  void measurement(const ObstacleHypothesis& measurement, const float weightedSum, const float minWidth);

  /** Trying to identify the type of the obstacle. Sets the team. upright, and type attributes.
   * @param measurement A new measurement of an obstacle
   * @param teamThreshold Fiddle parameter configured in module
   * @param uprightThreshold Fiddle parameter configured in module
   */
  void determineAndSetType(const ObstacleHypothesis& measurement, const int teamThreshold, const int uprightThreshold);

  /**
   * Whether the field boundary is further as this obstacle.
   */
  bool isFieldBoundaryFurtherAsObstacle(const CameraInfo& theCameraInfo,
                                        const CameraMatrix& theCameraMatrix,
                                        const FieldBoundary& theFieldBoundary);

  /** Calculates the squared Mahalanobis distances to the other obstacle. */
  float squaredMahalanobis(const ObstacleHypothesis& other) const;
};
