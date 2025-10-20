/**
 * @file GlobalOpponentsHypothesis.h
 *
 * Declaration of a class that represents a possible obstacle.
 *
 * @author Florian Maaß
 * @author Jan Fiedler & Nicole Schrader
 * @author Michelle Gusev
 */
#pragma once

#include "Tools/Modeling/Obstacle.h"
#include "Math/RingBuffer.h"

struct CameraInfo;
struct CameraMatrix;
struct FieldBoundary;
struct ImageCoordinateSystem;

/**
 * @class GlobalOpponentsHypothesis
 *
 * Represents a possible obstacle with some functions.
 */
class GlobalOpponentsHypothesis : public Obstacle
{
  friend class GlobalOpponentsTracker;

  int team = 0;                            /**< Negative value is for teammates. */
  int upright = 0;                         /**< Negative value is for fallen robots. */
  unsigned seenCount = 0u;                 /**< Somewhat validity (between minPercepts and maxPercepts in ObstacleModelProvider). */
  unsigned notSeenButShouldSeenCount = 0u; /**< How many times the obstacle was not seen but could be measured. */

  /** The constructor. */
  GlobalOpponentsHypothesis(const Type type);
  /**
   * The constructor.
   * @param The Obstacle.
   * @param seenCount The amount of percepts.
   */
  GlobalOpponentsHypothesis(const Matrix2f& covariance,
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
  bool isBehind(const GlobalOpponentsHypothesis& other) const;

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
   * @param modelWidthWeighting Factor how much the measured width affects the actual width.
   * @param minWidth The minimum width of an obstacle.
   */
  void measurement(const GlobalOpponentsHypothesis& measurement, const float modelWidthWeighting, const float minWidth);

  /** Trying to identify the type of the hypothesis. Sets the team. upright, and type attributes.
   * @param measurement A new measurement of an opponent (or something else)
   * @param teamThreshold Fiddle parameter configured in module
   * @param uprightThreshold Fiddle parameter configured in module
   */
  void determineAndSetType(const GlobalOpponentsHypothesis& measurement, const int teamThreshold, const int uprightThreshold);

  /**
   * Whether the field boundary is further as this obstacle.
   */
  bool isFieldBoundaryFurtherAsObstacle(const CameraInfo& theCameraInfo,
                                        const CameraMatrix& theCameraMatrix,
                                        const ImageCoordinateSystem& theImageCoordinateSystem,
                                        const FieldBoundary& theFieldBoundary);

  /** Calculates the squared Mahalanobis distances to the other obstacle. */
  float squaredMahalanobis(const GlobalOpponentsHypothesis& other) const;
};
