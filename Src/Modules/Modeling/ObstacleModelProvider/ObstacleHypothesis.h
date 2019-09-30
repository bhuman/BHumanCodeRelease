/**
 *
 * @author Florian Maaß
 */
#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"

class ObstacleModelProvider;
struct CameraMatrix;

class ObstacleHypothesis : public Obstacle
{
  friend class ObstacleModelProvider;

private:
  ObstacleHypothesis(const Matrix2f& covariance,
                     const Vector2f& center,
                     const Vector2f& left,
                     const Vector2f& right,
                     const unsigned lastmeasurement,
                     const unsigned seenCount,
                     const Type type);
  ObstacleHypothesis(const Type type);

  bool isBehind(const ObstacleHypothesis& other) const; /**< is this obstacle behind another one */
  bool isBetween(const float leftAngle, const float rightAngle) const /**< is the left and the right vector between the two given parameters left and right angle (radian)*/
  {
    return leftAngle > left.angle() && right.angle() > rightAngle;
  }

  void dynamic(const float odometryRotation, const Vector2f odometryTranslation, const Matrix2f odometryJacobian,
               const float odometryNoiseX, const float odometryNoiseY);
  void measurement(const ObstacleHypothesis& measurement, const float weightedSum,
                   const FieldDimensions& theFieldDimensions);
  void considerType(const ObstacleHypothesis& measurement, const int colorThreshold, const int uprightThreshold);

  bool isInImage(Vector2f& centerInImage, const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix) const; /**< is an obstacle in the image */
  bool fieldBoundaryFurtherAsObstacle(const Vector2f& centerInImage, const unsigned notSeenThreshold,
                                      const CameraInfo& theCameraInfo, const CameraMatrix& theCameraMatrix,
                                      const ImageCoordinateSystem& theImageCoordinateSystem,
                                      const FieldBoundary& theFieldBoundary);

  int color = 0;                          /**< negative value is for teammates */
  int upright = 0;                        /**< negative value is for fallen robots */
  unsigned seenCount = 0;                 /**< Somewhat validity (between minPercepts and maxPercepts in ObstacleModelProvider) */
  unsigned notSeenButShouldSeenCount = 0; /**< how many times the obstacle was not seen but could be measured */
};
