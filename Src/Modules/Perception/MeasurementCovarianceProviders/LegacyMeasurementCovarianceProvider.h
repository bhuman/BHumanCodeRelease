/**
 * @file LegacyMeasurementCovarianceProvider.h
 *
 * This module provides the function that was previously implemented in Measurements::positionToCovarianceMatrixInRobotCoordinates.
 * Although this approach for determining an uncertainty for a given perception has been used by B-Human for over a decade, it should
 * be replaced with a more sophisticated approach soon.
 * Having this functionality inside a module makes this replacement easier.
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/MeasurementCovariance.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Framework/Module.h"

MODULE(LegacyMeasurementCovarianceProvider,
{,
  REQUIRES(CameraMatrix),
  REQUIRES(CameraInfo),
  REQUIRES(MotionInfo),
  PROVIDES(MeasurementCovariance),
  LOADS_PARAMETERS(
  {,
    (Vector2f) robotRotationDeviationDefault,               /**< Deviation of the rotation of the robot's torso */
    (Vector2f) robotRotationDeviationInStandDefault,        /**< Deviation of the rotation of the robot's torso when it is standing. */
  }),
});

class LegacyMeasurementCovarianceProvider : public LegacyMeasurementCovarianceProviderBase
{
private:
  Pose3f inverseCameraMatrix;                 /**< Precomputed matrix that is needed for every covariance computation. */
  Vector2f currentDefaultRotationDeviation;   /**< Set to either robotRotationDeviationDefault or robotRotationDeviationInStandDefault */

  /** Update method that sets the actual function in the representation
   * @param measurementCovariance The representation that contains a function for computing the covariance
   * */
  void update(MeasurementCovariance& measurementCovariance);

  /** Implementation from Measurements::positionToCovarianceMatrixInRobotCoordinates
   * @param p A 2D position in robot-relative coordinates.
   * @param rotationDeviation The robot's current rotation deviation
   * @return A 2D covariance Matrix.
   */
  Matrix2f computeCovariance(const Vector2f& p, const Vector2f& rotationDeviation) const;

  /** Some debug drawings for testing
   * @param measurementCovariance The representation that is tested here
   */
  void draw(const MeasurementCovariance& measurementCovariance);
};
