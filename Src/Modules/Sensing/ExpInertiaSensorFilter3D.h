/**
* @file ExpInertiaSensorFilter3D.h
* Declaration of module ExpInertiaSensorFilter3D.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/InertiaSensorData.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Math/Matrix.h"

MODULE(ExpInertiaSensorFilter3D)
  REQUIRES(FrameInfo)
  REQUIRES(InertiaSensorData)
  REQUIRES(RobotModel)
  REQUIRES(SensorData)
  REQUIRES(RobotDimensions)
  PROVIDES_WITH_MODIFY(OrientationData)
  DEFINES_PARAMETER(Vector2f, gyroNoise, Vector2f(0.004f, 0.004f)) /**< The standard deviation of the gyroscopic sensors. */
  DEFINES_PARAMETER(Vector3f, accNoise, Vector3f(1.f, 1.f, 1.f)) /**< The standard deviation of the acceleration sensors. */
  DEFINES_PARAMETER(Vector3f, relativeKinematicsNoise, Vector3f(0.01f, 0.01f, 0.01f))
  DEFINES_PARAMETER(Vector2f, absoluteKinematicsNoise, Vector2f(0.102f, 0.102f)) /**< The standard deviation of an angle computed using kinematics. */
  DEFINES_PARAMETER(Vector2f, uprightThreshold, Vector2f(fromDegrees(20.f), fromDegrees(30.f))) /**< Use a calculated angle up to this angle (in rad). (We use the acceleration sensors otherwise.) */
  DEFINES_PARAMETER(Vector3f, horizontalProcessNoise, Vector3f(0.03f, 0.03f, 0.03f))
END_MODULE

/**
* @class ExpInertiaSensorFilter3D
* A module for estimating velocity and orientation of the torso.
*/
class ExpInertiaSensorFilter3D : public ExpInertiaSensorFilter3DBase
{
public:
  /** Default constructor. */
  ExpInertiaSensorFilter3D();

private:
  Vector2f gyroNoiseSqr;
  Vector3f relativeKinematicsNoiseSqr;
  Vector3f horizontalProcessNoiseSqr;
  Matrix3x3f accSensorCov; /**< The covariance matrix for acceleration sensor noise. */
  Matrix2x2f angleSensorCov; /**< The covariance matrix for angle sensor noise. */

  /**
  * Represents the state to be estimated.
  */
  class State
  {
  public:
    RotationMatrix rotation; /** The rotation of the torso. */

    /** Default constructor. */
    State() {}

    /**
    * Adds some world rotation given as angle axis.
    * @param value The flat vector to add.
    * @return A new object after the calculation.
    */
    State operator+(const Vector2f& value) const;

    /**
    * Adds some world rotation given as angle axis.
    * @param value The flat vector to add.
    * @return A reference this object after the calculation.
    */
    State& operator+=(const Vector2f& value);

    /**
    * Calculates a flat difference vector of two states.
    * @return The difference.
    */
    Vector2f operator-(const State& other) const;
  };

  Pose3D lastLeftFoot; /**< The pose of the left foot of the previous iteration. */
  Pose3D lastRightFoot; /**< The pose of the right foot of the previous iteration. */
  unsigned int lastTime; /**< The frame time of the previous iteration. */

  Vector2f safeRawAngle; /**< The last not corrupted angle from aldebarans angle estimation algorithm. */

  State x; /**< The estimate */
  Matrix2x2f cov; /**< The covariance of the estimate. */
  Matrix2x2f l; /**< The last caculated cholesky decomposition of \c cov. */
  State sigmaPoints[5]; /**< The last calculated sigma points. */

  Vector3f sigmaAccReadings[5]; /**< The expected acc sensor values at the sigma points. */
  Vector3f accReadingMean; /**< The mean of the expected acc sensor values at the sigma points. */
  Matrix3x3f accReadingsCov;
  Matrix3x2f accReadingsSigmaPointsCov;

  Vector2f sigmaAngleReadings[5]; /**< The expected acc sensor values at the sigma points. */
  Vector2f angleReadingMean; /**< The mean of the expected acc sensor values at the sigma points. */
  Matrix2x2f angleReadingsCov;
  Matrix2x2f angleReadingsSigmaPointsCov;

  /**
  * Updates the OrientationData representation.
  * @param orientationData The orientation data representation which is updated by this module.
  */
  void update(OrientationData& orientationData);

  /**
  * Initializes some derived constants.
  */
  void init();

  /**
  * Restores the initial state.
  */
  void reset();

  void processUpdate(const RotationMatrix* rotationOffset, const Vector3f& noiseSqr);
  void accReadingUpdate(const Vector3f& reading);
  void angleReadingUpdate(const Vector2f& reading);

  void cholOfCov();
  void generateSigmaPoints();
  void meanOfSigmaPoints();
  void covOfSigmaPoints();

  void accReadingModel(const State& sigmaPoint, Vector3f& reading) const;
  void meanOfSigmaAccReadings();
  void covOfSigmaAccReadingsAndSigmaPoints();
  void covOfSigmaAccReadings();

  void angleReadingModel(const State& sigmaPoint, Vector2f& reading) const;
  void meanOfSigmaAngleReadings();
  void covOfSigmaAngleReadingsAndSigmaPoints();
  void covOfSigmaAngleReadings();

  inline Matrix2x2f tensor(const Vector2f& a) {return Matrix2x2f(a * a.x, a * a.y);}
  inline Matrix3x3f tensor(const Vector3f& a) {return Matrix3x3f(a * a.x, a * a.y, a * a.z);}
  inline Matrix2x2f tensor(const Vector2f& a, const Vector2f& b) const {return Matrix2x2f(a * b.x, a * b.y);}
  inline Matrix3x2f tensor(const Vector3f& a, const Vector2f& b) const {return Matrix3x2f(a * b.x, a * b.y);}

  static float getRotationZ(const RotationMatrix& rot3d);
  static RotationMatrix getRotationW(const RotationMatrix& rot);
};
