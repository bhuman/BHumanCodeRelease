/**
 * @file Pose3fWithCov.h
 * This file declares a streamable that represents a pose together with its covariance
 * it is important to note that the covariance is in the tangent space of the SE3 lie group (lie algebra)
 * so it can not be directly converted to Cartesian coordinates but instead via the exponential map
 *
 * [1] L. Meyer, K. H. Strobl, and R. Triebel, “The Probabilistic Robot Kinematics Model and its Application to Sensor Fusion,” in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2022, pp. 3263–3270. doi: 10.1109/IROS47612.2022.9981399.
 *
 * @author Yannik Meinken
 */


#pragma once

#include "Pose3f.h"
#include "Streaming/AutoStreamable.h"
#include "SE3Tools.h"

STREAMABLE_WITH_BASE(SE3WithCov, Pose3f,
{
  SE3WithCov() = default;
  SE3WithCov(const SE3WithCov& other) = default;
  SE3WithCov(const Pose3f& pose):Pose3f(pose){};
  SE3WithCov(const Pose3f& pose, const Matrix6f& cov);
  SE3WithCov(const RotationMatrix& rot, const Matrix3f& rotCov);

  SE3WithCov& operator=(const SE3WithCov& other) = default;

  bool operator==(const SE3WithCov& other) const;
  bool operator!=(const SE3WithCov& other) const;

  SE3WithCov operator*(const SE3WithCov& other) const;

  SE3WithCov operator*(const Pose3f& other) const;
  SE3WithCov operator*(const RotationMatrix& rot) const;
  SE3WithCov operator+(const Vector3f& translation) const;

  /**
   * Boxplus operator to add a small perturbation expressed as a vector in the tangent space
   * SE3 + R⁶ -> SE3
   */
  SE3WithCov boxplus(const Vector6f tangentVector) const;

  // should not be necessary as it should be derived from Pose3f but for some reason does not compile without
  Vector3f operator*(const Vector3f& vec) const;

  SE3WithCov& operator*=(const SE3WithCov& other);

  SE3WithCov& operator*=(const Pose3f& other);
  SE3WithCov& operator*=(const RotationMatrix& rot);
  SE3WithCov& operator+=(const Vector3f& translation);

  SE3WithCov& conc(const SE3WithCov& other);
  SE3WithCov& conc(const Pose3f& other);

  SE3WithCov& translate(const Vector3f& trans);
  SE3WithCov& translate(float x, float y, float z);

  SE3WithCov translated(const Vector3f& trans) const;
  SE3WithCov translated(float x, float y, float z) const;

  SE3WithCov& rotate(const RotationMatrix& rot);
  SE3WithCov& rotateX(float angle);
  SE3WithCov& rotateY(float angle);
  SE3WithCov& rotateZ(float angle),

  (Matrix6f)(Matrix6f::Zero()) covariance,
});
