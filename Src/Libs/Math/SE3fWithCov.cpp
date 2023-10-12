/**
 * @file Pose3fWithCov.cpp
 * This file implements a streamable that represents a pose together with its covariance
 * it is important to note that the covariance is in the tangent space of the SE3 lie group (lie algebra)
 * so it can not be directly converted to Cartesian coordinates but instead via the exponential map
 *
 * Note that the uncertainty is applied in the local reference frame
 *
 * [1] L. Meyer, K. H. Strobl, and R. Triebel, “The Probabilistic Robot Kinematics Model and its Application to Sensor Fusion,” in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Oct. 2022, pp. 3263–3270. doi: 10.1109/IROS47612.2022.9981399.
 *
 * @author Yannik Meinken
 */

#include "SE3fWithCov.h"
#include "Covariance.h"

SE3WithCov::SE3WithCov(const Pose3f& pose, const Matrix6f& cov):
  Pose3f(pose), covariance(cov)
{}

SE3WithCov::SE3WithCov(const RotationMatrix& rot, const Matrix3f& rotCov):
  Pose3f(rot), covariance(Matrix6f::Zero())
{
  covariance.block<3, 3>(2, 2) = rotCov;
}

bool SE3WithCov::operator==(const SE3WithCov& other) const
{
  bool equalPose = this->Pose3f::operator==(other);
  bool equalCov = covariance == other.covariance;

  return equalPose && equalCov;
}

bool SE3WithCov::operator!=(const SE3WithCov& other) const
{
  return !(*this == other);
}

SE3WithCov SE3WithCov::operator*(const SE3WithCov& other) const
{
  Pose3f newPose = Pose3f(this->Pose3f::operator*(other));

  // [1]
  // note that the error is considered in the local frame
  // todo: only first order approximation maybe use higher order in the future
  Matrix6f adjointInverseOther = SE3::adjoint(other.inverse());
  // todo: exploit matrix structure to speed up calculation
  Matrix6f newCov = adjointInverseOther * covariance * adjointInverseOther.transpose() + other.covariance;

  Covariance::fixCovariance<6>(newCov);

  return SE3WithCov(newPose, newCov);
}

SE3WithCov SE3WithCov::operator*(const Pose3f& other) const
{
  return *this * SE3WithCov(other);
}

SE3WithCov SE3WithCov::operator*(const RotationMatrix& rot) const
{
  return *this * Pose3f(rot);
}

SE3WithCov SE3WithCov::operator+(const Vector3f& translation) const
{
  return *this * Pose3f(translation);
}

SE3WithCov SE3WithCov::boxplus(const Vector6f tangentVector) const
{
  // note that the perturbation is applied in the local frame
  return *this * SE3::exp(tangentVector);
}

SE3WithCov& SE3WithCov::operator*=(const SE3WithCov& other)
{
  //does this actually change the value stored at "this"?
  *this = *this * other;
  return *this;
}

SE3WithCov& SE3WithCov::operator*=(const Pose3f& other)
{
  *this = *this * other;
  return *this;
}

SE3WithCov& SE3WithCov::operator*=(const RotationMatrix& rot)
{
  *this = *this * rot;
  return *this;
}

SE3WithCov& SE3WithCov::operator+=(const Vector3f& translation)
{
  *this = *this + translation;
  return *this;
}

Vector3f SE3WithCov::operator*(const Vector3f& vec) const
{
  return this->Pose3f::operator*(vec);
}

SE3WithCov& SE3WithCov::conc(const SE3WithCov& other)
{
  return *this *= other;
}

SE3WithCov& SE3WithCov::conc(const Pose3f& other)
{
  return *this *= other;
}

SE3WithCov& SE3WithCov::translate(const Vector3f& trans)
{
  return *this *= Pose3f(trans);
}

SE3WithCov& SE3WithCov::translate(float x, float y, float z)
{
  return this->translate(Vector3f(x, y, z));
}

SE3WithCov SE3WithCov::translated(const Vector3f& trans) const
{
  return *this + trans;
}

SE3WithCov SE3WithCov::translated(const float x, const float y, const float z) const
{
  return *this + Vector3f(x, y, z);
}

SE3WithCov& SE3WithCov::rotate(const RotationMatrix& rot)
{
  return *this *= rot;
}

SE3WithCov& SE3WithCov::rotateX(float angle)
{
  return *this *= RotationMatrix::aroundX(angle);
}

SE3WithCov& SE3WithCov::rotateY(float angle)
{
  return *this *= RotationMatrix::aroundY(angle);
}

SE3WithCov& SE3WithCov::rotateZ(float angle)
{
  return *this *= RotationMatrix::aroundZ(angle);
}
