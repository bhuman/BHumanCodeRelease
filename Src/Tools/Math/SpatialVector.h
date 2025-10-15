/**
 * @file SpatialVector3f.h
 *
 * This class defines a class to represent spatial vectors.
 *
 * @author Felix Wenk
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Eigen.h"
#include "Math/Pose3f.h"
#include "Representations/Configuration/MassCalibration.h"

/**
 * A spatial vector that describes angular and linear components of a body's motion/force.
 * See "Rigid Body Dynamics Algorithms" by Roy Featherstone or https://royfeatherstone.org/teaching/2008/slidesX4.pdf
 * @tparam motion Whether this is a motion vector (i.e. velocity/acceleration). Otherwise it is a force vector.
 */
template<bool motion = true>
struct SpatialVector3f
{
  SpatialVector3f() = default;
  SpatialVector3f(const Vector3f& angular, const Vector3f& linear) : angular(angular), linear(linear) {}

  SpatialVector3f<motion>& operator*=(const float factor)
  {
    angular *= factor;
    linear *= factor;
    return *this;
  }

  SpatialVector3f<motion>& operator/=(const float factor)
  {
    angular /= factor;
    linear /= factor;
    return *this;
  }

  SpatialVector3f<motion> operator*(const float factor) const
  {
    return SpatialVector3f(*this) *= factor;
  }

  SpatialVector3f<motion> operator/(const float factor) const
  {
    return SpatialVector3f(*this) /= factor;
  }

  SpatialVector3f<motion>& operator-=(const SpatialVector3f<motion>& other)
  {
    angular -= other.angular;
    linear -= other.linear;
    return *this;
  }

  SpatialVector3f<motion>& operator+=(const SpatialVector3f<motion>& other)
  {
    angular += other.angular;
    linear += other.linear;
    return *this;
  }

  SpatialVector3f<motion> operator-(const SpatialVector3f<motion>& other)
  {
    return SpatialVector<motion>(*this) -= other;
  }

  SpatialVector3f<motion> operator+(const SpatialVector3f<motion>& other) const
  {
    return SpatialVector3f<motion>(*this) += other;
  }

  SpatialVector3f<motion> operator-() const
  {
    return SpatialVector3f<motion>(-angular, -linear);
  }

  float dot(const SpatialVector3f<!motion>& other) const
  {
    return angular.dot(other.angular) + linear.dot(other.linear);
  }

  template<bool otherMotion>
  SpatialVector3f<otherMotion> cross(const SpatialVector3f<otherMotion>& other) const
  {
    static_assert(motion);
    if constexpr(otherMotion)
      return SpatialVector3f<true>(angular.cross(other.angular), angular.cross(other.linear) + linear.cross(other.angular));
    else
      return SpatialVector3f<false>(angular.cross(other.angular) + linear.cross(other.linear), angular.cross(other.linear));
  }

  static SpatialVector3f<false> applyInertia(const MassCalibration::MassInfo& massInfo, const SpatialVector3f<true>& vector)
  {
    const Vector3f massMoment = massInfo.mass * massInfo.offset;
    return SpatialVector3f<false>((massInfo.inertiaMatrix * vector.angular) + massMoment.cross(vector.linear),
                                  (massInfo.mass * vector.linear) - massMoment.cross(vector.angular));
  }

  static SpatialVector3f<motion> applyPose3f(const Pose3f& pose, const SpatialVector3f<motion>& vector)
  {
    const Vector3f rotatedAngular = pose.rotation * vector.angular;
    const Vector3f rotatedLinear = pose.rotation * vector.linear;
    if constexpr(motion)
    {
      return SpatialVector3f<true>(rotatedAngular, rotatedLinear + pose.translation.cross(rotatedAngular));
    }
    else
    {
      return SpatialVector3f<false>(rotatedAngular + pose.translation.cross(rotatedLinear), rotatedLinear);
    }
  }

  bool allFinite() const
  {
    return angular.allFinite() && linear.allFinite();
  }

  Vector3f angular = Vector3f::Zero(); /*< The angular part of this vector (i.e. top). */
  Vector3f linear = Vector3f::Zero(); /*< The linear part of this vector (i.e. bottom). */
};
