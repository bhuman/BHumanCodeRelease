/**
 * @file Tools/Math/Rotation.h
 * Rotation related functionality
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#pragma once

#include "Angle.h"
#include "Approx.h"
#include "Eigen.h"

namespace Rotation
{
  Quaternionf aroundX(float angle);
  Quaternionf aroundY(float angle);
  Quaternionf aroundZ(float angle);

  /**
   * The spherical linear interpolation (slerp) between the two rotations.
   * Where interpolate(0.0f, q1, q2) = q1 and interpolate(1.0f, q1, q2) = q2.
   * @param t interpolation factor. Range: [0.0f, 1.0f].
   */
  Quaternionf interpolate(float t, const Quaternionf& q1, const Quaternionf& q2);

  Quaternionf removeZRotation(const Quaternionf& rotation);
  Quaternionf splitOffZRotation(const Quaternionf& rotation, Quaternionf& zRot);

  namespace Euler
  {
    // Euler angles are expressed in z y x manner.

    Quaternionf fromAngles(float x, float y, float z);
    Quaternionf fromAngles(const Vector3a& rotation);
    Quaternionf fromAngles(const Vector3f& rotation);
    Vector3f getAngles(const Quaternionf& rot);
    float getXAngle(const Quaternionf& rot);
    float getYAngle(const Quaternionf& rot);
    float getZAngle(const Quaternionf& rot);
  }

  namespace Aldebaran
  {
    // Aldebaran uses these angles when calculating the orientation of the robot via the imu
    float getXAngle(const Quaternionf& rot);
    float getXAngle(const Matrix3f& rot);
    float getYAngle(const Quaternionf& rot);
    float getYAngle(const Matrix3f& rot);
  }

  namespace AngleAxis
  {
    // Do not missinterpret the vectors as Euler angles!
    Vector3f pack(const AngleAxisf& angleAxis);
    AngleAxisf unpack(const Vector3f& angleAxisVec);
  }
}

inline Quaternionf Rotation::aroundX(float angle)
{
  return Quaternionf(AngleAxisf(angle, Vector3f::UnitX()));
}

inline Quaternionf Rotation::aroundY(float angle)
{
  return Quaternionf(AngleAxisf(angle, Vector3f::UnitY()));
}

inline Quaternionf Rotation::aroundZ(float angle)
{
  return Quaternionf(AngleAxisf(angle, Vector3f::UnitZ()));
}

inline Quaternionf Rotation::interpolate(float t, const Quaternionf& q1, const Quaternionf& q2)
{
  return q1.slerp(t, q2);
}

inline Quaternionf Rotation::removeZRotation(const Quaternionf& rotation)
{
  const Vector3f& z = Vector3f::UnitZ();
  const Vector3f zR = rotation.inverse() * z;
  const Vector3f c = zR.cross(z);
  const float sin = c.norm();
  const float cos = zR.dot(z);
  if(Approx::isZero(sin))
    if(cos < 0.f) // 180 degree rotation
      return rotation; // There's no unique decomposition.
    else
      return Quaternionf::Identity();
  else
  {
    const float angle = std::atan2(sin, cos);
    return Quaternionf(AngleAxisf(angle, c.normalized()));
  }
}

inline Quaternionf Rotation::splitOffZRotation(const Quaternionf& rotation, Quaternionf& zRot)
{
  const Quaternionf xyRot = removeZRotation(rotation);
  zRot = rotation * xyRot.inverse();
  return xyRot;
}

inline Quaternionf Rotation::Euler::fromAngles(float x, float y, float z)
{
  return AngleAxisf(z, Vector3f::UnitZ()) * AngleAxisf(y, Vector3f::UnitY()) * AngleAxisf(x, Vector3f::UnitX());
}

inline Quaternionf Rotation::Euler::fromAngles(const Vector3a& rotation)
{
  return fromAngles(rotation.x(), rotation.y(), rotation.z());
}

inline Quaternionf Rotation::Euler::fromAngles(const Vector3f& rotation)
{
  return fromAngles(rotation.x(), rotation.y(), rotation.z());
}

inline Vector3f Rotation::Euler::getAngles(const Quaternionf& rot)
{
  const Matrix3f mat = rot.normalized().toRotationMatrix();
  const float m20 = mat(2, 0);

  if(std::abs(m20) < 0.999999f)
  {
    const float m00 = mat(0, 0);
    const float m10 = mat(1, 0);
    const float m21 = mat(2, 1);
    const float m22 = mat(2, 2);

    const float y1 = -std::asin(m20);
    const float y2 = pi - y1;
    const float cy1 = std::cos(y1);
    const float cy2 = std::cos(y2);
    const float x1 = std::atan2(m21 / cy1, m22 / cy1);
    const float x2 = std::atan2(m21 / cy2, m22 / cy2);
    const float z1 = std::atan2(m10 / cy1, m00 / cy1);
    const float z2 = std::atan2(m10 / cy2, m00 / cy2);

    Vector3f v1(x1, y1, z1);
    Vector3f v2(x2, y2, z2);

    return v1.norm() < v2.norm() ? v1 : v2;
  }
  else
  {
    const float x = std::atan2(mat(0, 1), mat(0, 2)); // x = +-z + atan2(...) , but we set z = 0...
    const float y = -std::asin(m20);

    return Vector3f(x, y, 0.f);
  }
}

inline float Rotation::Euler::getXAngle(const Quaternionf& rot)
{
  return getAngles(rot).x();
}

inline float Rotation::Euler::getYAngle(const Quaternionf& rot)
{
  return getAngles(rot).y();
}

inline float Rotation::Euler::getZAngle(const Quaternionf& rot)
{
  return getAngles(rot).z();
}

inline float Rotation::Aldebaran::getXAngle(const Quaternionf& rot)
{
  //     | a b c |        | a d g |        | 0 |   | g |
  // A = | d e f |   At = | b e h |   At * | 0 | = | h | = v
  //     | g h i |        | c f i |        | 1 |   | i |

  // Project        | 0 |
  // v into  : v -> | h | = v'
  // yz plane       | i |

  // calculate angle      | 0 |       angleX = acos((v' * z) / (|v'| * |z|))
  // bewteen v' and : z = | 0 |,  <=> angleX = acos(i / |v'|)
  // z-axis of A          | 1 |   <=> angleX = acos(i / sqrt(h * h + i * i))

  const float tx = 2.f * rot.x();
  const float h = 2.f * rot.z() * rot.y() + tx * rot.w();
  const float i = 1.f - (tx * rot.x() + 2.f * rot.y() * rot.y());

  const float len = std::sqrt(h * h + i * i);
  return len > 1e-5f ? std::acos(i / len) * (h > 0.f ? 1.f : -1.f) : 0.f;
}

inline float Rotation::Aldebaran::getXAngle(const Matrix3f& rot)
{
  //     | a b c |        | a d g |        | 0 |   | g |
  // A = | d e f |   At = | b e h |   At * | 0 | = | h | = v
  //     | g h i |        | c f i |        | 1 |   | i |

  // Project        | 0 |
  // v into  : v -> | h | = v'
  // yz plane       | i |

  // calculate angle      | 0 |       angleX = acos((v' * z) / (|v'| * |z|))
  // bewteen v' and : z = | 0 |,  <=> angleX = acos(i / |v'|)
  // z-axis of A          | 1 |   <=> angleX = acos(i / sqrt(h * h + i * i))

  const float h = rot(2, 1);
  const float i = rot(2, 2);

  const float len = std::sqrt(h * h + i * i);
  return len > 1e-5f ? std::acos(i / len) * (h > 0.f ? 1.f : -1.f) : 0.f;
}

inline float Rotation::Aldebaran::getYAngle(const Quaternionf& rot)
{
  //     | a b c |        | a d g |        | 0 |   | g |
  // A = | d e f |   At = | b e h |   At * | 0 | = | h | = v
  //     | g h i |        | c f i |        | 1 |   | i |

  // Project        | g |
  // v into  : v -> | 0 | = v'
  // xz plane       | i |

  // calculate angle      | 0 |       angleX = acos((v' * z) / (|v'| * |z|))
  // bewteen v' and : z = | 0 |,  <=> angleX = acos(i / |v'|)
  // z-axis of A          | 1 |   <=> angleX = acos(i / sqrt(g * g + i * i))

  // calculate g and i

  const float ty = 2.f * rot.y();
  const float g = 2.f * rot.z() * rot.x() - ty * rot.w();
  const float i = 1.f - (2.f * rot.x() * rot.x() + ty * rot.y());

  const float len = std::sqrt(g * g + i * i);
  return len > 1e-5f ? std::acos(i / len) * (g > 0.f ? -1.f : 1.f) : 0.f;
}

inline float Rotation::Aldebaran::getYAngle(const Matrix3f& rot)
{
  //     | a b c |        | a d g |        | 0 |   | g |
  // A = | d e f |   At = | b e h |   At * | 0 | = | h | = v
  //     | g h i |        | c f i |        | 1 |   | i |

  // Project        | g |
  // v into  : v -> | 0 | = v'
  // xz plane       | i |

  // calculate angle      | 0 |       angleX = acos((v' * z) / (|v'| * |z|))
  // bewteen v' and : z = | 0 |,  <=> angleX = acos(i / |v'|)
  // z-axis of A          | 1 |   <=> angleX = acos(i / sqrt(g * g + i * i))

  const float g = rot(2, 0);
  const float i = rot(2, 2);

  const float len = std::sqrt(g * g + i * i);
  return len > 1e-5f ? std::acos(i / len) * (g > 0.f ? -1.f : 1.f) : 0.f;
}

inline Vector3f Rotation::AngleAxis::pack(const AngleAxisf& angleAxis)
{
  const float angle = angleAxis.angle();
  if(Approx::isZero(angle))
    return Vector3f::Zero();
  else
    return angleAxis.axis().normalized(angle);
}

inline AngleAxisf Rotation::AngleAxis::unpack(const Vector3f& angleAxisVec)
{
  const float angle = angleAxisVec.norm();
  if(Approx::isZero(angle))
    return AngleAxisf::Identity();
  else
    return AngleAxisf(angle, angleAxisVec.normalized());
}
