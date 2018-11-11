/**
 * @file Tools/Math/RotationMatrix.cpp
 * Implementation of class RotationMatrix
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 * @author <a href="mailto:alexists@tzi.de">Alexis Tsogias</a>
 */

#include "Rotation.h"
#include "RotationMatrix.h"

AngleAxisf RotationMatrix::getAngleAxis() const
{
  return AngleAxisf(*this);
}

Vector3f RotationMatrix::getPackedAngleAxis() const
{
  return Rotation::AngleAxis::pack(getAngleAxis());
}

RotationMatrix& RotationMatrix::rotateX(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m01 = mat(0, 1);
  const float m11 = mat(1, 1);
  const float m21 = mat(2, 1);
  mat(0, 1) = m01 * c + mat(0, 2) * s;
  mat(1, 1) = m11 * c + mat(1, 2) * s;
  mat(2, 1) = m21 * c + mat(2, 2) * s;
  mat(0, 2) = mat(0, 2) * c - m01 * s;
  mat(1, 2) = mat(1, 2) * c - m11 * s;
  mat(2, 2) = mat(2, 2) * c - m21 * s;
  return *this;
}

RotationMatrix& RotationMatrix::rotateY(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m00 = mat(0, 0);
  const float m10 = mat(1, 0);
  const float m20 = mat(2, 0);
  mat(0, 0) = m00 * c - mat(0, 2) * s;
  mat(1, 0) = m10 * c - mat(1, 2) * s;
  mat(2, 0) = m20 * c - mat(2, 2) * s;
  mat(0, 2) = mat(0, 2) * c + m00 * s;
  mat(1, 2) = mat(1, 2) * c + m10 * s;
  mat(2, 2) = mat(2, 2) * c + m20 * s;
  return *this;
}

RotationMatrix& RotationMatrix::rotateZ(const float angle)
{
  Matrix3f& mat = *this;
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  const float m00 = mat(0, 0);
  const float m10 = mat(1, 0);
  const float m20 = mat(2, 0);
  mat(0, 0) = m00 * c + mat(0, 1) * s;
  mat(1, 0) = m10 * c + mat(1, 1) * s;
  mat(2, 0) = m20 * c + mat(2, 1) * s;
  mat(0, 1) = mat(0, 1) * c - m00 * s;
  mat(1, 1) = mat(1, 1) * c - m10 * s;
  mat(2, 1) = mat(2, 1) * c - m20 * s;
  return *this;
}

RotationMatrix RotationMatrix::aroundX(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          1.f, 0.f, 0.f,
          0.f, c, -s,
          0.f, s, c).finished();
}

RotationMatrix RotationMatrix::aroundY(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          c, 0.f, s,
          0.f, 1.f, 0.f,
          -s, 0.f, c).finished();
}

RotationMatrix RotationMatrix::aroundZ(const float angle)
{
  const float c = std::cos(angle);
  const float s = std::sin(angle);
  return (RotationMatrix() <<
          c, -s, 0.f,
          s, c, 0.f,
          0.f, 0.f, 1.f).finished();
}

RotationMatrix RotationMatrix::fromEulerAngles(const float x, const float y, const float z)
{
  return Rotation::Euler::fromAngles(x, y, z);
}

RotationMatrix RotationMatrix::fromEulerAngles(const Vector3f rotation)
{
  return Rotation::Euler::fromAngles(rotation);
}
