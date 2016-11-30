#pragma once

#include "RotationMatrix.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Pose3f,
{
  Pose3f() = default;
  Pose3f(const Pose3f& other) = default;
  Pose3f(const RotationMatrix& rotation, const Vector3f& translation);
  Pose3f(const RotationMatrix& rotation);
  Pose3f(const Vector3f& translation);
  Pose3f(const float x, const float y, const float z);

  Pose3f& operator=(const Pose3f& other) = default;

  bool operator==(const Pose3f& other) const;
  bool operator!=(const Pose3f& other) const;

  Vector3f operator*(const Vector3f& vec) const;

  Pose3f operator*(const Pose3f& other) const;
  Pose3f operator*(const RotationMatrix& rotation) const;
  Pose3f operator+(const Vector3f& translation) const;

  Pose3f& operator*=(const Pose3f& other);
  Pose3f& operator*=(const RotationMatrix& rotation);
  Pose3f& operator+=(const Vector3f& translation);

  Pose3f& conc(const Pose3f& other);

  Pose3f& translate(const Vector3f& trans);
  Pose3f& translate(float x, float y, float z);

  Pose3f translated(const Vector3f& trans) const;
  Pose3f translated(float x, float y, float z) const;

  Pose3f& rotate(const RotationMatrix& rot);
  Pose3f& rotateX(float angle);
  Pose3f& rotateY(float angle);
  Pose3f& rotateZ(float angle);

  Pose3f& invert();
  Pose3f inverse() const,

  (RotationMatrix) rotation,
  (Vector3f)(Vector3f::Zero()) translation,
});

inline Pose3f::Pose3f(const RotationMatrix& rotation, const Vector3f& translation) :
  rotation(rotation), translation(translation)
{}

inline Pose3f::Pose3f(const RotationMatrix& rotation) :
  rotation(rotation)
{}

inline Pose3f::Pose3f(const Vector3f& translation) :
  translation(translation)
{}

inline Pose3f::Pose3f(const float x, const float y, const float z) :
  translation(x, y, z)
{}

inline bool Pose3f::operator==(const Pose3f& other) const
{
  return translation == other.translation && rotation == other.rotation;
}

inline bool Pose3f::operator!=(const Pose3f& other) const
{
  return !(*this == other);
}

inline Vector3f Pose3f::operator*(const Vector3f& vec) const
{
  return rotation * vec + translation;
}

inline Pose3f Pose3f::operator*(const Pose3f& other) const
{
  return Pose3f(rotation * other.rotation, *this * other.translation);
}

inline Pose3f Pose3f::operator*(const RotationMatrix& rot) const
{
  return Pose3f(rotation * rot, translation);
}

inline Pose3f Pose3f::operator+(const Vector3f& trans) const
{
  return Pose3f(rotation, *this * trans);
}

inline Pose3f& Pose3f::operator*=(const Pose3f& other)
{
  translation = *this * other.translation;
  rotation *= other.rotation;
  return *this;
}

inline Pose3f& Pose3f::operator*=(const RotationMatrix& rot)
{
  rotation *= rot;
  return *this;
}

inline Pose3f& Pose3f::operator+=(const Vector3f& trans)
{
  translation = *this * trans;
  return *this;
}

inline Pose3f& Pose3f::conc(const Pose3f& other)
{
  return *this *= other;
}

inline Pose3f& Pose3f::translate(const Vector3f& trans)
{
  return *this += trans;
}

inline Pose3f& Pose3f::translate(float x, float y, float z)
{
  return *this += Vector3f(x, y, z);
}

inline Pose3f Pose3f::translated(const Vector3f& trans) const
{
  return *this + trans;
}

inline Pose3f Pose3f::translated(const float x, const float y, const float z) const
{
  return *this + Vector3f(x, y, z);
}

inline Pose3f& Pose3f::rotate(const RotationMatrix& rot)
{
  return *this *= rot;
}

inline Pose3f& Pose3f::rotateX(float angle)
{
  return *this *= RotationMatrix::aroundX(angle);
}

inline Pose3f& Pose3f::rotateY(float angle)
{
  return *this *= RotationMatrix::aroundY(angle);
}

inline Pose3f& Pose3f::rotateZ(float angle)
{
  return *this *= RotationMatrix::aroundZ(angle);
}

inline Pose3f& Pose3f::invert()
{
  rotation = rotation.inverse();
  translation = rotation * -translation;
  return *this;
}

inline Pose3f Pose3f::inverse() const
{
  return Pose3f(*this).invert();
}

inline Pose3f operator*(const RotationMatrix& rotation, const Pose3f& pose)
{
  return Pose3f(rotation * pose.rotation, rotation * pose.translation);
}
