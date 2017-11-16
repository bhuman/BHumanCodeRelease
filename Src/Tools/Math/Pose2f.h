#pragma once

#include "Angle.h"
#include "Eigen.h"
#include "Random.h"
#include "Tools/Range.h"

STREAMABLE(Pose2f,
{
  Pose2f() = default;
  Pose2f(const Pose2f& other) = default;
  Pose2f(const float x, const float y);
  Pose2f(const Vector2f& translation);
  Pose2f(const Angle rotation);
  Pose2f(const Angle rotation, const Vector2f& translation);
  Pose2f(const Angle rotation, const float x, const float y);
  Pose2f(const float rot, const float x, const float y);

  Pose2f& operator=(const Pose2f& other) = default;

  Vector2f operator*(const Vector2f& other) const;

  /**
   * Concatenation of this pose with another. The resulting rotation is not normalized!
   * Use operator+ / operator+= for normalized rotations.
   */
  Pose2f operator*(const Pose2f& other) const;
  Pose2f& operator*=(const Pose2f& other);

  /**
   * Concatenation of this pose with another pose. The resulting rotation is not normalized!
   */
  Pose2f operator+(const Pose2f& other) const;
  Pose2f& operator+=(const Pose2f& other);

  /**
   * Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates C-A=B.
   * @param other The other pose that will be used as origin for the new pose.
   * @return A reference to this pose after calculating the difference.
   */
  Pose2f& operator-=(const Pose2f& other);

  /**
   * Difference of this pose relative to another pose.
   * @param other The other pose that will be used as origin for the new pose.
   * @return The resulting pose.
   */
  Pose2f operator-(const Pose2f& other) const;

  /** unary minus */
  Pose2f operator-() const;

  /**
   * Comparison of another pose with this one.
   * @param other The other pose that will be compared to this one
   * @return Whether the two poses are equal.
   */
  bool operator==(const Pose2f& other) const;

  /**
   * Comparison of another pose with this one.
   * @param other The other pose that will be compared to this one
   * @return Whether the two poses are unequal.
   */
  bool operator!=(const Pose2f& other) const;

  Pose2f& translate(const Vector2f& trans);
  Pose2f& translate(const float x, const float y);
  Pose2f& rotate(const Angle& rot);

  /** Inverts this pose and returns reference to this. */
  Pose2f& invert();
  /** Calculates the inverse transformation of this pose */
  Pose2f inverse() const;

  Pose2f dotMirror() const;

  bool isFinite() const;

  /**
   * The function creates a random pose.
   * @param x The range for x-values of the pose.
   * @param y The range for y-values of the pose.
   * @param angle The range for the rotation of the pose.
   */
  static Pose2f random(const Rangef& angle, const Rangef& x, const Rangef& y),

  (Angle)(0) rotation,
  (Vector2f)(Vector2f::Zero()) translation,
});

inline Pose2f::Pose2f(const float x, const float y) :
  translation(x, y)
{}

inline Pose2f::Pose2f(const Vector2f& translation) :
  translation(translation)
{}

inline Pose2f::Pose2f(const Angle rotation) :
  rotation(rotation)
{}

inline Pose2f::Pose2f(const Angle rotation, const Vector2f& translation) :
  rotation(rotation), translation(translation)
{}

inline Pose2f::Pose2f(const Angle rotation, const float x, const float y) :
  rotation(rotation), translation(x, y)
{}

inline Pose2f::Pose2f(const float rot, const float x, const float y) :
  rotation(rot), translation(x, y)
{}

inline Vector2f Pose2f::operator*(const Vector2f& other) const
{
  const float s = std::sin(rotation);
  const float c = std::cos(rotation);
  return (Vector2f(other.x() * c - other.y() * s, other.x() * s + other.y() * c) + translation);
}

inline Pose2f Pose2f::operator*(const Pose2f& other) const
{
  return Pose2f(*this) *= other;
}

inline Pose2f& Pose2f::operator*=(const Pose2f& other)
{
  translation = *this * other.translation;
  rotation += other.rotation;
  rotation.normalize();
  return *this;
}

inline Pose2f Pose2f::operator+(const Pose2f& other) const
{
  return Pose2f(*this) += other;
}

inline Pose2f& Pose2f::operator+=(const Pose2f& other)
{
  translation = *this * other.translation;
  rotation += other.rotation;
  rotation.normalize();
  return *this;
}

inline Pose2f& Pose2f::operator-=(const Pose2f& other)
{
  translation -= other.translation;
  Pose2f p(-other.rotation);
  return *this = p + *this;
}

inline Pose2f Pose2f::operator-(const Pose2f& other) const
{
  return Pose2f(*this) -= other;
}

inline Pose2f Pose2f::operator-() const
{
  return Pose2f() - (*this);
}

inline bool Pose2f::operator==(const Pose2f& other) const
{
  return ((translation == other.translation) && (rotation == other.rotation));
}

inline bool Pose2f::operator!=(const Pose2f& other) const
{
  return !(Pose2f(*this) == other);
}

inline Pose2f& Pose2f::translate(const Vector2f& trans)
{
  translation = *this * trans;
  return *this;
}

inline Pose2f& Pose2f::translate(const float x, const float y)
{
  translation = *this * Vector2f(x, y);
  return *this;
}

inline Pose2f& Pose2f::rotate(const Angle& rot)
{
  rotation += rot;
  return *this;
}

inline Pose2f& Pose2f::invert()
{
  rotation = -rotation;
  const Vector2f trans = -translation;
  translation = Eigen::Rotation2D<float>(rotation) * trans;
  return *this;
}

inline Pose2f Pose2f::inverse() const
{
  return Pose2f(*this).invert();
}

inline Pose2f Pose2f::dotMirror() const
{
  return Pose2f(Angle::normalize(rotation + pi), -translation);
}

inline bool Pose2f::isFinite() const
{
  return std::isfinite(translation.x()) && std::isfinite(translation.y()) && std::isfinite(rotation);
}

inline Pose2f Pose2f::random(const Rangef& angle, const Rangef& x, const Rangef& y)
{
  return Pose2f(Random::uniform(angle.min, angle.max), Vector2f(Random::uniform(x.min, x.max), Random::uniform(y.min, y.max)));
}
