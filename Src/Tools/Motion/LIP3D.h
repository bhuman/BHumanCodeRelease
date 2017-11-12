#pragma once

#include "LIP.h"
#include "Tools/Math/Eigen.h"

class LIP3D
{
public:
  Vector2f position = Vector2f::Zero();
  Vector2f velocity = Vector2f::Zero();

  LIP3D(const Array2f& LIPHeights);
  LIP3D(const Vector2f& position, const Vector2f& velocity, const Array2f& LIPHeights);
  LIP3D(const LIP3D& other);

  LIP3D& operator=(const LIP3D& other);

  void setLIPHeights(const Array2f& LIPHeights);
  const Array2f& getLIPHeights() const { return heights; };
  const Array2f& getK() const { return k; };
  LIP getXLIP() const;
  LIP getYLIP() const;

  LIP3D predict(float time, const Vector2f& zmp = Vector2f::Zero()) const;
  LIP3D& update(float time, const Vector2f& zmp = Vector2f::Zero());

  Vector2f requiredVelocity(const Vector2f& pos, float time) const;

private:
  Array2f k = Array2f::Zero();
  Array2f heights = Array2f::Zero();
};

inline LIP3D::LIP3D(const Array2f& LIPHeights)
{
  setLIPHeights(LIPHeights);
}

inline LIP3D::LIP3D(const Vector2f& position, const Vector2f& velocity, const Array2f& LIPHeights) :
  position(position), velocity(velocity)
{
  setLIPHeights(LIPHeights);
}

inline LIP3D::LIP3D(const LIP3D& other) :
  position(other.position), velocity(other.velocity)
{
  setLIPHeights(other.heights);
}

inline LIP3D& LIP3D::operator=(const LIP3D& other)
{
  position = other.position;
  velocity = other.velocity;
  setLIPHeights(other.heights);
  return *this;
}

inline void LIP3D::setLIPHeights(const Array2f& LIPHeights)
{
  heights = LIPHeights;
  k << std::sqrt(Constants::g / heights.x()), std::sqrt(Constants::g / heights.y());
}

inline LIP LIP3D::getXLIP() const
{
  return LIP(position.x(), velocity.x(), heights.x());
}

inline LIP LIP3D::getYLIP() const
{
  return LIP(position.y(), velocity.y(), heights.y());
}

inline LIP3D LIP3D::predict(float time, const Vector2f& zmp) const
{
  return LIP3D(*this).update(time, zmp);
}

inline LIP3D& LIP3D::update(float time, const Vector2f& zmp)
{
  const Array2f sinhkt(std::sinh(k.x() * time), std::sinh(k.y() * time));
  const Array2f coshkt(std::cosh(k.x() * time), std::cosh(k.y() * time));

  const Vector2f newPosition = (position - zmp).array() * coshkt + (velocity.array() / k) * sinhkt + zmp.array();
  velocity = ((position - zmp).array() * k * sinhkt + velocity.array() * coshkt).eval();
  position = newPosition;
  return *this;
}

inline Vector2f LIP3D::requiredVelocity(const Vector2f& pos, float time) const
{
  const Array2f sinhkt(std::sinh(k.x() * time), std::sinh(k.y() * time));
  const Array2f coshkt(std::cosh(k.x() * time), std::cosh(k.y() * time));

  return k.array() * (pos.array() - position.array() * coshkt) / sinhkt;
}
