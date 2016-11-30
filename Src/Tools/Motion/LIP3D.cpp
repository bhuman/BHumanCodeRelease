#include "LIP3D.h"

LIP3D& LIP3D::update(float time, const Vector2f& zmp)
{
  const Array2f sinhkt(std::sinh(k.x() * time), std::sinh(k.y() * time));
  const Array2f coshkt(std::cosh(k.x() * time), std::cosh(k.y() * time));

  const Vector2f newPosition = (position - zmp).array() * coshkt + (velocity.array() / k) * sinhkt + zmp.array();
  const Vector2f newVelocity = (position - zmp).array() * k * sinhkt + velocity.array() * coshkt;
  position = newPosition;
  velocity = newVelocity;
  return *this;
}

Vector2f LIP3D::requiredVelocity(const Vector2f& pos, float time)
{
  const Array2f sinhkt(std::sinh(k.x() * time), std::sinh(k.y() * time));
  const Array2f coshkt(std::cosh(k.x() * time), std::cosh(k.y() * time));

  return k.array() * (pos.array() - position.array() * coshkt) / sinhkt;
}
