//Do NOT directly include this file, instead include "Tools/Math/Eigen.h"

inline PlainObject& normalize(RealScalar l)
{
  const RealScalar vl = norm();
  if(vl > 1e-9)
  {
    derived() *= (l / vl);
  }
  return derived();
}

inline PlainObject normalized(RealScalar l) const
{
  return PlainObject(derived()).normalize(l);
}

inline PlainObject& mirror()
{
  //mirror is only defined for vectors with length 2
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
  derived().x() = -derived().x();
  derived().y() = -derived().y();
  return derived();
}

inline PlainObject& rotateLeft()
{
  //rotateLeft() is only defined for vectors with length 2
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
  Scalar buffer = -derived().y();
  derived().y() = derived().x();
  derived().x() = buffer;
  return derived();
}

inline PlainObject& rotateRight()
{
  //rotateRight() is only defined for vectors with length 2
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
  Scalar buffer = -derived().x();
  derived().x() = derived().y();
  derived().y() = buffer;
  return derived();
}

inline PlainObject& rotate(const float alpha)
{
  //angle() is only defined for vectors with length 2
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
  const float buffer = static_cast<float>(derived().x());
  const float a = std::cos(alpha);
  const float b = std::sin(alpha);
  derived().x() = static_cast<Scalar>(a * static_cast<float>(derived().x()) - b * static_cast<float>(derived().y()));
  derived().y() = static_cast<Scalar>(b * buffer + a * static_cast<float>(derived().y()));
  return derived();
}

inline PlainObject rotated(const float alpha) const
{
  return PlainObject(derived()).rotate(alpha);
}

inline RealScalar angle() const
{
  //angle() is only defined for vectors with length 2
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 2);
  const RealScalar x = derived().x();
  const RealScalar y = derived().y();
  return std::atan2(y, x);
}

/**
 * Calculation of the angle between this vector and the other one
 * @param other The other vector that creates the angle with this one
 * @return the angle in range of [0;pi]
 */
RealScalar angleTo(const MatrixBase<Derived>& other) const
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
  const RealScalar normProd = derived().norm() * other.norm();
  ASSERT(normProd != 0.f);
  return normProd == RealScalar(0.f) ? RealScalar(0.f) : std::acos(derived().dot(other) / normProd);
}
