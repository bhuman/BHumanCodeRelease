#pragma once

#include "LIP.h"
#include "Tools/Math/Eigen.h"

class LIP3D
{
public:
  Vector2f position = Vector2f::Zero();
  Vector2f velocity = Vector2f::Zero();

private:
  Array2f k = Array2f::Zero();
  Array2f heights = Array2f::Zero();

public:
  LIP3D(const Array2f& LIPHeights);
  LIP3D(const Vector2f& position, const Vector2f& velocity, const Array2f& LIPHeights);
  LIP3D(const LIP3D& other);

  LIP3D& operator=(const LIP3D& other);

  void setLIPHeights(const Array2f& LIPHeights);
  const Array2f& getLIPHeights() const { return heights; };
  const Array2f& getK() const { return k; };

  LIP3D predict(float time, const Vector2f& zmp = Vector2f::Zero()) const;
  LIP3D& update(float time, const Vector2f& zmp = Vector2f::Zero());

  Vector2f requiredVelocity(const Vector2f& pos, float time);

  LIP getXLIP() const;
  LIP getYLIP() const;
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

inline LIP3D LIP3D::predict(float time, const Vector2f& zmp) const
{
  return LIP3D(*this).update(time, zmp);
}

inline LIP LIP3D::getXLIP() const
{
  return LIP(position.x(), velocity.x(), heights.x());
}

inline LIP LIP3D::getYLIP() const
{
  return LIP(position.y(), velocity.y(), heights.y());
}
