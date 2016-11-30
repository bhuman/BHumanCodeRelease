#include "LIP.h"
#include "Platform/BHAssert.h"
#include "Tools/Math/BHMath.h"

void LIP::setLIPHeight(float LIPHeight)
{
  ASSERT(LIPHeight != 0.f);
  height = LIPHeight;
  k = std::sqrt(Constants::g / height);
}

LIP& LIP::update(float timePassed, float zmp)
{
  const float sinhkt = std::sinh(k * timePassed);
  const float coshkt = std::cosh(k * timePassed);

  const float newPosition = (position - zmp) * coshkt + (velocity / k) * sinhkt + zmp;
  velocity = (position - zmp) * k * sinhkt + velocity * coshkt;
  position = newPosition;
  return *this;
}

float LIP::energy(float zmp) const
{
  return 0.5f * (sqr(velocity) - k * k * sqr(position - zmp));
}

float LIP::energy(float pos, float vel, float comHeight)
{
  return 0.5f * (sqr(vel) - (Constants::g / comHeight) * sqr(pos));
}

float LIP::requiredPositionForPosition(float pos, float time, float zmp) const
{
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return zmp + (pos - zmp - velocity * sinhkt / k) / coshkt;
}

float LIP::requiredVelocityForPosition(float pos, float time, float zmp) const
{
  ASSERT(time != 0.f);
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return k * ((pos - zmp) - (position - zmp) * coshkt) / sinhkt;
}

float LIP::requiredVelocityForVelocity(float vel, float time, float zmp) const
{
  const float sinhkt = std::sinh(k * time);
  const float coshkt = std::cosh(k * time);

  return (vel - (position - zmp) * k * sinhkt) / coshkt;
}

float LIP::requiredZMPForPosition(float pos, float time) const
{
  const float kT = k * time;
  const float a = std::exp(kT);
  const float b = std::exp(-kT);

  // This formula produces bullshit at the end of a step, although it is just a conversion from the formula in exponentil form...
  //return (position * std::cosh(kT) + (velocity / k) * std::cosh(kT) - pos) / (std::cosh(kT) - 1.f);

  return (position * (a + b) + (velocity / k) * (a - b) - 2.f * pos) / (a + b - 2.f);
}

float LIP::requiredZMPForVelocity(float vel, float time) const
{
  ASSERT(time != 0.f);
  const float kT = k * time;
  const float a = std::exp(kT);
  const float b = std::exp(-kT);

  return (velocity * (a + b) * 0.5f - vel) / (k * (a - b) * 0.5f) + position;
}

float LIP::timeToPosition(float pos, float zmp) const
{
  const float a = position - zmp + velocity / k;
  const float b = position - zmp - velocity / k;
  const float p_z = pos - zmp;

  const float c = std::sqrt(sqr(p_z) - a * b);

  const float res1 = std::log((p_z + c) / a) / k;
  const float res2 = std::log((p_z - c) / a) / k;

  if(!std::isfinite(res1))
    return res2;
  else if(!std::isfinite(res2))
    return res1;
  else
    return res1 > res2 ? res1 : res2;
}

float LIP::timeToVelocity(float vel, float zmp) const
{
  const float a = position - zmp + velocity / k;
  const float b = position - zmp - velocity / k;

  const float c = std::sqrt(sqr(vel) / sqr(k) + a * b);

  const float res1 = std::log((vel / k + c) / a) / k;
  const float res2 = std::log((vel / k - c) / a) / k;

  if(!std::isfinite(res1))
    return res2;
  else if(!std::isfinite(res2))
    return res1;
  else
    return res1 > res2 ? res1 : res2;
}
