#pragma once

#include <cmath>
#include "Tools/Math/Constants.h"

class LIP
{
public:
  float position = 0.f;
  float velocity = 0.f;

private:
  float k = 0.f; // k = sqrt(g / h)
  float height = 0.f;

public:
  LIP(float LIPHeight);
  LIP(float pos, float vel, float LIPHeight);
  LIP(const LIP& other);

  void setLIPHeight(float LIPHeight);
  float getLIPHeight() const { return height; };
  float getK() const { return k; };

  LIP predict(float time, float zmp = 0.f) const;
  LIP& update(float time, float zmp = 0.f);

  float energy(float zmp = 0.f) const;
  static float energy(float pos, float vel, float LIPHeight);
  float requiredPositionForPosition(float pos, float time, float zmp = 0.f) const;
  float requiredVelocityForPosition(float pos, float time, float zmp = 0.f) const;
  float requiredVelocityForVelocity(float vel, float time, float zmp = 0.f) const;
  float requiredZMPForPosition(float pos, float time) const;
  float requiredZMPForVelocity(float vel, float time) const;
  float timeToPosition(float pos, float zmp = 0.f) const;
  float timeToVelocity(float vel, float zmp = 0.f) const;
};

inline LIP::LIP(float LIPHeight)
{
  setLIPHeight(LIPHeight);
}

inline LIP::LIP(float position, float vel, float LIPHeight) :
  position(position), velocity(vel)
{
  setLIPHeight(LIPHeight);
}

inline LIP::LIP(const LIP& other) :
  position(other.position), velocity(other.velocity)
{
  setLIPHeight(other.height);
}

inline LIP LIP::predict(float time, float zmp) const
{
  return LIP(*this).update(time, zmp);
}
