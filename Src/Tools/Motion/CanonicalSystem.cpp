#include "CanonicalSystem.h"
#include <cmath>
#include "Platform/BHAssert.h"

CanonicalSystem::CanonicalSystem(const float executionTime, const float startingPhase,
                                 const float finalPhaseValue) :
  z(startingPhase), T(executionTime), alpha(-logf(finalPhaseValue))
{
  ASSERT(T > 0);
  ASSERT(alpha != 0);
}
float CanonicalSystem::step(const float dt)
{
  z += -alpha * z * dt / T;
  currentTime += dt;
  return z;
}

void CanonicalSystem::reset()
{
  z = 1.0f;
  currentTime = 0.0f;
}

float CanonicalSystem::getPhaseAt(const float time) const
{
  return expf(-alpha / T * time);
}
