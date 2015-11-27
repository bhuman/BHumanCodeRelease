/**
 * File:   PT2.cpp
 * Author: arne
 *
 * Created on October 31, 2014, 3:26 PM
 */

#include "PT2.h"
#include <cmath>
#include "Platform/BHAssert.h"

PT2::PT2(float T, float D, float K, float initialPosition,
         float initialVelocity, float maxVelocity) :
  x(T * initialVelocity),
  y(initialPosition),
  yd(initialVelocity),
  T(T), D(D), K(K),
  V(maxVelocity)
{}

void PT2::initialize(const float initialPosition, const float initialVelocity)
{
  y = initialPosition;
  yd = initialVelocity;
  x = T * initialVelocity;
  initialized = true;
}

void PT2::step(const float goal, const float dt)
{
  //this difference equation has been derived from the pt2 differential equation
  // see: http://webber.physik.uni-freiburg.de/~hon/vorlss02/Literatur/Ingenieurswiss/Regelungstechnik/ProgrammiereRegelung.pdf
  //      exercise 1.2 (german only, sorry)

  /**equivalent python code
    dy = dt / self.T * self.x # == dt * x / T
    dy = min(dy, dt * self.ydMax)
    dx = dt / self.T * (self.K * e - self.y - 2 * self.D * self.x)
    self.x += dx
    #limit self.x to ydMax by converting x to velocity, limiting it and converting back
    current_velocity = min(self.x / self.T, self.ydMax)
    self.x = current_velocity * self.T
    self.y += dy
    return self.y
   */

  ASSERT(T != 0.0);
  const float maxVel = dt * V; //max allowed velocity for this step
  const float oldYd = yd;
  const float dy = std::fmin(dt / T * x, maxVel);
  const float dx = dt / T * (K * goal - y - 2 * D * x);
  x += dx;
  yd = std::fmin(x / T, V);//convert x to velocity and limit it
  x = yd * T;//convert limited velocity back to x to limit x as well
  y += dy;
  ydd = (yd - oldYd) / dt;
}

bool PT2::isInitialized() const
{
  return initialized;
}

float PT2::getPosition() const
{
  return y;
}

float PT2::getVelocity() const
{
  return yd;
}

float PT2::getAcceleration() const
{
  return ydd;
}

bool PT2::isValid() const
{
  //The only parameter that has any restrictions is T
  return T != 0.0f;
}
