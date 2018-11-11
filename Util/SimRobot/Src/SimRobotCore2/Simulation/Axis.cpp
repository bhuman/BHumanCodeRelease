/**
* @file Simulation/Axis.cpp
* Implementation of class Axis
* @author Colin Graf
*/

#include <cmath>

#include "Simulation/Axis.h"
#include "Simulation/Actuators/Joint.h"
#include "Simulation/Motors/Motor.h"
#include "Platform/Assert.h"

Axis::~Axis()
{
  if(deflection)
    delete deflection;
  if(motor)
    delete motor;
}

void Axis::create()
{
  // normalize axis
  const float len = std::sqrt(x * x + y * y + z * z);
  if(len == float())
    x = 1.f;
  else
  {
    const float invLen = 1.f / len;
    x *= invLen;
    y *= invLen;
    z *= invLen;
  }
}

void Axis::addParent(Element& element)
{
  joint = dynamic_cast<Joint*>(&element);
  ASSERT(!joint->axis);
  joint->axis = this;
}
