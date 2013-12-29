/**
* @file Simulation/Appearances/SphereAppearance.cpp
* Implementation of class SphereAppearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/SphereAppearance.h"

void SphereAppearance::assembleAppearances() const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set();

  GLUquadricObj* q = gluNewQuadric();
  gluSphere(q, radius, 16, 16);
  gluDeleteQuadric(q);

  surface->unset();

  GraphicalObject::assembleAppearances();
  glPopMatrix();
}
