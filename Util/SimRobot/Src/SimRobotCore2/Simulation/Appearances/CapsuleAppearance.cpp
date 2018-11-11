/**
* @file Simulation/Appearances/CapsuleAppearance.cpp
* Implementation of class CapsuleAppearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/CapsuleAppearance.h"

void CapsuleAppearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set(color);

  GLUquadricObj* q = gluNewQuadric();
  float cylinderHeight = height - radius - radius;
  glTranslatef(0.f, 0.f, cylinderHeight * -0.5f);
  gluCylinder(q, radius, radius, cylinderHeight, 16, 1);
  gluSphere(q, radius, 16, 16);
  glTranslatef(0, 0, cylinderHeight);
  gluSphere(q, radius, 16, 16);
  gluDeleteQuadric(q);

  surface->unset();

  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}
