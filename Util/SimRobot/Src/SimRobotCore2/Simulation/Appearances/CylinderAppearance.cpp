/**
* @file Simulation/Appearances/CylinderAppearance.cpp
* Implementation of class CylinderAppearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/CylinderAppearance.h"

void CylinderAppearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set(color);

  GLUquadricObj* q = gluNewQuadric();
  glTranslatef(0.f, 0.f, height * -0.5f);
  gluCylinder(q, radius, radius, height, 16, 1);
  glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
  gluDisk(q, 0, radius, 16, 1);
  glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
  glTranslatef(0,0,height);
  gluDisk(q, 0, radius, 16, 1);
  glTranslatef(0.f, 0.f, height * -0.5f);
  gluDeleteQuadric(q);

  surface->unset();

  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}
