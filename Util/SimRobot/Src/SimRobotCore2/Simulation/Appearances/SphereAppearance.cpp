/**
* @file Simulation/Appearances/SphereAppearance.cpp
* Implementation of class SphereAppearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/SphereAppearance.h"

void SphereAppearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set(color, false);

  GLUquadricObj* q = gluNewQuadric();
  gluQuadricNormals(q, GLU_SMOOTH);
  gluQuadricTexture(q, GL_TRUE);
  gluSphere(q, radius, 16, 16);
  gluDeleteQuadric(q);

  surface->unset(false);

  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}
