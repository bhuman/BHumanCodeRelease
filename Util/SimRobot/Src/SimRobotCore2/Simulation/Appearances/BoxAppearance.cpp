/**
* @file Simulation/Appearances/BoxAppearance.cpp
* Implementation of class BoxAppearance
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Appearances/BoxAppearance.h"

void BoxAppearance::assembleAppearances(SurfaceColor color) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  surface->set(color);

  float lx = depth * 0.5f;
  float ly = width * 0.5f;
  float lz = height * 0.5f;

  // -y-side
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, -1, 0);
    glVertex3f(lx, -ly, -lz);
    glVertex3f(lx, -ly, lz);
    glVertex3f(-lx, -ly, lz);
    glVertex3f(-lx, -ly, -lz);
  glEnd();

  // y-side
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 1, 0);
    glVertex3f(-lx, ly, lz);
    glVertex3f(lx, ly, lz);
    glVertex3f(lx, ly, -lz);
    glVertex3f(-lx, ly, -lz);
  glEnd();

  // -x-side
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-1, 0, 0);
    glVertex3f(-lx, -ly, -lz);
    glVertex3f(-lx, -ly, lz);
    glVertex3f(-lx, ly, lz);
    glVertex3f(-lx, ly, -lz);
  glEnd();

  // x-side
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(1, 0, 0);
    glVertex3f(lx, -ly, -lz);
    glVertex3f(lx, ly, -lz);
    glVertex3f(lx, ly, lz);
    glVertex3f(lx, -ly, lz);
  glEnd();

  // bottom
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 0, -1);
    glVertex3f(-lx, -ly, -lz);
    glVertex3f(-lx, ly, -lz);
    glVertex3f(lx, ly, -lz);
    glVertex3f(lx, -ly, -lz);
  glEnd();

  // top
  glBegin(GL_TRIANGLE_FAN);
    glNormal3f(0, 0, 1);
    glVertex3f(-lx, -ly, lz);
    glVertex3f(lx, -ly, lz);
    glVertex3f(lx, ly, lz);
    glVertex3f(-lx, ly, lz);
  glEnd();

  surface->unset();

  GraphicalObject::assembleAppearances(color);
  glPopMatrix();
}
