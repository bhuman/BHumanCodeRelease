/**
* @file Simulation/Geometries/BoxGeometry.cpp
* Implementation of class BoxGeometry
* @author Colin Graf
*/

#include "Platform/OpenGL.h"
#include "Simulation/Geometries/BoxGeometry.h"
#include <cmath>
#include <algorithm>

dGeomID BoxGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  innerRadius = std::min(std::min(depth, width), height) * 0.5f;
  innerRadiusSqr = innerRadius * innerRadius;
  outerRadius = std::sqrt(depth * depth * 0.25f + width * width * 0.25f + height * height * 0.25f);
  return dCreateBox(space, depth, width, height);
}

void BoxGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    const float lx = depth * 0.5f;
    const float ly = width * 0.5f;
    const float lz = height * 0.5f;

    // -y-side
    glBegin(GL_TRIANGLE_FAN);
      glColor4fv(color);
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
  }

  ::PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
