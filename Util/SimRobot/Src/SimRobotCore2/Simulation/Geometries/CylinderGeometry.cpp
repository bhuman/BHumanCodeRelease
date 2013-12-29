/**
* @file Simulation/Geometries/CylinderGeometry.cpp
* Implementation of class CylinderGeometry
* @author Colin Graf
*/

#include "Platform/OpenGL.h"
#include "Simulation/Geometries/CylinderGeometry.h"
#include <cmath>

dGeomID CylinderGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  innerRadius = radius;
  innerRadiusSqr = innerRadius * innerRadius;
  outerRadius = std::sqrt(height * height * 0.25f + radius * radius);
  return dCreateCylinder(space, radius, height);
}

void CylinderGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    glColor4fv(color);
    GLUquadricObj* q = gluNewQuadric();
    glTranslatef(0.f, 0.f, height * -0.5f);
    gluCylinder(q, radius, radius, height, 16, 1);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    gluDisk(q, 0, radius, 16, 1);
    glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
    glTranslatef(0,0,height);
    gluDisk(q, 0, radius, 16, 1);
    gluDeleteQuadric(q);
  }

  ::PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
