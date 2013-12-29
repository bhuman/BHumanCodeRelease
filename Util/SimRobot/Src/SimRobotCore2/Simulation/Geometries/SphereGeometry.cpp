/**
* @file Simulation/Geometries/SphereGeometry.cpp
* Implementation of class SphereGeometry
* @author Colin Graf
*/

#include "Platform/OpenGL.h"

#include "Simulation/Geometries/SphereGeometry.h"

dGeomID SphereGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  innerRadius = radius;
  innerRadiusSqr = innerRadius * innerRadius;
  outerRadius = radius;
  return dCreateSphere(space, radius);
}

void SphereGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    glColor4fv(color);
    GLUquadricObj* q = gluNewQuadric();
    gluSphere(q, radius, 16, 16);
    gluDeleteQuadric(q);
  }

  ::PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
