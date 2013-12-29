/**
* @file Simulation/Geometries/CapsuleGeometry.cpp
* Implementation of class CapsuleGeometry
* @author Colin Graf
*/

#include <algorithm>

#include "Platform/OpenGL.h"

#include "Simulation/Geometries/CapsuleGeometry.h"

dGeomID CapsuleGeometry::createGeometry(dSpaceID space)
{
  Geometry::createGeometry(space);
  innerRadius = radius;
  innerRadiusSqr = innerRadius * innerRadius;
  outerRadius = std::max(radius, height * 0.5f);
  return dCreateCapsule(space, radius, height - radius - radius);
}

void CapsuleGeometry::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    glColor4fv(color);
    GLUquadricObj* q = gluNewQuadric();
    const float cylinderHeight = height - radius - radius;
    glTranslatef(0.f, 0.f, cylinderHeight * -0.5f);
    gluCylinder(q, radius, radius, cylinderHeight, 16, 1);
    gluSphere(q, radius, 16, 16);
    glTranslatef(0, 0, cylinderHeight);
    gluSphere(q, radius, 16, 16);
    gluDeleteQuadric(q);
  }

  ::PhysicalObject::drawPhysics(flags);
  glPopMatrix();
}
