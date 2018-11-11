/**
* @file Simulation/Joints/Joint.cpp
* Implementation of class Joint
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
* @author <A href="mailto:kspiess@informatik.uni-bremen.de">Kai Spiess</A>
* @author Colin Graf
* @author Thomas Röfer
*/

#include <cmath>
#include "Platform/OpenGL.h"

#include "Joint.h"
#include "Simulation/Axis.h"
#include "Simulation/Simulation.h"
#include "Simulation/Motors/Motor.h"
#include "CoreModule.h"
#include "Tools/OpenGLTools.h"

Joint::~Joint()
{
  if(joint)
    dJointDestroy(joint);
}

void Joint::drawPhysics(unsigned int flags) const
{
  glPushMatrix();
  glMultMatrixf(transformation);

  if(flags & SimRobotCore2::Renderer::showPhysics)
  {
    glBegin(GL_LINES);
      glNormal3f(0, 0, 1.f);
      glColor3f(std::abs(axis->x), std::abs(axis->y), std::abs(axis->z));
      glVertex3f(axis->x * -0.05f, axis->y * -0.05f, axis->z * -0.05f);
      glVertex3f(axis->x * 0.05f, axis->y * 0.05f, axis->z * 0.05f);
    glEnd();

    GLUquadricObj* q = gluNewQuadric();
    gluSphere(q, 0.002, 10, 10);
    gluDeleteQuadric(q);
  }

  ::PhysicalObject::drawPhysics(flags);

  glPopMatrix();
}

void Joint::registerObjects()
{
  // add sensors and actuators
  if(axis->motor)
    axis->motor->registerObjects();

  // add children
  ::PhysicalObject::registerObjects();
}
