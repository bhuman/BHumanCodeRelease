/**
* @file Simulation/Joints/Hinge.cpp
* Implementation of class Hinge
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
* @author <A href="mailto:kspiess@informatik.uni-bremen.de">Kai Spiess</A>
* @author Colin Graf
*/

#include <cmath>
#include "Platform/OpenGL.h"

#include "Hinge.h"
#include "Simulation/Body.h"
#include "Simulation/Axis.h"
#include "Simulation/Simulation.h"
#include "Simulation/Motors/ServoMotor.h"
#include "Platform/Assert.h"
#include "CoreModule.h"
#include "Tools/OpenGLTools.h"

void Hinge::createPhysics()
{
  ASSERT(axis);

  //
  axis->create();

  if(axis->deflection && axis->deflection->offset != 0.f)
    pose.rotate(Matrix3x3<>(Vector3<>(axis->x, axis->y, axis->z) * axis->deflection->offset));

  //
  ::PhysicalObject::createPhysics();

  // find bodies to connect
  Body* parentBody = dynamic_cast<Body*>(parent);
  ASSERT(!parentBody || parentBody->body);
  ASSERT(!children.empty());
  Body* childBody = dynamic_cast<Body*>(*children.begin());
  ASSERT(childBody);
  ASSERT(childBody->body);

  // create joint
  joint = dJointCreateHinge(Simulation::simulation->physicalWorld, 0);
  dJointAttach(joint, childBody->body, parentBody ? parentBody->body : 0);
  //set hinge joint parameter
  dJointSetHingeAnchor(joint, pose.translation.x, pose.translation.y, pose.translation.z);
  Vector3<> globalAxis = pose.rotation * Vector3<>(axis->x, axis->y, axis->z);
  dJointSetHingeAxis(joint, globalAxis.x, globalAxis.y, globalAxis.z);
  if(axis->cfm != -1.f)
    dJointSetHingeParam(joint, dParamCFM, dReal(axis->cfm));

  if(axis->deflection)
  {
    const Axis::Deflection& deflection = *axis->deflection;
    float minHingeLimit = deflection.min;
    float maxHingeLimit = deflection.max;
    if(minHingeLimit > maxHingeLimit)
      minHingeLimit = maxHingeLimit;
    //Set physical limits to higher values (+10%) to avoid strange hinge effects.
    //Otherwise, sometimes the motor exceeds the limits.
    float internalTolerance = (maxHingeLimit - minHingeLimit) * 0.1f;
    if(dynamic_cast<ServoMotor*>(axis->motor))
    {
      minHingeLimit -= internalTolerance;
      maxHingeLimit += internalTolerance;
    }
    dJointSetHingeParam(joint, dParamLoStop, dReal(minHingeLimit - axis->deflection->offset));
    dJointSetHingeParam(joint, dParamHiStop, dReal(maxHingeLimit - axis->deflection->offset));
    // this has to be done due to the way ODE sets joint stops
    dJointSetHingeParam(joint, dParamLoStop, dReal(minHingeLimit - axis->deflection->offset));
    if(deflection.stopCFM != -1.f)
      dJointSetHingeParam(joint, dParamStopCFM, dReal(deflection.stopCFM));
    if(deflection.stopERP != -1.f)
      dJointSetHingeParam(joint, dParamStopERP, dReal(deflection.stopERP));
  }

  // create motor
  if(axis->motor)
  {
    axis->motor->create(this);
    if(axis->deflection) // Move setpoint to a position inside the deflection range
      axis->motor->setpoint = axis->deflection->offset;
  }

  OpenGLTools::convertTransformation(rotation, translation, transformation);
}

const QIcon* Hinge::getIcon() const
{
  return &CoreModule::module->hingeIcon;
}
