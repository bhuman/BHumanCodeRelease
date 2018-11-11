/**
* @file Simulation/Joints/Slider.cpp
* Implementation of class Slider
* @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
* @author <A href="mailto:kspiess@informatik.uni-bremen.de">Kai Spiess</A>
* @author Colin Graf
* @author Thomas Röfer
*/

#include <cmath>
#include "Platform/OpenGL.h"

#include "Slider.h"
#include "Simulation/Body.h"
#include "Simulation/Axis.h"
#include "Simulation/Simulation.h"
#include "Simulation/Motors/ServoMotor.h"
#include "Platform/Assert.h"
#include "CoreModule.h"
#include "Tools/OpenGLTools.h"

void Slider::createPhysics()
{
  ASSERT(axis);

  //
  axis->create();

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
  joint = dJointCreateSlider(Simulation::simulation->physicalWorld, 0);
  dJointAttach(joint, childBody->body, parentBody ? parentBody->body : 0);
  //set Slider joint parameter
  const Vector3f globalAxis = pose.rotation * Vector3f(axis->x, axis->y, axis->z);
  dJointSetSliderAxis(joint, globalAxis.x(), globalAxis.y(), globalAxis.z());
  if(axis->cfm != -1.f)
    dJointSetSliderParam(joint, dParamCFM, axis->cfm);

  if(axis->deflection)
  {
    const Axis::Deflection& deflection = *axis->deflection;
    float minSliderLimit = deflection.min;
    float maxSliderLimit = deflection.max;
    if(minSliderLimit > maxSliderLimit)
      minSliderLimit = maxSliderLimit;
    //Set physical limits to higher values (+10%) to avoid strange Slider effects.
    //Otherwise, sometimes the motor exceeds the limits.
    float internalTolerance = (maxSliderLimit - minSliderLimit) * 0.1f;
    if(dynamic_cast<ServoMotor*>(axis->motor))
    {
      minSliderLimit -= internalTolerance;
      maxSliderLimit += internalTolerance;
    }
    dJointSetSliderParam(joint, dParamLoStop, minSliderLimit);
    dJointSetSliderParam(joint, dParamHiStop, maxSliderLimit);
    // this has to be done due to the way ODE sets joint stops
    dJointSetSliderParam(joint, dParamLoStop, minSliderLimit);
    if(deflection.stopCFM != -1.f)
      dJointSetSliderParam(joint, dParamStopCFM, deflection.stopCFM);
    if(deflection.stopERP != -1.f)
      dJointSetSliderParam(joint, dParamStopERP, deflection.stopERP);
  }

  // create motor
  if(axis->motor)
    axis->motor->create(this);

  OpenGLTools::convertTransformation(rotation, translation, transformation);
}

const QIcon* Slider::getIcon() const
{
  return &CoreModule::module->sliderIcon;
}
