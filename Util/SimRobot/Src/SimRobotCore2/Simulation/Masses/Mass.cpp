/**
* @file Simulation/Masses/Mass.cpp
* Implementation of class Mass
* @author Colin Graf
*/

#include "Simulation/Masses/Mass.h"
#include "Platform/Assert.h"
#include "Tools/ODETools.h"

const dMass& Mass::createMass()
{
  if(!created)
  {
    assembleMass();
    for(std::list<SimObject*>::const_iterator iter = children.begin(), end = children.end(); iter != end; ++iter)
    {
      Mass* childMassDesc = dynamic_cast<Mass*>(*iter);
      ASSERT(childMassDesc);
      const dMass& childMass = childMassDesc->createMass();
      if(childMassDesc->translation || childMassDesc->rotation)
      {
        dMass shiftedChildMass = childMass;
        if(childMassDesc->rotation)
        {
          dMatrix3 matrix;
          ODETools::convertMatrix(*childMassDesc->rotation, matrix);
          dMassRotate(&shiftedChildMass, matrix);
        }
        if(childMassDesc->translation)
          dMassTranslate(&shiftedChildMass, childMassDesc->translation->x(), childMassDesc->translation->y(), childMassDesc->translation->z());
        dMassAdd(&mass, &shiftedChildMass);
      }
      else
        dMassAdd(&mass, &childMass);
    }
    created = true;
  }
  return mass;
}

void Mass::assembleMass()
{
  dMassSetZero(&mass);
}
