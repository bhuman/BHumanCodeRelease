/**
 * @file BodyDynamics.h
 * Implementation of BodyDynamics methods.
 * @author Felix Wenk
 */

#include "BodyDynamics.h"

void BodyDynamics::init(const MassCalibration &massCalibration)
{
  for(int i = 0; i < MassCalibration::numOfLimbs; ++i)
  {
    const MassCalibration::MassInfo& m = massCalibration.masses[i];
    limbs[i].I = SpatialInertia(m.inertiaMatrix, m.offset, m.mass);
  }
  inertiasInitialized = true;
}
