/**
 * @file RobotModel.cpp
 * Implementation of struct RobotModel.
 * @author Alexander Härtl
 */

#include "RobotModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Motion/ForwardKinematic.h"

RobotModel::RobotModel(const JointAngles& jointAngles, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  setJointData(jointAngles, robotDimensions, massCalibration);
}

void RobotModel::setJointData(const JointAngles& jointAngles, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  ForwardKinematic::calculateHeadChain(jointAngles, robotDimensions, limbs);
  ForwardKinematic::calculateArmChain(Arms::left, jointAngles, robotDimensions, limbs);
  ForwardKinematic::calculateArmChain(Arms::right, jointAngles, robotDimensions, limbs);
  ForwardKinematic::calculateLegChain(Legs::left, jointAngles, robotDimensions, limbs);
  ForwardKinematic::calculateLegChain(Legs::right, jointAngles, robotDimensions, limbs);

  soleLeft = limbs[Limbs::footLeft] + Vector3f(0.f, 0.f, -robotDimensions.footHeight);
  soleRight = limbs[Limbs::footRight] + Vector3f(0.f, 0.f, -robotDimensions.footHeight);

  updateCenterOfMass(massCalibration);
}

void RobotModel::updateCenterOfMass(const MassCalibration& massCalibration)
{
  // calculate center of mass
  centerOfMass = Vector3f::Zero();
  for(int i = 0; i < Limbs::numOfLimbs; i++)
  {
    const MassCalibration::MassInfo& limb = massCalibration.masses[i];
    centerOfMass += (limbs[i] * limb.offset) * limb.mass;
  }
  centerOfMass /= massCalibration.totalMass;
}

void RobotModel::draw() const
{
  DEBUG_DRAWING3D("representation:RobotModel", "robot")
  {
    for(int i = 0; i < Limbs::numOfLimbs; ++i)
      SUBCOORDINATES3D("representation:RobotModel", limbs[i], 50, 1);
  }
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel:centerOfMass", "robot");
  SPHERE3D("representation:RobotModel:centerOfMass", centerOfMass.x(), centerOfMass.y(), centerOfMass.z(), 10, ColorRGBA::red);

  PLOT("representation:RobotModel:centerOfMass:x", centerOfMass.x());
  PLOT("representation:RobotModel:centerOfMass:x", centerOfMass.y());
  PLOT("representation:RobotModel:centerOfMass:z", centerOfMass.z());
}
