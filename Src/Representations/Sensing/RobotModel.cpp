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

  for(unsigned side = 0; side < 2; side++)
  {
    const bool left = side == 0;
    ForwardKinematic::calculateArmChain(Arms::Arm(side), jointAngles, robotDimensions, limbs);
    ForwardKinematic::calculateLegChain(left, jointAngles, robotDimensions, limbs);
  }

  // calculate center of mass
  centerOfMass = Vector3f::Zero();
  totalMass = 0.0;
  for(int i = 0; i < Limbs::numOfLimbs; i++)
  {
    const MassCalibration::MassInfo& limb(massCalibration.masses[i]);
    totalMass += limb.mass;
    centerOfMass += (limbs[i] * limb.offset) * limb.mass;
  }
  centerOfMass /= totalMass;
}

void RobotModel::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel", "robot");
  COMPLEX_DRAWING3D("representation:RobotModel")
  {
    for(int i = 0; i < Limbs::numOfLimbs; ++i)
      SUBCOORDINATES3D("representation:RobotModel", limbs[i], 50, 1);
  }
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel:centerOfMass", "robot");
  SPHERE3D("representation:RobotModel:centerOfMass", centerOfMass.x(), centerOfMass.y(), centerOfMass.z(), 10, ColorRGBA::red);

  DECLARE_PLOT("representation:RobotModel:centerOfMassX");
  DECLARE_PLOT("representation:RobotModel:centerOfMassY");
  DECLARE_PLOT("representation:RobotModel:centerOfMassZ");
  PLOT("representation:RobotModel:centerOfMassX", centerOfMass.x());
  PLOT("representation:RobotModel:centerOfMassY", centerOfMass.y());
  PLOT("representation:RobotModel:centerOfMassZ", centerOfMass.z());
}
