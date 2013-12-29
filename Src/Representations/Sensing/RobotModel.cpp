/**
* @file RobotModel.cpp
* Implementation of class RobotModel.
* @author Alexander Härtl
*/

#include "RobotModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/ForwardKinematic.h"

RobotModel::RobotModel(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  setJointData(joints, robotDimensions, massCalibration);
}

void RobotModel::setJointData(const JointData& joints, const RobotDimensions& robotDimensions, const MassCalibration& massCalibration)
{
  ForwardKinematic::calculateHeadChain(joints, robotDimensions, massCalibration, limbs);

  for(int side = 0; side < 2; side++)
  {
    const bool left = side == 0;
    ForwardKinematic::calculateArmChain(left, joints, robotDimensions, massCalibration, limbs);
    ForwardKinematic::calculateLegChain(left, joints, robotDimensions, massCalibration, limbs);
  }

  // calculate center of mass
  centerOfMass = Vector3<>();
  totalMass = 0.0;
  for(int i = 0; i < MassCalibration::numOfLimbs; i++)
  {
    const MassCalibration::MassInfo& limb(massCalibration.masses[i]);
    totalMass += limb.mass;
    centerOfMass += (limbs[i] * limb.offset) * limb.mass;
  }
  centerOfMass /= totalMass;
}

void RobotModel::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel", "origin");
  COMPLEX_DRAWING3D("representation:RobotModel",
  {
    for(int i = 0; i < MassCalibration::numOfLimbs; ++i)
    {
      const Pose3D& p = limbs[i];
      const Vector3<>& v = p.translation;
      const Vector3<> v1 = p * Vector3<>(50, 0, 0);
      const Vector3<> v2 = p * Vector3<>(0, 50, 0);
      const Vector3<> v3 = p * Vector3<>(0, 0, 50);
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v1.x, v1.y, v1.z, 1, ColorRGBA(255, 0, 0));
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v2.x, v2.y, v2.z, 1, ColorRGBA(0, 255, 0));
      LINE3D("representation:RobotModel", v.x, v.y, v.z, v3.x, v3.y, v3.z, 1, ColorRGBA(0, 0, 255));
    }
  });
  DECLARE_DEBUG_DRAWING3D("representation:RobotModel:centerOfMass", "origin");
  SPHERE3D("representation:RobotModel:centerOfMass", centerOfMass.x, centerOfMass.y, centerOfMass.z, 25, ColorRGBA(255, 0, 0));
}
