/**
* @file RobotModelProvider.cpp
*
* This file implements a module that provides information about the current state of the robot's limbs.
*
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#include "RobotModelProvider.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"


void RobotModelProvider::update(RobotModel& robotModel)
{
  robotModel.setJointData(theFilteredJointData, theRobotDimensions, theMassCalibration);

  DECLARE_DEBUG_DRAWING3D("module:RobotModelProvider:massOffsets", "origin",
  {
    for(int i = 0; i < MassCalibration::numOfLimbs; ++i)
    {
      const Vector3<>& v(robotModel.limbs[i] * theMassCalibration.masses[i].offset);
      SPHERE3D("module:RobotModelProvider:massOffsets", v.x, v.y, v.z, 3, ColorRGBA(0, 200, 0));
    }
  });

  DECLARE_DEBUG_DRAWING3D("module:RobotModelProvider:joints", "origin",
  {
    const float axisLineWidth = 1.f;

    for(int i = 0; i < 2; ++i)
    {
      int firstJoint = i == 0 ? MassCalibration::pelvisLeft : MassCalibration::pelvisRight;
      for(int i = 0; i < (MassCalibration::footLeft + 1) - MassCalibration::pelvisLeft; ++i)
      {
        Pose3D& next = robotModel.limbs[firstJoint + i];
        SPHERE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

        Vector3<> axis  = i == 0 ? Vector3<>(0.f, firstJoint == MassCalibration::pelvisLeft ? -50.f : 50.f, 50.f).normalize(50.f) : i == 1 || i == 5 ? Vector3<>(50.f, 0.f, 0.f) : Vector3<>(0.f, 50.f, 0.f);
        ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : i == 1 || i == 5 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
        Vector3<> p = next * axis;
        LINE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);
      }
    }
    for(int i = 0; i < 2; ++i)
    {
      int firstJoint = i == 0 ? MassCalibration::shoulderLeft : MassCalibration::shoulderRight;
      for(int i = 0; i < (MassCalibration::foreArmLeft + 1) - MassCalibration::shoulderLeft; ++i)
      {
        Pose3D& next = robotModel.limbs[firstJoint + i];
        SPHERE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

        Vector3<> axis  = i == 1 || i == 3 ? Vector3<>(0.f, 0.f, 50.f) : i == 2 ? Vector3<>(50.f, 0.f, 0.f) : Vector3<>(0.f, 50.f, 0.f);
        ColorRGBA color = i == 1 || i == 3 ? ColorRGBA(0, 0, 255)      : i == 2 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
        Vector3<> p = next * axis;
        LINE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);
      }
    }
    for(int i = 0; i < (MassCalibration::head + 1) - MassCalibration::neck; ++i)
    {
      Pose3D& next = robotModel.limbs[MassCalibration::neck + i];
      SPHERE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, 3, ColorRGBA(0, 0, 0));

      Vector3<> axis  = i == 0 ? Vector3<>(0.f, 0.f, 50.f) : Vector3<>(0.f, 50.f, 0.f);
      ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : ColorRGBA(0, 255, 0);
      Vector3<> p = next * axis;
      LINE3D("module:RobotModelProvider:joints", next.translation.x, next.translation.y, next.translation.z, p.x, p.y, p.z, axisLineWidth, color);
    }
  });
}


MAKE_MODULE(RobotModelProvider, Sensing)
