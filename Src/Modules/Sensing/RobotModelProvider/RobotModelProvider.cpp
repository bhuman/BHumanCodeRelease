/**
 * @file RobotModelProvider.cpp
 *
 * This file implements a module that provides information about the current state of the robot's limbs.
 *
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 */

#include "RobotModelProvider.h"
#include "Tools/Debugging/DebugDrawings3D.h"

MAKE_MODULE(RobotModelProvider, sensing)

void RobotModelProvider::update(RobotModel& robotModel)
{
  robotModel.setJointData(theJointAngles, theRobotDimensions, theMassCalibration);

  DEBUG_DRAWING3D("module:RobotModelProvider:massOffsets", "robot")
  {
    for(int i = 0; i < Limbs::numOfLimbs; ++i)
    {
      const Vector3f v = robotModel.limbs[i] * theMassCalibration.masses[i].offset;
      SPHERE3D("module:RobotModelProvider:massOffsets", v.x(), v.y(), v.z(), 3, ColorRGBA(0, 200, 0));
    }
  }

  DEBUG_DRAWING3D("module:RobotModelProvider:joints", "robot")
  {
    const float axisLineWidth = 1.f;

    for(int i = 0; i < 2; ++i)
    {
      int firstJoint = i == 0 ? Limbs::pelvisLeft : Limbs::pelvisRight;
      for(int i = 0; i < (Limbs::footLeft + 1) - Limbs::pelvisLeft; ++i)
      {
        Pose3f& next = robotModel.limbs[firstJoint + i];
        SPHERE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), 3, ColorRGBA::black);

        Vector3f axis  = i == 0 ? Vector3f(0.f, firstJoint == Limbs::pelvisLeft ? -50.f : 50.f, 50.f).normalize(50.f) : i == 1 || i == 5 ? Vector3f(50.f, 0.f, 0.f) : Vector3f(0.f, 50.f, 0.f);
        ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : i == 1 || i == 5 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
        Vector3f p = next * axis;
        LINE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), p.x(), p.y(), p.z(), axisLineWidth, color);
      }
    }
    for(int i = 0; i < 2; ++i)
    {
      int firstJoint = i == 0 ? Limbs::shoulderLeft : Limbs::shoulderRight;
      for(int i = 0; i < (Limbs::foreArmLeft + 1) - Limbs::shoulderLeft; ++i)
      {
        Pose3f& next = robotModel.limbs[firstJoint + i];
        SPHERE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), 3, ColorRGBA::black);

        Vector3f axis  = i == 1 || i == 3 ? Vector3f(0.f, 0.f, 50.f) : i == 2 ? Vector3f(50.f, 0.f, 0.f) : Vector3f(0.f, 50.f, 0.f);
        ColorRGBA color = i == 1 || i == 3 ? ColorRGBA(0, 0, 255)      : i == 2 ? ColorRGBA(255, 0, 0)      : ColorRGBA(0, 255, 0);
        Vector3f p = next * axis;
        LINE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), p.x(), p.y(), p.z(), axisLineWidth, color);
      }
    }
    for(int i = 0; i < (Limbs::head + 1) - Limbs::neck; ++i)
    {
      Pose3f& next = robotModel.limbs[Limbs::neck + i];
      SPHERE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), 3, ColorRGBA::black);

      Vector3f axis  = i == 0 ? Vector3f(0.f, 0.f, 50.f) : Vector3f(0.f, 50.f, 0.f);
      ColorRGBA color = i == 0 ? ColorRGBA(0, 0, 255)      : ColorRGBA(0, 255, 0);
      Vector3f p = next * axis;
      LINE3D("module:RobotModelProvider:joints", next.translation.x(), next.translation.y(), next.translation.z(), p.x(), p.y(), p.z(), axisLineWidth, color);
    }
  }
}
