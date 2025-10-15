/**
 * @file BoosterFsrSensorDataProvider.cpp
 *
 * This file implements a module that computes fake FSR readings
 * from the deviations of the joints.
 *
 * @author Thomas Röfer
 */

#include "BoosterFsrSensorDataProvider.h"
#include "Debugging/Plot.h"
#include "Math/BHMath.h"

MAKE_MODULE(BoosterFsrSensorDataProvider);

#define D(joint) DECLARE_PLOT("module:BoosterFsrSensorDataProvider:" #joint)
#define P(joint) PLOT("module:BoosterFsrSensorDataProvider:" #joint, diff[Joints::joint].toDegrees())

void BoosterFsrSensorDataProvider::update(FsrSensorData& theFsrSensorData)
{
  D(lHipPitch);
  D(lHipRoll);
  D(lHipYaw);
  D(lKneePitch);
  D(lAnklePitch);
  D(lAnkleRoll);
  D(rHipPitch);
  D(rHipRoll);
  D(rHipYaw);
  D(rKneePitch);
  D(rAnklePitch);
  D(rAnkleRoll);

  DECLARE_PLOT("module:BoosterFsrSensorDataProvider:lSum");
  DECLARE_PLOT("module:BoosterFsrSensorDataProvider:rSum");

  // Remember the offsets that were requested in addition to the joint angles.
  buffer.push_front(theModifiedJointRequest.angles);
  FOREACH_ENUM(Joints::Joint, joint)
    buffer.front()[joint] *= theJointRequest.stiffnessData.stiffnesses[joint] * 0.01f;

  if(buffer.full())
  {
    // Compute deviations per joint.
    Angle diff[Joints::numOfJoints];
    FOREACH_ENUM(Joints::Joint, joint)
      if(theJointPlay.jointState[joint].lastExecutedRequest != JointAngles::off)
        diff[joint] = theJointAngles.angles[joint] - theJointPlay.jointState[joint].lastExecutedRequest - buffer.back()[joint];
      else
        diff[joint] = 0_deg;

    // Compute accumulated deviations per leg.
    Angle sum[Legs::numOfLegs] = {0_deg, 0_deg};
    FOREACH_ENUM(Legs::Leg, leg)
    {
      for(int joint = Joints::firstLeftLegJoint + leg * 6,
              kneeJoint = joint + Joints::lKneePitch - Joints::firstLeftLegJoint,
              lastJoint = joint + 4;
          joint < lastJoint; ++joint)
        if(Joints::canNegate(static_cast<Joints::Joint>(joint), true))
          sum[leg] += std::abs(diff[joint]);
        else if(joint != kneeJoint)
          sum[leg] -= diff[joint];
        else // knee pitch
          sum[leg] += diff[joint];
    }

    // Compute pseudo totals.
    const Rangea angleRange(std::min(baseRange.min, std::min(sum[Legs::left], sum[Legs::right])),
                            std::max(baseRange.max, std::max(sum[Legs::left], sum[Legs::right])));
    float totals[Legs::numOfLegs];
    float totalSum = 0.f;
    FOREACH_ENUM(Legs::Leg, leg)
    {
      totals[leg] = mapToRange(sum[leg], angleRange.min, angleRange.max, 0_rad, 1_rad);
      totalSum += totals[leg];
    }

    // Fill representation.
    FOREACH_ENUM(Legs::Leg, leg)
    {
      float& total = theFsrSensorData.totals[leg];
      total = totalSum == 0.f ? 0.f : totals[leg] / totalSum * theMassCalibration.totalMass * 0.001f;
      const float forward = mapToRange(diff[leg ? Joints::rAnklePitch : Joints::lAnklePitch], -maxAngle, maxAngle, 1_rad, 0_rad); // @todo check sign
      const float left = mapToRange(diff[leg ? Joints::rAnkleRoll : Joints::lAnkleRoll], -maxAngle, maxAngle, 0_rad, 1_rad); // @todo check sign
      auto& pressures = theFsrSensorData.pressures[leg];
      pressures[FsrSensors::fl] = total * forward * left;
      pressures[FsrSensors::fr] = total * forward * (1.f - left);
      pressures[FsrSensors::bl] = total * (1.f - forward) * left;
      pressures[FsrSensors::br] = total * (1.f - forward) * (1.f - left);
    }

    // Some plots
    P(lHipPitch);
    P(lHipRoll);
    P(lHipYaw);
    P(lKneePitch);
    P(lAnklePitch);
    P(lAnkleRoll);
    P(rHipPitch);
    P(rHipRoll);
    P(rHipYaw);
    P(rKneePitch);
    P(rAnklePitch);
    P(rAnkleRoll);
    PLOT("module:BoosterFsrSensorDataProvider:lSum", sum[Legs::left].toDegrees());
    PLOT("module:BoosterFsrSensorDataProvider:rSum", sum[Legs::right].toDegrees());
  }
}
