#include "JointDataPrediction.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void JointDataPrediction::draw() const
{
  DECLARE_PLOT("representation:JointDataPrediction:position:lHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:lHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:position:lHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:lKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:lAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:lAnkleRoll");
  DECLARE_PLOT("representation:JointDataPrediction:position:rHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:rHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:position:rHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:rKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:rAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:position:rAnkleRoll");

  DECLARE_PLOT("representation:JointDataPrediction:velocity:lHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:lHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:lHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:lKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:lAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:lAnkleRoll");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:velocity:rAnkleRoll");

  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:lAnkleRoll");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rHipYawPitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rHipRoll");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rHipPitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rKneePitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rAnklePitch");
  DECLARE_PLOT("representation:JointDataPrediction:acceleration:rAnkleRoll");

  PLOT("representation:JointDataPrediction:velocity:lHipYawPitch",
          velocities[Joints::lHipYawPitch]);
  PLOT("representation:JointDataPrediction:velocity:lHipRoll",
          velocities[Joints::lHipRoll]);
  PLOT("representation:JointDataPrediction:velocity:lHipPitch",
          velocities[Joints::lHipPitch]);
  PLOT("representation:JointDataPrediction:velocity:lKneePitch",
          velocities[Joints::lKneePitch]);
  PLOT("representation:JointDataPrediction:velocity:lAnklePitch",
          velocities[Joints::lAnklePitch]);
  PLOT("representation:JointDataPrediction:velocity:lAnkleRoll",
          velocities[Joints::lAnkleRoll]);
  PLOT("representation:JointDataPrediction:velocity:rHipYawPitch",
          velocities[Joints::rHipYawPitch]);
  PLOT("representation:JointDataPrediction:velocity:rHipRoll",
          velocities[Joints::rHipRoll]);
  PLOT("representation:JointDataPrediction:velocity:rHipPitch",
          velocities[Joints::rHipPitch]);
  PLOT("representation:JointDataPrediction:velocity:rKneePitch",
          velocities[Joints::rKneePitch]);
  PLOT("representation:JointDataPrediction:velocity:rAnklePitch",
          velocities[Joints::rAnklePitch]);
  PLOT("representation:JointDataPrediction:velocity:rAnkleRoll",
          velocities[Joints::rAnkleRoll]);

  PLOT("representation:JointDataPrediction:position:lHipYawPitch",
          angles[Joints::lHipYawPitch]);
  PLOT("representation:JointDataPrediction:position:lHipRoll",
          angles[Joints::lHipRoll]);
  PLOT("representation:JointDataPrediction:position:lHipPitch",
          angles[Joints::lHipPitch]);
  PLOT("representation:JointDataPrediction:position:lKneePitch",
          angles[Joints::lKneePitch]);
  PLOT("representation:JointDataPrediction:position:lAnklePitch",
          angles[Joints::lAnklePitch]);
  PLOT("representation:JointDataPrediction:position:lAnkleRoll",
          angles[Joints::lAnkleRoll]);
  PLOT("representation:JointDataPrediction:position:rHipYawPitch",
          angles[Joints::rHipYawPitch]);
  PLOT("representation:JointDataPrediction:position:rHipRoll",
          angles[Joints::rHipRoll]);
  PLOT("representation:JointDataPrediction:position:rHipPitch",
          angles[Joints::rHipPitch]);
  PLOT("representation:JointDataPrediction:position:rKneePitch",
          angles[Joints::rKneePitch]);
  PLOT("representation:JointDataPrediction:position:rAnklePitch",
          angles[Joints::rAnklePitch]);
  PLOT("representation:JointDataPrediction:position:rAnkleRoll",
          angles[Joints::rAnkleRoll]);

  PLOT("representation:JointDataPrediction:acceleration:lHipYawPitch",
          accelerations[Joints::lHipYawPitch]);
  PLOT("representation:JointDataPrediction:acceleration:lHipRoll",
          accelerations[Joints::lHipRoll]);
  PLOT("representation:JointDataPrediction:acceleration:lHipPitch",
          accelerations[Joints::lHipPitch]);
  PLOT("representation:JointDataPrediction:acceleration:lKneePitch",
          accelerations[Joints::lKneePitch]);
  PLOT("representation:JointDataPrediction:acceleration:lAnklePitch",
          accelerations[Joints::lAnklePitch]);
  PLOT("representation:JointDataPrediction:acceleration:lAnkleRoll",
          accelerations[Joints::lAnkleRoll]);
  PLOT("representation:JointDataPrediction:acceleration:rHipYawPitch",
          accelerations[Joints::rHipYawPitch]);
  PLOT("representation:JointDataPrediction:acceleration:rHipRoll",
          accelerations[Joints::rHipRoll]);
  PLOT("representation:JointDataPrediction:acceleration:rHipPitch",
          accelerations[Joints::rHipPitch]);
  PLOT("representation:JointDataPrediction:acceleration:rKneePitch",
          accelerations[Joints::rKneePitch]);
  PLOT("representation:JointDataPrediction:acceleration:rAnklePitch",
          accelerations[Joints::rAnklePitch]);
  PLOT("representation:JointDataPrediction:acceleration:rAnkleRoll",
          accelerations[Joints::rAnkleRoll]);

  DECLARE_DEBUG_DRAWING3D("representation:JointDataPrediction:com", "robot");
  CROSS3D("representation:JointDataPrediction:com", com.x(), com.y(), com.z(),
          50, 50, ColorRGBA::red);
  LINE3D("representation:JointDataPrediction:com", com.x(), com.y(), com.z(),
         com.x() + comAcceleration.x(), com.y() + comAcceleration.y(),
         com.z() + comAcceleration.z(), 10, ColorRGBA::red);

  DECLARE_PLOT("representation:JointDataPrediction:comAccX");
  DECLARE_PLOT("representation:JointDataPrediction:comAccY");
  DECLARE_PLOT("representation:JointDataPrediction:comAccZ");
  DECLARE_PLOT("representation:JointDataPrediction:comX");
  DECLARE_PLOT("representation:JointDataPrediction:comY");
  DECLARE_PLOT("representation:JointDataPrediction:comAtan2");
  DECLARE_PLOT("representation:JointDataPrediction:comAccAtan2");
  PLOT("representation:JointDataPrediction:comAccX", comAcceleration.x());
  PLOT("representation:JointDataPrediction:comAccY", comAcceleration.y());
  PLOT("representation:JointDataPrediction:comAccZ", comAcceleration.z());
  PLOT("representation:JointDataPrediction:comAccAtan2", atan2(comAcceleration.y(), comAcceleration.x()));
  PLOT("representation:JointDataPrediction:comX", com.x());
  PLOT("representation:JointDataPrediction:comY", com.y());
  PLOT("representation:JointDataPrediction:comAtan2", atan2(com.y(), com.x()));

  DECLARE_PLOT("representation:JointDataPrediction:angleSum");
  float sum = 0.0f;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    sum += angles[i];
  }
  PLOT("representation:JointDataPrediction:angleSum", sum);

  DECLARE_PLOT("representation:JointDataPrediction:squareVelSum");
  float velSum = 0.0f;
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    velSum += velocities[i] * velocities[i];
  }
  PLOT("representation:JointDataPrediction:squareVelSum", velSum);
}
